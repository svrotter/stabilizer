use crate::{
    hardware::dac::DacCode, hardware::design_parameters::TIMER_FREQUENCY,
};
use core::convert::TryFrom;
use miniconf::Miniconf;
use serde::{Deserialize, Serialize};

/// Types of signals that can be generated.
#[derive(Copy, Clone, Debug, Serialize, Deserialize, Miniconf, PartialEq)]
pub enum Signal {
    Sine,
    Cosine,
    Square,
    Triangle,
}
#[derive(Copy, Clone, Debug, Serialize, Deserialize, Miniconf, PartialEq)]
pub enum Trigger {
    PeakMin,
    PeakMax,
    Rising,
    Falling,
    None,
}
/// Basic configuration for a generated signal.
///
/// # Miniconf
/// `{"signal": <signal>, "frequency", 1000.0, "symmetry": 0.5, "amplitude": 1.0}`
///
/// Where `<signal>` may be any of [Signal] variants, `frequency` specifies the signal frequency
/// in Hertz, `symmetry` specifies the normalized signal symmetry which ranges from 0 - 1.0, and
/// `amplitude` specifies the signal amplitude in Volts.
#[derive(Copy, Clone, Debug, Miniconf, Deserialize)]
pub struct BasicConfig {
    /// The signal type that should be generated. See [Signal] variants.
    pub signal: Signal,

    /// The frequency of the generated signal in Hertz.
    pub frequency: f32,

    /// The normalized symmetry of the signal. At 0% symmetry, the duration of the first half oscillation is minimal.
    /// At 25% symmetry, the first half oscillation lasts for 25% of the signal period. For square wave output this
    /// symmetry is the duty cycle.
    pub symmetry: f32,

    /// The amplitude of the output signal in volts.
    pub amplitude: f32,

    /// Voltage offset in Volt
    pub offset: f32,

    /// Phase offset in degree
    pub phase_offset_deg: f32
}

impl Default for BasicConfig {
    fn default() -> Self {
        Self {
            frequency: 1.0e3,
            symmetry: 0.5,
            signal: Signal::Cosine,
            amplitude: 0.0,
            phase_offset_deg: 0.0,
            offset: 0.0,
        }
    }
}

/// Represents the errors that can occur when attempting to configure the signal generator.
#[derive(Copy, Clone, Debug)]
pub enum Error {
    /// The provided amplitude is out-of-range.
    InvalidAmplitude,
    /// The provided symmetry is out of range.
    InvalidSymmetry,
    /// The provided frequency is out of range.
    InvalidFrequency,
    /// The provided voltage offset with given amplitude is out of range.
    InvalidOffset,
    /// The provided phase offset is out of range.
    InvalidPhaseOffset,
}

impl BasicConfig {
    /// Convert configuration into signal generator values.
    ///
    /// # Args
    /// * `sample_ticks` - The number of timer sample ticks between each sample.
    pub fn try_into_config(
        self,
        sample_ticks: u32,
    ) -> Result<Config, Error> {
        let symmetry_complement = 1.0 - self.symmetry;
        // Validate symmetry
        if self.symmetry < 0.0 || symmetry_complement < 0.0 {
            return Err(Error::InvalidSymmetry);
        }

        let fdac = ( (TIMER_FREQUENCY.0 * 1_000_000) as f32) / (sample_ticks as f32);
        let fratio = self.frequency/fdac;
        let ftw_div2 = fratio*( (1u64 << 31) as f32);

        // Validate base frequency tuning word to be below Nyquist.
        const NYQUIST: f32 = (1u32 << 31) as _;
        if ftw_div2 < 0.0 || 2.0 * ftw_div2 > NYQUIST {
            return Err(Error::InvalidFrequency);
        }

        // Calculate the frequency tuning words.
        // Clip both frequency tuning words to within Nyquist before rounding.
        let frequency_tuning_word = [
            if self.symmetry * NYQUIST > ftw_div2 {
                ftw_div2 / self.symmetry
            } else {
                NYQUIST
            } as i32,
            if symmetry_complement * NYQUIST > ftw_div2 {
                ftw_div2 / symmetry_complement
            } else {
                NYQUIST
            } as i32,
        ];

        // Validate maximum/minimum output (offset)
        let max_output: f32 = self.amplitude + self.offset;
        let min_output: f32 = self.offset - self.amplitude;
        if min_output < -10.0 || max_output > 10.0 {
            return Err(Error::InvalidOffset);
        }    

        // Validate phase offset
        if self.phase_offset_deg < -360.0 || self.phase_offset_deg > 360.0{
            return Err(Error::InvalidPhaseOffset);    
        }
        let phase_offset: i64;
        // Potentially occuring rounding errors should be neglible and are ignored 
        phase_offset = ( self.phase_offset_deg/360.0 * 4294967295.0 ) as i64;
        

        Ok(Config {
            amplitude: DacCode::try_from(self.amplitude)
                .or(Err(Error::InvalidAmplitude))?
                .into(),
            offset: DacCode::try_from(self.offset)
            .or(Err(Error::InvalidOffset))?
            .into(),
            signal: self.signal,
            frequency_tuning_word,
            phase_offset,
        })
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Config {
    /// The type of signal being generated
    pub signal: Signal,

    /// The full-scale output code of the signal
    pub amplitude: i16,

    /// DC offset output code of the signal 
    pub offset: i16,

    /// The frequency tuning word of the signal. Phase is incremented by this amount
    pub frequency_tuning_word: [i32; 2],

    /// Signal phase offset. Phase accumulator is increased/decrease by this amount 
    pub phase_offset: i64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            signal: Signal::Cosine,
            amplitude: 0,
            offset: 0,
            frequency_tuning_word: [0, 0],
            phase_offset: 0,
        }
    }
}

#[derive(Debug)]
pub struct SignalGenerator {
    phase_accumulator: i32,
    config: Config,
}

impl Default for SignalGenerator {
    fn default() -> Self {
        Self {
            config: Config::default(),
            phase_accumulator: 0,
        }
    }
}

impl SignalGenerator {
    /// Construct a new signal generator with some specific config.
    ///
    /// # Args
    /// * `config` - The config to use for generating signals.
    ///
    /// # Returns
    /// The generator
    pub fn new(config: Config) -> Self {
        Self {
            config,
            phase_accumulator: 0,
        }
    }

    /// Update waveform generation settings.
    pub fn update_waveform(&mut self, new_config: Config) {
        self.config = new_config;
    }

    /// Checks if signal_generator crossed zero phase recently.
    /// Tested only for symmetric signals.
    pub fn triggered(&self, trigger: Trigger) -> bool {
        // return true if no trigger type is selected
        if trigger == Trigger::None{
            return true;
        }

        // calculate current phase accumulator value including phase offset
        let mut phase_accu_shifted = self
                .phase_accumulator
                .wrapping_add( (self.config.phase_offset>>1) as i32 )
                .wrapping_add( (self.config.phase_offset>>1) as i32 );

        // set phase accu shifted correctly for correct recognition depending on signal type
        match self.config.signal {
            Signal::Sine => {
                if (trigger == Trigger::PeakMin) || (trigger == Trigger::PeakMax) {
                    phase_accu_shifted = phase_accu_shifted.wrapping_add( (1<<30) as i32 );
                } 
            }
            Signal::Cosine =>{
                if (trigger == Trigger::Rising) || (trigger == Trigger::Falling) {
                    phase_accu_shifted = phase_accu_shifted.wrapping_add(  (1<<30) as i32 );
                } 
                if (trigger == Trigger::PeakMin) || (trigger == Trigger::PeakMax) {
                    phase_accu_shifted = phase_accu_shifted.wrapping_add(  (1<<31) as i32 );
                } 
            }
            Signal::Square =>{
                if (trigger == Trigger::PeakMin) || (trigger == Trigger::PeakMax) {
                    phase_accu_shifted = phase_accu_shifted.wrapping_add(  (1<<30) as i32 );
                } 
            }
            Signal::Triangle=>{
                if (trigger == Trigger::Rising) || (trigger == Trigger::Falling) {
                    phase_accu_shifted = phase_accu_shifted.wrapping_sub(  (1<<30) as i32 );
                } 
            }
        }

        // calculate next phase accumulator value to compare it with current 
        let sign = phase_accu_shifted.is_negative();
        let phase_accu_next = phase_accu_shifted
                                    .wrapping_add(self.config.frequency_tuning_word[sign as usize]);
        // determine if trigger occured
        match trigger {
            Trigger::Rising => {
                if (phase_accu_next >= 0) && (phase_accu_shifted < 0) {
                    return true;            
                }
            }
            Trigger::Falling => {
                if (phase_accu_next <= 0) && (phase_accu_shifted > 0) {
                    return true;            
                }
            }
            Trigger::PeakMax =>{
                if (phase_accu_next <= 0) && (phase_accu_shifted > 0) {
                    return true;            
                }
            }
            Trigger::PeakMin =>{
                if (phase_accu_next >= 0) && (phase_accu_shifted < 0) {
                    return true;            
                }
            }
            _ => (),
        }  
        return false;      
    }
}

impl core::iter::Iterator for SignalGenerator {
    type Item = i16;

    /// Get the next value in the generator sequence.
    fn next(&mut self) -> Option<i16> {

        // Add phase_offset divided by 2 twice as its max value is +/-2^32.
        // Rounding occuring for odd phase_offset should be neglible and are ignored 
        let mut phase_accu_shifted = self
                .phase_accumulator
                .wrapping_add( (self.config.phase_offset>>1) as i32 )
                .wrapping_add( (self.config.phase_offset>>1) as i32 );

        let sign = phase_accu_shifted.is_negative();
        phase_accu_shifted = phase_accu_shifted
            .wrapping_add(self.config.frequency_tuning_word[sign as usize]);

        let scale = match self.config.signal {
            Signal::Sine => (idsp::cossin(phase_accu_shifted).1 >> 16),
            Signal::Cosine => (idsp::cossin(phase_accu_shifted).0 >> 16),
            Signal::Square => {
                if sign {
                    -1 << 15
                } else {
                    1 << 15
                }
            }
            Signal::Triangle => {
                (phase_accu_shifted >> 15).abs() - (1 << 15)
            }
        };

        self.phase_accumulator = self
                    .phase_accumulator
                    .wrapping_add(self.config.frequency_tuning_word[sign as usize]);

        // Calculate the final output result as an i16.
        Some( (((self.config.amplitude as i32 * scale) >> 15) + self.config.offset as i32) as _)
    }
}

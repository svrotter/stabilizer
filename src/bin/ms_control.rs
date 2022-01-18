#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(dead_code)]
//#![deny(warnings)]
#![no_std]
#![no_main]

use core::{
    convert::TryFrom,
    sync::atomic::{fence, Ordering},
};

use stm32h7xx_hal::time::MegaHertz;
use mutex_trait::prelude::*; 
use rtic::time::duration::Extensions; 
use idsp::iir;
use stabilizer::{
    hardware::{self, AFE0, AFE1, adc::{Adc0Input, Adc1Input, AdcCode}, 
    afe::Gain, 
    dac::{Dac0Output, Dac1Output, DacCode}, 
    design_parameters::TIMER_FREQUENCY, 
    hal, 
    signal_generator_sr,
    system_timer::SystemTimer}, 
    net::{
            data_stream::{FrameGenerator, StreamFormat, StreamTarget},
            udp_messages::{MsgFrameGenerator, MessageFormat, MessageTarget},
            miniconf::Miniconf,
            serde::{Deserialize, Serialize},
            telemetry::{Telemetry, TelemetryBuffer},
            NetworkState, NetworkUsers,
        }};
use miniconf::MiniconfAtomic;

// The number of samples in each batch process as a power of 2
const BATCH_SIZE_LOG2: u8 = 3;

// The number of samples in each batch process
const BATCH_SIZE: usize = 1<<BATCH_SIZE_LOG2;

// Number of 100MHz timer ticks between each sample. 
// At 100MHz, 10ns per tick, 208 corresponds to a sampling period of 2.08 uS or 480.769kHz.
const ADC_SAMPLE_TICKS: u32 = 208;

const STREAM_SET_SIZE: usize = 4;
const STREAM_FRAME_PAYLOAD: u16 = 1024;
const BATCHES_PER_FRAME: u16 = (STREAM_FRAME_PAYLOAD as f32/
                                BATCH_SIZE as f32/
                                STREAM_SET_SIZE as f32 /2.0) as u16;

const LUT_SIZE: usize = 32;
const BATCH_PER_LUT: usize = LUT_SIZE>>BATCH_SIZE_LOG2;
const FILTER_COEFF: f32 = 1.0/(LUT_SIZE as f32); 

// The number of cascaded IIR biquads per channel.
const IIR_CASCADE_LENGTH: usize = 1;
const DAC_CTRL_VMIN: f32 = 0.0;
const DAC_CTRL_VMAX: f32 = 10.0;

const LINES_SIZE: usize = 32;

static mut SIGNAL_BUF: [i16; BATCH_SIZE] = [0; BATCH_SIZE];
static mut SEARCH_BUF: [i16; BATCH_SIZE] = [0; BATCH_SIZE];
static mut DEMOD_BUF: [f32; BATCH_SIZE] = [0.0; BATCH_SIZE];
static mut ERROR_BUF: [u16; BATCH_SIZE] = [0; BATCH_SIZE];
static mut STREAM_BUF: [u16; BATCH_SIZE*STREAM_SET_SIZE] = [0; BATCH_SIZE*STREAM_SET_SIZE];
static mut ACCU: [f32; BATCH_PER_LUT] = [0.0; BATCH_PER_LUT];
static mut ACCU_IDX: usize = 0;
static mut STREAM_TRIGGER: bool = false;

#[derive(Copy, Clone, Debug, Serialize, Deserialize, Miniconf, PartialEq)]
pub enum StreamMode {
    Cont = 0,
    Shot = 1,
    Stop = 2,
}

#[derive(Copy, Clone, Debug, Serialize, Deserialize, Miniconf, PartialEq)]
pub enum AppMode {
    Man = 0,
    Ctrl = 1,
    CtrlMan = 2,
    CtrlRef = 3,
    SrchMan = 4,
}

#[derive(Copy, Clone, Debug, Serialize, Deserialize, Miniconf, PartialEq)]
pub enum Data {
    None = 0,
    Mod = 1,
    Demod = 2,
    ErrMod = 3,
    ErrDemod = 4,
    CtrlDac = 5,
    CtrlSig = 6,
    Lines = 7,
}

#[derive(Copy, Clone, Debug, Miniconf, Deserialize)]
pub struct LutConfig{
    size: usize,
    amplitude: f32,
    offset: f32,
    symmetry: f32,
    frequency: f32,
    phase_offset_deg: f32,
    signal: signal_generator_sr::Signal,
}

#[derive(Copy, Clone, Debug, Deserialize, Miniconf)]
pub struct LinesConfig{
    threshold: f32,
    offset: f32,
    hysteresis: f32,
}
#[derive(Copy, Clone, Debug)]
pub struct Lines{
    config: LinesConfig,
    minmax: i8,
    found_line: bool,
    location: [u16; LINES_SIZE],
    location_type: [i8; LINES_SIZE],
    location_idx: usize,
}
impl Lines{
    pub fn new() -> Self {
        Self {
            config: LinesConfig{
                threshold: 0.01,
                offset: 0.0,
                hysteresis: 0.001,
            },
            minmax: 0,
            found_line: false,
            location: [0; LINES_SIZE],
            location_type: [0; LINES_SIZE],
            location_idx: 0,    
        }
    }
}


#[derive(Copy, Clone, Debug, Serialize, Deserialize, MiniconfAtomic)]
pub struct Streams {
    pub stream_set: [Data; STREAM_SET_SIZE],
}


#[derive(Copy, Clone, Debug)]
pub struct Lut{
    data: [i16; LUT_SIZE],
    idx: usize,
    config: LutConfig,
}

impl Lut{
    fn update(&mut self){
        // Create LUT signal generator BasicConfig struct
        let signal_bconfig_lut = signal_generator_sr::BasicConfig {
            signal: self.config.signal,
            amplitude: self.config.amplitude,
            frequency: self.config.frequency,
            symmetry: self.config.symmetry,
            offset: self.config.offset,
            phase_offset_deg: self.config.phase_offset_deg,
        };
        // Convert BasicConfig struct to signal_generator::Config and create
        // LUT signal generator
        let signal_config_lut = signal_bconfig_lut
            .try_into_config(ADC_SAMPLE_TICKS).unwrap();
        let mut signal_generator_lut = signal_generator_sr::SignalGenerator::new(
            signal_config_lut
        );

        // create LUT data
        for i in 0..self.config.size{
            self.data[i] = signal_generator_lut.next().unwrap();
        }  
    }
}

#[derive(Copy, Clone, Debug, Deserialize, Miniconf)]
pub struct Settings {
    /// Configure the Analog Front End (AFE) gain.
    ///
    /// # Path
    /// `afe/<n>`
    ///
    /// * <n> specifies which channel to configure. <n> := [0, 1]
    ///
    /// # Value
    /// Any of the variants of [Gain] enclosed in double quotes.
    afe: [Gain; 2],

    /// Specifies the telemetry output period in seconds.
    ///
    /// # Path
    /// `telemetry_period`
    ///
    /// # Value
    /// Any non-zero value less than 65536.
    telemetry_period: u16,

    msg_target: MessageTarget,
    msg_transmit: bool,
    msg_count: u16,
    msg_request: u16,

    stream_target: StreamTarget,
    stream_batch_request: u32,
    stream_batch_count: u32,
    stream_trigger: signal_generator_sr::Trigger,
    stream_mode: StreamMode,
    stream_request: bool,
    streams: Streams,

    lut_config: [LutConfig; 2],

    signal_generator_ctrl: signal_generator_sr::BasicConfig,

    iir_ctrl: iir::IIR<f32>,
    ctrl_offset: f32,
    ctrl_offset_i16: i16,

    app_mode: AppMode,

    lines_config: LinesConfig,

    commit: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            // Gain for [AFE0, AFE1]
            afe: [Gain::G1, Gain::G1],

            // The default telemetry period in seconds.
            telemetry_period: 0,

            msg_target: MessageTarget{
                ip: [192,168,137,1],
                port: 9934,
            },
            msg_transmit: false,
            msg_count: 0,
            msg_request: 1,

            stream_target: StreamTarget{
                ip: [192,168,137,1],
                port: 9933,
            },
            stream_batch_count: 0,
            stream_batch_request: 0,
            stream_trigger: signal_generator_sr::Trigger::Rising,
            stream_mode: StreamMode::Stop,
            stream_request: false,
            streams: Streams{
                stream_set: [Data::ErrMod, Data::ErrDemod, Data::CtrlDac, Data::Mod]
            },

            lut_config: 
                // modulation LUT
                [LutConfig{
                    size: LUT_SIZE,
                    amplitude: 1.0,
                    offset: 0.0,
                    symmetry: 0.5,
                    frequency: (TIMER_FREQUENCY.0 * 1_000_000) as f32/
                        ( (ADC_SAMPLE_TICKS*LUT_SIZE as u32) as f32 ),
                    phase_offset_deg: 0.0, 
                    signal: signal_generator_sr::Signal::Sine,
                },

                // demodulation LUT
                LutConfig{
                    size: LUT_SIZE,
                    amplitude: 1.0,
                    offset: 0.0,
                    symmetry: 0.5,
                    frequency: (TIMER_FREQUENCY.0 * 1_000_000) as f32/
                        ( (ADC_SAMPLE_TICKS*LUT_SIZE as u32) as f32 ),
                    phase_offset_deg: 50.0, 
                    signal: signal_generator_sr::Signal::Sine,
                }],

            signal_generator_ctrl: signal_generator_sr::BasicConfig{
                signal: signal_generator_sr::Signal::Triangle,
                frequency: 100.0,
                symmetry: 0.5,
                amplitude: 0.2,
                offset: 5.0,
                phase_offset_deg: 0.0,
            },

            // IIR filter tap gains are an array `[b0, b1, b2, a1, a2]` such that the
            // new output is computed as `y0 = a1*y1 + a2*y2 + b0*x0 + b1*x1 + b2*x2`.
            // The array is `iir_state[channel-index][cascade-index][coeff-index]`.
            // The IIR coefficients can be mapped to other transfer function
            // representations, for example as described in https://arxiv.org/abs/1508.06319
            iir_ctrl: iir::IIR::new(1., DAC_CTRL_VMIN, DAC_CTRL_VMAX),
            ctrl_offset: 0.0,
            ctrl_offset_i16: 0,

            app_mode: AppMode::Man,

            lines_config: Lines::new().config,

            commit: false,
        }
    }
}

#[rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, SDMMC])]
mod app {
    use stabilizer::hardware::signal_generator_sr::{BasicConfig, Config};

    use super::*;

    #[monotonic(binds = TIM15)]
    type Monotonic = SystemTimer; 

    #[shared]
    
    struct Shared {
        network: NetworkUsers<Settings, Telemetry>,
        settings: Settings,
        msg_generator: MsgFrameGenerator,
        telemetry: TelemetryBuffer,
        lut: [Lut; 2],
        signal_generator_ctrl: signal_generator_sr::SignalGenerator,
        lines: Lines,
    }

    #[local]
    struct Local {
        afes: (AFE0, AFE1),
        generator: FrameGenerator,
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        signal: &'static mut [i16],
        search: &'static mut [i16],
        demod: &'static mut [f32],
        error: &'static mut [u16],
        stream_buf: &'static mut [u16],
        accu: &'static mut [f32],
        accu_idx: usize,
        iir_state: iir::Vec5<f32>,
        stream_trigger: bool,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Configure the microcontroller
        let (mut stabilizer, _pounder) = hardware::setup::setup(
            c.core,
            c.device,
            BATCH_SIZE,
            ADC_SAMPLE_TICKS,
        );

        let mut network = NetworkUsers::new(
            stabilizer.net.stack,
            stabilizer.net.phy,
            env!("CARGO_BIN_NAME"),
            stabilizer.net.mac_address,
            option_env!("BROKER")
                .unwrap_or("192.168.137.1")
                .parse()
                .unwrap(),
        );

        let msg_generator = network
            .configure_messaging(MessageFormat::Unknown);
        let generator = network
            .configure_streaming(StreamFormat::Unknown, BATCH_SIZE as u8);

        let settings = Settings::default();

        // Enable ADC/DAC events
        stabilizer.adcs.0.start();
        stabilizer.adcs.1.start();
        stabilizer.dacs.0.start();
        stabilizer.dacs.1.start();

        let mut lines_init: Lines = Lines::new();
        lines_init.config = settings.lines_config;
        
        let mut shared = Shared {
            network,
            telemetry: TelemetryBuffer::default(),
            settings,
            msg_generator,
            lut: [Lut{
                data: [0; LUT_SIZE],
                idx: 0,
                config: settings.lut_config[0],
            },
            Lut{
                data: [0; LUT_SIZE],
                idx: 0,
                config: settings.lut_config[1],
            }],

            signal_generator_ctrl: signal_generator_sr::SignalGenerator::new(
                settings.signal_generator_ctrl
                .try_into_config(ADC_SAMPLE_TICKS).unwrap()
            ),
            lines: lines_init,
        };
        shared.lut[0].update();
        shared.lut[1].update();

        shared.network.set_msg_destination(settings.msg_target.into());
        shared.network.direct_stream(settings.stream_target.into());

        let local = Local {
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            generator,
            signal: unsafe{&mut SIGNAL_BUF[0..]},
            search: unsafe{&mut SEARCH_BUF[0..]},
            demod: unsafe{&mut DEMOD_BUF[0..]},
            error: unsafe{&mut ERROR_BUF[0..]},
            stream_buf: unsafe{&mut STREAM_BUF[0..]},
            accu: unsafe{&mut ACCU[0..]},
            accu_idx: unsafe{ACCU_IDX},
            iir_state: [0.; 5],
            stream_trigger: unsafe{STREAM_TRIGGER},
        };
 
        // Spawn a settings and telemetry update for default settings.
        ethernet_link::spawn().unwrap();
        if settings.telemetry_period > 0 {
            telemetry_task::spawn().unwrap();
        }

        // Start sampling ADCs (start as late as possible to prevent processing congestion)
        stabilizer.adc_dac_timer.start();

        (shared, local, init::Monotonics(SystemTimer::default()))
        
    }

    /// Main DSP processing routine.
    ///
    #[task(binds=DMA1_STR4, 
        shared=[settings, telemetry, lut, signal_generator_ctrl,lines], 
        local=[adcs, dacs, generator, search, signal, demod, error, stream_buf, 
                accu, accu_idx, iir_state, stream_trigger], 
        priority=3)]
    #[link_section = ".itcm.process"]
    fn process(c: process::Context) {
        
        let process::SharedResources {
            settings,
            telemetry,
            lut,
            signal_generator_ctrl,
            lines,
        } = c.shared;

        
        let process::LocalResources {
            adcs: (ref mut adc0, ref mut adc1),
            dacs: (ref mut dac0, ref mut dac1),
            generator,
            search,
            signal,
            demod,
            error,
            iir_state,
            stream_buf,
            accu,
            accu_idx,
            stream_trigger,
        } = c.local;

        (settings, telemetry, lut, signal_generator_ctrl, lines).lock(
            |settings, telemetry, lut, signal_generator_ctrl, lines| {
                (adc1, adc0, dac0, dac1).lock(|adc1, adc0, dac0, dac1| {

                    // feed dac1 buffer with new modulation values
                    for i in 0..BATCH_SIZE{
                        dac1[i] = DacCode::from(lut[0].data[lut[0].idx+i]).0;
                    }
                    
                    // demodulate ADC0 batch data and calculate batch accumulator value 
                    let mut error_b: f32 = 0.0; 
                    
                    accu[*accu_idx] = 0.0;
                    for i in 0..BATCH_SIZE{
                        // convert ADC data to voltage float 
                        let adc0_i: f32 = f32::from(AdcCode(adc0[i]));
                        // convert demod LUT entry to voltage float
                        let lut_demod_i: f32 = f32::from(DacCode::from(lut[1].data[lut[1].idx+i]));
                        // demodulate and add to FIR filter batch accumulator
                        demod[i] = adc0_i*lut_demod_i;
                        accu[*accu_idx] += FILTER_COEFF*demod[i]; 
                    }
                    // sum all batch FIR batch accumulator values
                    for i in 0..BATCH_PER_LUT{
                        error_b += accu[i];
                    }
                
                    match settings.app_mode {
                        AppMode::Man => {
                            // feed dac0 buffer with new manual ctrl values and check if trigger occured on ctrl signal
                            for i in 0..BATCH_SIZE{
                                signal[i] = signal_generator_ctrl.next().unwrap()+settings.ctrl_offset_i16;
                                dac0[i] = DacCode::from(signal[i]).0;
                                
                                let trigger = signal_generator_ctrl.triggered(settings.stream_trigger);
                                if trigger && (settings.stream_batch_count > 0) {
                                    *stream_trigger = true;
                                }
                            }
                        }
                        AppMode::Ctrl => {   
                            let u=settings.iir_ctrl.update(iir_state, error_b, false);
                            let u_i16: i16 = (u * (i16::MAX as f32 / (2.5*4.096))) as i16;          
                            for i in 0..BATCH_SIZE {
                                dac0[i] = DacCode::from(u_i16+settings.ctrl_offset_i16).0;
                            }
                            *stream_trigger = true;
                        }
                        AppMode::CtrlMan => {
                            let u=settings.iir_ctrl.update(iir_state, error_b, false);
                            let u_i16: i16 = (u * (i16::MAX as f32 / (2.5*4.096))) as i16; 
                            
                            for i in 0..BATCH_SIZE{
                                signal[i] = signal_generator_ctrl.next().unwrap();
                                dac0[i] = DacCode::from(u_i16+settings.ctrl_offset_i16+signal[i]).0;
                                
                                let trigger = signal_generator_ctrl.triggered(settings.stream_trigger);
                                if trigger && (settings.stream_batch_count > 0) {
                                    *stream_trigger = true;
                                }
                            }                           
                        }
                        AppMode::CtrlRef => { 
                            let signal_generator_i: f32;
                            for i in 0..BATCH_SIZE{
                                signal[i] = signal_generator_ctrl.next().unwrap();

                                let trigger = signal_generator_ctrl.triggered(settings.stream_trigger);
                                if trigger && (settings.stream_batch_count > 0) {
                                    *stream_trigger = true;
                                }
                            }
                            signal_generator_i = f32::from(DacCode::from(signal[0]));
                            let u=settings.iir_ctrl.update(iir_state,
                                signal_generator_i - error_b, false);
                            let u_i16: i16 = (u * (i16::MAX as f32 / (2.5*4.096))) as i16; 
                            
                            for i in 0..BATCH_SIZE{
                                dac0[i] = DacCode::from(u_i16+settings.ctrl_offset_i16).0;
                            }                           
                        }
                        AppMode::SrchMan => {
                            let old_dac = dac0[0];
                            for i in 0..BATCH_SIZE{
                                signal[i] = signal_generator_ctrl.next().unwrap()+settings.ctrl_offset_i16;
                                dac0[i] = DacCode::from(signal[i]).0;
                                
                                let trigger = signal_generator_ctrl.triggered(settings.stream_trigger);
                                if trigger && (settings.stream_batch_count > 0) {
                                    *stream_trigger = true;
                                }
                            }   

                            if *stream_trigger{
                                if (error_b+lines.config.offset) > (lines.config.threshold+lines.config.hysteresis){
                                    if lines.minmax == 0 {
                                        lines.minmax = 1;
                                    }
                                    else {
                                        lines.found_line = false;
                                    }
                                    for i in 0..BATCH_SIZE{
                                        search[i] = 1;
                                    }
                                }
                                else if (error_b+lines.config.offset) < -(lines.config.threshold+lines.config.hysteresis){
                                    if lines.minmax == 0 {
                                        lines.minmax = -1;
                                    }
                                    else {
                                        lines.found_line = false;
                                    }
                                    for i in 0..BATCH_SIZE{
                                        search[i] = -1;
                                    }
                                }
                                else if idsp::abs(error_b+lines.config.offset) < (lines.config.threshold-lines.config.hysteresis) {
                                    if !lines.found_line & (lines.minmax == 2) {
                                        lines.minmax = 0;
                                    }
                                    for i in 0..BATCH_SIZE{
                                        search[i] = 0;
                                    }    
                                
                                }

                                match lines.minmax{
                                    -1 => {
                                        if error_b > 0.0 {
                                            lines.found_line = true;
                                            if *stream_trigger{
                                                lines.location_type[lines.location_idx] = -1;
                                                lines.location[lines.location_idx] = dac0[0];
                                                lines.location_idx += 1;
                                                if lines.location_idx == LINES_SIZE{
                                                    lines.location_idx = 0;
                                                }

                                                lines.minmax = 2;
                                                for i in 0..BATCH_SIZE{
                                                    search[i] = 2;
                                                }
                                            }
                                        }
                                    }
                                    1 => {
                                        if error_b < 0.0 {
                                            lines.found_line = true;
                                            if *stream_trigger{
                                                lines.location_type[lines.location_idx] = -1;
                                                lines.location[lines.location_idx] = dac0[0];
                                                lines.location_idx += 1;
                                                if lines.location_idx == LINES_SIZE{
                                                    lines.location_idx = 0;
                                                }
                                                lines.minmax = 2;
                                                for i in 0..BATCH_SIZE{
                                                    search[i] = 2;
                                                }
                                            }
                                        }
                                    }
                                    _ => ()
                                }
                            }    
                        }
                    }
                    
                    // stretch decimate error to batch length and convert for streaming
                    let error_i16: i16 = (error_b * (i16::MAX as f32 / (2.5*4.096))) as i16;
                    for i in 0..BATCH_SIZE{ 
                        error[i] = error_i16 as u16;
                    }

                    // Preserve instruction and data ordering w.r.t. DMA flag access.
                    fence(Ordering::SeqCst);

                    for i in 0..STREAM_SET_SIZE {
                        match settings.streams.stream_set[i] {
                            Data::None=>{
                                for j in 0..BATCH_SIZE {
                                    stream_buf[i*BATCH_SIZE+j] = 0;
                                }
                            }
                            Data::Mod => {
                                for j in 0..BATCH_SIZE {
                                    stream_buf[i*BATCH_SIZE+j] = dac1[j];
                                }    
                            }
                            Data::Demod => {
                                for j in 0..BATCH_SIZE {
                                    stream_buf[i*BATCH_SIZE+j] = DacCode::from(lut[1].data[lut[1].idx+j]).0;
                                }
                            }
                            Data::ErrMod => {
                                for j in 0..BATCH_SIZE {
                                    stream_buf[i*BATCH_SIZE+j] = adc0[j];
                                }  
                            }
                            Data::ErrDemod => {
                                for j in 0..BATCH_SIZE {
                                    stream_buf[i*BATCH_SIZE+j] = error[j];
                                }  
                            }
                            Data::CtrlDac => {
                                for j in 0..BATCH_SIZE {
                                    stream_buf[i*BATCH_SIZE+j] = dac0[j];
                                }  
                            }
                            Data::CtrlSig => {
                                for j in 0..BATCH_SIZE {
                                    stream_buf[i*BATCH_SIZE+j] = DacCode::from(signal[j]).0;
                                }    
                            }
                            Data::Lines => {
                                for j in 0..BATCH_SIZE {
                                    stream_buf[i*BATCH_SIZE+j] = search[j] as u16;
                                }    
                            }
                        }
                    }

                    // prepare indeces for next batch
                    *accu_idx += 1;
                    lut[0].idx += BATCH_SIZE;
                    lut[1].idx += BATCH_SIZE;
                    if *accu_idx == BATCH_PER_LUT {
                        *accu_idx = 0;
                        lut[0].idx = 0;
                        lut[1].idx = 0;
                    }

                    if settings.stream_batch_count > 0 {
                        // Stream the data.
                        if *stream_trigger{
                            const N: usize = BATCH_SIZE * core::mem::size_of::<u16>();
                            generator.add::<_, { N * STREAM_SET_SIZE }>(|buf| {
                                let data = unsafe {
                                    core::slice::from_raw_parts(
                                        stream_buf.as_ptr() as *const u8,
                                        N*STREAM_SET_SIZE,
                                    )
                                };
                                buf.copy_from_slice(data)
                            });
                            
                            settings.stream_batch_count-=1;
                        }
                        // reset the trigger if this was last batch
                        if settings.stream_batch_count == 0 {
                            match settings.stream_mode {
                                StreamMode::Shot => {
                                    *stream_trigger = false;
                                }
                                StreamMode::Cont => { 
                                    *stream_trigger = false;    
                                }
                                StreamMode::Stop => { 
                                    *stream_trigger = false;
                                    settings.stream_batch_request = 0;
                                }
                            }
                        }
                    }

                    if settings.telemetry_period > 0 {
                        //Update telemetry measurements.
                        telemetry.adcs = [ 
                            AdcCode(adc0[0]),
                            AdcCode(adc1[0]),
                        ];

                        telemetry.dacs = [
                            DacCode(dac0[0]),
                            DacCode(dac1[0]),
                        ];
                    }

                    // Preserve instruction and data ordering w.r.t. DMA flag access.
                    fence(Ordering::SeqCst);
                });
            },
        );
    }

    #[idle(shared=[network])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            // poll for settings changes and process streaming and messages
            match c.shared.network.lock(|net| net.update()) {
                NetworkState::SettingsChanged => {
                    settings_update::spawn().unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => cortex_m::asm::wfi(),
            }
        }
    }

    #[task(priority = 1, local=[afes], shared=[network, settings, lut, signal_generator_ctrl, lines])]
    fn settings_update(mut c: settings_update::Context) {
        let settings = c.shared.network.lock(|net| *net.miniconf.settings());  
        if settings.commit {

            c.local.afes.0.set_gain(settings.afe[0]);
            c.local.afes.1.set_gain(settings.afe[1]);

            let target = settings.stream_target.into();
            c.shared.network.lock(|net| net.direct_stream(target));
            let msg_target = settings.msg_target.into();
            c.shared.network.lock(|net| net.set_msg_destination(msg_target));

            let mut lut_temp: [Lut; 2] =
                [Lut{
                    data: [0; LUT_SIZE],
                    idx: 0,
                    config: settings.lut_config[0],
                },
                Lut{
                    data: [0; LUT_SIZE],
                    idx: 0,
                    config: settings.lut_config[1],
                }];
            lut_temp[0].update();
            lut_temp[1].update();
            
            c.shared.lut.lock(|lut| {
                lut[0].data = lut_temp[0].data;
                lut[1].data = lut_temp[1].data;
            });

            // Update the signal generator
            c.shared.signal_generator_ctrl.lock(|sg_ctrl|{
                let sg_config = settings.signal_generator_ctrl
                        .try_into_config(ADC_SAMPLE_TICKS).unwrap();    
                sg_ctrl.update_waveform(sg_config);
            });

            // Update the line search parameter
            c.shared.lines.lock(|lines| {
                lines.config.threshold = settings.lines_config.threshold;
                lines.config.offset = settings.lines_config.offset;
                lines.config.hysteresis = settings.lines_config.hysteresis;
            });

            c.shared.settings.lock(|current| {
                // Respawn telemetry task if needed
                if current.telemetry_period == 0 {
                    if settings.telemetry_period > 0 {
                        telemetry_task::spawn().unwrap();
                    }
                }
                *current = settings;
                current.ctrl_offset_i16 = (settings.ctrl_offset * (i16::MAX as f32 / (2.5*4.096))) as i16;

            });
        }
        if settings.stream_request{
            // round batch request to integer number of frames
            let stream_breq = (settings.stream_batch_request/BATCHES_PER_FRAME as u32)
                                            *BATCHES_PER_FRAME as u32;
            c.shared.settings.lock(|current| {    
                current.stream_batch_count = stream_breq;
            });
        }
        if settings.msg_transmit{
            c.shared.settings.lock(|current| {    
                current.msg_count = settings.msg_request;
                send_lines::spawn().unwrap();
            });
        }
    }

    #[task(priority = 1, shared=[network, settings, telemetry])]
    fn telemetry_task(mut c: telemetry_task::Context) {
        let telemetry: TelemetryBuffer =
            c.shared.telemetry.lock(|telemetry| *telemetry);

        let (gains, telemetry_period) = c
            .shared
            .settings
            .lock(|settings| (settings.afe, settings.telemetry_period));

        c.shared.network.lock(|network| {
            network
                .telemetry
                .publish(&telemetry.finalize(gains[0], gains[1]))
        });

        // Schedule the telemetry task in the future.
        if telemetry_period > 0 {
            telemetry_task::Monotonic::spawn_after(
                (telemetry_period as u32).seconds(),
            )
            .unwrap();
        }
    }

    #[task(priority = 1, shared=[network])]
    fn ethernet_link(mut c: ethernet_link::Context) {
        c.shared.network.lock(|net| net.processor.handle_link());
        ethernet_link::Monotonic::spawn_after(1u32.seconds()).unwrap();
    }

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { hal::ethernet::interrupt_handler() }
    }

    #[task(priority = 2, shared=[network, settings, msg_generator, lines])]
    fn send_lines(mut c: send_lines::Context) {
        let mut msg_count: u16 = 0;
        c.shared.settings.lock(| settings |{
            settings.msg_count -= 1;
            msg_count = settings.msg_count
        });
        
        // const LENGTH: usize = 17;
        // let mut dummy_lines: [u16; LINES_SIZE] = [0; LINES_SIZE];
        // for i in 0..LENGTH {
        //     dummy_lines[i] = i as u16;
        // }  

        let mut lines = Lines::new();
        
        c.shared.lines.lock(|lines_shared|{
            lines = *lines_shared;
            lines_shared.location_idx = 0; // todo is that ok?
        });
        //lines.location = dummy_lines; // todo remove this
        //lines.location_idx = LENGTH;

        const N: usize = core::mem::size_of::<u16>()*LINES_SIZE;
        c.shared.msg_generator.lock(|msg_generator|{
            msg_generator.add::<_, { N }>(|buf| {
                let data = unsafe {
                    core::slice::from_raw_parts(
                        lines.location.as_ptr() as *const u8,
                        N,
                    )
                };
                buf.copy_from_slice(data)
            }); 
            
            msg_generator.set_msg_len(lines.location_idx as u16);  
            msg_generator.queue_for_tx();    
        });

        if msg_count > 0{
            send_lines::spawn().unwrap();
        }
    }

}

import os
path = os.path.dirname(os.path.realpath(__file__))
import stabilizer_if


# Create instance of Stabilizer device class and add plot instance
stabilizer = stabilizer_if.stabilizerClass('ms_control')

""" Script flow options """
#execute settings update bat immediately if =1
stabilizer.execute_update = 1
# start streaming data if =1
stabilizer.stream = 1
# plots streamed data if =1
stabilizer.plot_data = 1
# saves data with chosen tag as csv if =1
stabilizer.save_data = 0
stabilizer.save_tag = 'saveload_test'


"""## Stabilizer device settings ##"""
#stabilizer.app_mode = 'Man'
#stabilizer.app_mode = 'Ctrl'
#stabilizer.app_mode = 'CtrlMan'
stabilizer.app_mode = 'CtrlRef'

# data to be streamed (and plotted if enabled)
# availabe options: Mod Demod ErrMod ErrDemod CtrlDac CtrlSig Srch
stabilizer.streams = ['ErrMod','ErrDemod','CtrlDac','CtrlSig']

# stream type
#stabilizer.stream_mode = 'Shot'
stabilizer.stream_mode = 'Cont'
stabilizer.stream_decimation = 1

"""# Stabilizer basic settings #"""
# Stabilizer telemetry period
stabilizer.telemetry_period = 10
# possible gains: G1, G2, G5, G10
stabilizer.gain_afe0 = 'G2'
stabilizer.gain_afe1 = 'G1'

"""# Stabilizer streaming settings #"""
# number of batches to be streamed. Stabilizer only sends full frames,
# so received number of batches may be less.
# one frame contains 16 batches for application ms_control
stabilizer.stream_batch_request = 16*250

# Connection timeout for streaming
stabilizer.timeout = 3

"""# Modulation/Demodulation settings #"""
""" Modulation LUT settings """
# Modulation voltage amplitude
stabilizer.lut_config0_amplitude = 0.5
stabilizer.lut_config0_phase_offset_deg = 0

""" Demodulation LUT settings """
# Demodulation virtual voltage amplitude
stabilizer.lut_config1_amplitude = 1.0
# Demodulation signal phase in deg
stabilizer.lut_config1_phase_offset_deg = 110


"""# VCO control properties #"""
# general control offset, independant of application mode
stabilizer.ctrl_offset = 6.145

""" Control signal portion from signal generator """
# attributes only applicable in app modes Man, CtrlRef and CtrlMan
stabilizer.sig_ctrl_signal = 'Triangle' 
# sweep frequency tuning word must be a divider of 2^32 for perfect alignment
# with modulation signal. Use frequency=1/(10e-9*208)/(2**X)
stabilizer.sig_ctrl_frequency = stabilizer.sampling_freq/(2**15) 
stabilizer.sig_ctrl_amplitude = 0.1
# additional offset to ctrl_offset, only available in app modes 
stabilizer.sig_ctrl_offset = 0
# Trigger in control signal for data recording
stabilizer.sig_ctrl_stream_trigger = 'PeakMin'

""" IIR PID controller """
# attributes only applicable in app modes Ctrl, CtrlRef and CtrlMan
stabilizer.add_iir('iir_ctrl', stabilizer.sampling_freq/stabilizer.batch_size)
stabilizer.iir_ctrl.Kp=-0.1
stabilizer.iir_ctrl.Ki=-100
stabilizer.iir_ctrl.Kd=-5e-05
stabilizer.iir_ctrl.y_min = -2
stabilizer.iir_ctrl.y_max = 2
stabilizer.iir_ctrl.compute_coeff()
stabilizer.iir_ctrl.y_offset = 0
#stabilizer.iir_ctrl.freqz(10000)
 
"""## Stream plot options ##"""
stabilizer.add_plot()
#stabilizer.plot.plots = ['ErrMod','ErrDemod','CtrlDac']
stabilizer.plot.xtype = 'time_us'
# ='auto' or single tuple for all plots e.g. =(0,100)
stabilizer.plot.xlim = 'auto'
# ='auto' or list of tuples with one entry per subplot 
# e.g. 2 plots =[(-1,1), (-1,1)]
stabilizer.plot.ylim = 'auto'
#stabilizer.plot.ylim=[(-0.1,0.1), (-0.1,0.1), (0,10)]
stabilizer.plot.tolerance = 0.2
stabilizer.plot.refresh_ylim = 1

#%%
stabilizer.run()

#%% Post execution processing examples
# manual save
# stabilizer.save('some_tag')

# load and plot example
# s = stabilizer_if.load('ms_control_saveload_test_1.csv')
# s.add_plot()
# s.plot.plots = ['ErrMod','ErrDemod','CtrlDac']
# s.data_from_int()
# s.plot.update()
# fig_idx = 1
#plot_name = 'savedData/testplot'
#stabilizer_if.save_plot(fig_idx,plot_name)
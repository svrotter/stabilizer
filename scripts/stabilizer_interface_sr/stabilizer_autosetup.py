import os
import sys
path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(path)
import stabilizer_if

#todo

# receive udp messages on stabilizer
# implement PRBS
# measure plant dead time


# Create instance of Stabilizer device class and add plot instance
mac = '04-91-62-d9-4c-7f'
broker = '192.168.137.1'
stabilizer = stabilizer_if.stabilizerClass('ms_control', mac, broker)

""" Script flow options """
#execute settings update immediately if =1
stabilizer.execute_update = 1

# start streaming data if =1
stabilizer.stream = 1
# plots streamed data if =1
stabilizer.plot_data = 1
# saves data with chosen tag as csv if =1
stabilizer.save_data = 0
stabilizer.save_tag = 'saveload_test'

"""## Stabilizer device settings ##"""
# ms_control application modes
stabilizer.app_mode = 'Man'
#stabilizer.app_mode = 'Ctrl'
#stabilizer.app_mode = 'CtrlMan'
#stabilizer.app_mode = 'CtrlRef'
#stabilizer.app_mode = 'SrchMan'

"""# Stabilizer basic settings #"""
# Stabilizer telemetry period
stabilizer.telemetry_period = 0
# possible analog front end gains: G1, G2, G5, G10
stabilizer.gain_afe0 = 'G2'
stabilizer.gain_afe1 = 'G1'

"""# Stabilizer streaming settings #"""
# stream type (continuous or single shot)
#stabilizer.stream_mode = 'Shot'
stabilizer.stream_mode = 'Cont'
# stream decimation e.g. every second datapoint is processed if =2
stabilizer.stream_decimation = 1

# data to be streamed
# availabe options: Mod Demod ErrMod ErrDemod CtrlDac CtrlSig Lines
stabilizer.streams = ['ErrMod','Mod','ErrDemod','CtrlDac']

# set the length of data stream in different units 
# This length will be rounded to fit full frames and may be smaller at the end
stabilizer.set_stream_length(500, 'frames')

# Connection timeout for streaming and commands
stabilizer.timeout = 1
stabilizer.list_dopped_frames = 1

"""# Modulation/Demodulation settings #"""
""" Modulation LUT settings """
# Modulation voltage amplitude
stabilizer.lut_config0_amplitude = 1
# Modulation signal phase in deg
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
# with modulation signal. Use frequency=stabilizer.sampling_freq/(2**int(X))
stabilizer.sig_ctrl_frequency = stabilizer.sampling_freq/(2**16) 
stabilizer.sig_ctrl_amplitude = 0.1
stabilizer.sig_ctrl_offset = 0
# Trigger in control signal for data recording
stabilizer.sig_ctrl_stream_trigger = 'PeakMin'

""" IIR PID controller """
# attributes only applicable in app modes Ctrl, CtrlRef and CtrlMan
stabilizer.add_iir('iir_ctrl', stabilizer.sampling_freq/stabilizer.batch_size)
stabilizer.iir_ctrl.Kp=-0.1
stabilizer.iir_ctrl.Ki=-100
stabilizer.iir_ctrl.Kd=-2e-05
stabilizer.iir_ctrl.y_min = -2
stabilizer.iir_ctrl.y_max = 2
stabilizer.iir_ctrl.compute_coeff()
stabilizer.iir_ctrl.y_offset = 0
# plots frequency response of the IIR controller
#stabilizer.iir_ctrl.freqz(10000)
 
""" Line Search """
stabilizer.lines_config_threshold = 0.025
stabilizer.lines_config_offset = 0
stabilizer.lines_config_hysteresis = 0.005

"""## Stream plot options ##"""
stabilizer.add_plot()
# defines which channels of the whole stream should be plotted (default=all)
#stabilizer.plot.plots = ['ErrMod','ErrDemod','CtrlDac']
stabilizer.plot.xtype = 'time_ms'
# ='auto' or single tuple for all plots e.g. =(0,100)
stabilizer.plot.xlim = 'auto'
# ='auto' or list of tuples with one entry per subplot 
# e.g. 2 plots ylim=[(-1,1), (-1,1)]
stabilizer.plot.ylim = 'auto'
# plot y limit tolerances from max/min values when ylim='auto' 
stabilizer.plot.tolerance = 0.2
# freezes the plot y limits if =0
stabilizer.plot.refresh_ylim = 1
# plots a new line for every data set received 
#(should only be used with stream_mode='Cont')
#stabilizer.plot.persistent = True
#stabilizer.plot.persistent_color = True

#%% do autosetup, start the stream and plot if these actions are enabled
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
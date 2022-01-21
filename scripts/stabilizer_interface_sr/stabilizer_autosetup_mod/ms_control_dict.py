
BASIC_SETTINGS = {    
    0: ['sampling_freq', 1/(10e-9*208)],
    1: ['stream_port', 9933],
    2: ['frame_size', 1032],
    3: ['frame_payload', 1024],
    4: ['batch_size', 8],
    5: ['committing', True],
    6: ['stream_requesting', True],
    7: ['stream_format', 0],
    8: ['stream_channels', 4],
    9: ['msg_port', 9934],
    10: ['msg_size', 521],
}

# dictionary of settings and their types for save/load functions
SAVE = {
    'dflt':[['application','str'],
    ['dicts','str'],
    ['mac','str'],
    ['app_mode','str'],
    ['sampling_freq','float'],
    ['batch_size','int'],
    ['gain_afe0','str'],
    ['gain_afe1','str'],
    ['stream_batch_request','int'], 
    ['received_batch_count','int'], 
    ['received_frame_count','int'], 
    ['lut_config0_amplitude','float'],
    ['lut_config0_phase_offset_deg','float'],
    ['lut_config1_amplitude','float'],
    ['lut_config1_phase_offset_deg','float'],
    ['ctrl_offset','float'],
    ['sig_ctrl_signal','str'],
    ['sig_ctrl_frequency','float'],
    ['sig_ctrl_amplitude','float'],
    ['sig_ctrl_offset','float'],
    ['sig_ctrl_stream_trigger','str'],
    ['iir_ctrl','cmd','s.add_iir("iir_ctrl", s.sampling_freq/s.batch_size)'],
    ['iir_ctrl.Kp','float'],
    ['iir_ctrl.Ki','float'],
    ['iir_ctrl.Kd','float'],
    ['iir_ctrl.y_offset','float'],
    ['iir_ctrl.y_min','float'],
    ['iir_ctrl.y_max','float'],
    ['stream_decimation','int'],
    ['streams','list_str_4'],
    ['data','np_int_4','streams']]
}

# dictionary of settings, their type and miniconf path 
CONF = {
#regular entries [stabilizer attribute name, type, miniconf path]
# double entries for stabilizer attribute names are forbidden
# type: (setting types)_(data type)
# setting types:
#    cfg -> save/update in run_conf
#    auto -> execute automated update on remote device
#    obj -> needs to be set if attribute is an object. 
#           The attributes of the object can't have auto tag
#    sha -> "shadow" attributes, that are changed in run_conf but not
#           mentioned directly
# data types:
#   num: number
#   str: string (or enum on stabilizer)
#   tar: streaming target
#   iir: iir filter of iirClass
#   cmd: command to be executed in run_conf, needs to be paired with
#        setting type cfg. E.g. used for init objects in run_conf           
#   cmt: comment printed in run_conf
#   list: python list of basic types. If a list is chosen with auto type,
#         the miniconf path is a list of the struct containing the list
#         and the name of the field with contains the list entries.
#         Eg. in ms_control onboard settings struct streams with field
#         stream_set requires ['streams=','stream_set']
#   lim: plot limits (string or list)
    'dflt':[['s-init','cfg_cmd', 's=stabilizerClass("ms_control")\n'],
    ['stream_kill','cfg_num',''],
    ['rewrite_conf','cfg_num',''],
    ['app_mode', 'auto-cfg_str', 'app_mode='],
    ['telemetry_period', 'cfg-auto_num','telemetry_period='],
    ['gain_afe0','cfg-auto_str','afe/0='],
    ['gain_afe1','cfg-auto_str','afe/1='],
    ['lut_config0_amplitude','cfg-auto_num','lut_config/0/amplitude='],
    ['lut_config0_phase_offset_deg','cfg-auto_num',\
                             'lut_config/0/phase_offset_deg='],
    ['lut_config1_amplitude','cfg-auto_num','lut_config/1/amplitude='],
    ['lut_config1_phase_offset_deg','cfg-auto_num',\
                             'lut_config/1/phase_offset_deg='],
    ['ctrl_offset','auto-cfg_num','ctrl_offset='],
    ['sig_ctrl_signal','cfg-auto_str','signal_generator_ctrl/signal='],
    ['sig_ctrl_frequency','cfg-auto_num','signal_generator_ctrl/frequency='],
    ['sig_ctrl_amplitude','cfg-auto_num','signal_generator_ctrl/amplitude='],
    ['sig_ctrl_offset','cfg-auto_num','signal_generator_ctrl/offset='],
    ['sig_ctrl_stream_trigger','cfg-auto_str','stream_trigger='],
    ['stream_port','auto_tar','stream_target='],
    ['msg_port','auto_tar','msg_target='],
    ['stream_mode','auto_str','stream_mode='],
    ['stream_request_length','cfg_num',''],
    ['stream_request_unit','cfg_str',''],
    ['stream_request','cfg_cmd','s.set_stream_length(s.stream_request_length, s.stream_request_unit)\n'],
    ['stream_batch_request','sha-auto_num','stream_batch_request='],
    ['streams', 'auto-cfg_list',['streams=', 'stream_set']],
    # init new iirClass object in run_conf:
    ['iir_ctrl-init','cfg_cmd', 'iir_ctrl=s.add_iir("iir_ctrl", s.sampling_freq/s.batch_size)\n'],
    ['iir_ctrl.Kp','cfg-chld_num',''],
    ['iir_ctrl.Ki','cfg-chld_num',''],
    ['iir_ctrl.Kd','cfg-chld_num',''],
    ['iir_ctrl.ba','cfg-chld_cmd','iir_ctrl.ba=s.iir_ctrl.compute_coeff()\n'],
    ['iir_ctrl.ba-cmt','cfg_cmt','#s.iir_ctrl.ba=[0,0,0,0,0]\n'],
    ['iir_ctrl.y_offset','cfg-chld_num',''],#auto included in iir_ctrl struct
    ['iir_ctrl.y_min','cfg-chld_num',''],#auto tag included in iir_ctrl struct
    ['iir_ctrl.y_max','cfg-chld_num',''],#auto tag included in iir_ctrl struct
    # struct needed for autosetup, needs to be last entry for iir_ctrl
    # attributes
    ['iir_ctrl','auto-cfg-obj_iir', 'iir_ctrl='],
    ['lines_config_threshold','cfg-auto_num','lines_config/threshold='],
    ['lines_config_offset','cfg-auto_num','lines_config/offset='],
    ['lines_config_hysteresis','cfg-auto_num','lines_config/hysteresis='],
    ['plot-init','cfg_cmd', 'plot=s.add_plot()\n'],
    ['stream_decimation','cfg_num',''],
    ['plot.plots','cfg_list',''],
    ['plot.xlim','cfg_lim',''],
    ['plot.ylim','cfg_cmt','#s.plot.xlim=(-1,1)\n'],
    ['plot.ylim','cfg_lim',''],
    ['plot.ylim','cfg_cmt','#s.plot.ylim=[(-1,1),(-1,1),(-1,1)]\n'],
    ['plot.xtype','cfg_str',''],
    ['plot.tolerance','cfg_num',''],
    ['plot.refresh_ylim','cfg_num','']]
}
    
DATA_INFO = { 
    # refer to https://numpy.org/doc/stable/user/basics.types.html for
    # data types
    'Mod': ['DAC', 'np.ushort','Modulation signal', 'Voltage [V]'],
    'Demod': ['DAC', 'np.ushort', 'Demodulation signal', 'Voltage [V]'],            
    'ErrMod': ['VOLT', 'np.short', 'Error signal modulated (ADC input)', 'Voltage [V]'],
    'ErrDemod': ['VOLT', 'np.short', 'Error signal demodulated & filtered', 'Voltage [V]'],
    'CtrlDac': ['DAC', 'np.ushort', 'VCO control signal on DAC', 'Voltage [V]'],
    'CtrlSig': ['DAC', 'np.ushort','VCO control signal from Signal Generator', 'Voltage [V]'],
    'Lines': ['NUM', 'np.short','Spectral feature detection', 'Arbitrary Unit'],
}

PARAM = {
    'param name':     ['number', 'data type', 'length in bytes'],
    'commit':         [1, 'bool', 1],
    'stream_request': [2, 'bool', 1],
    'msg_request':    [3, 'bool', 1],
        
}
import traceback
from importlib import reload
import socket
import select
import struct
import subprocess
from datetime import datetime
import sys
import os
path = os.path.dirname(os.path.realpath(__file__))
import numpy as np
import csv
import matplotlib.pyplot as plt
plt.rcParams.update({'font.family':'sans-serif'})
import logging
logger = logging.getLogger()
logger.setLevel(logging.WARNING)
ch = logging.StreamHandler()
ch.setLevel(logging.WARNING)
formatter = logging.Formatter('%(levelname)s: %(message)s')
ch.setFormatter(formatter)
if logger.hasHandlers() == False:
    logger.addHandler(ch)
from iir_coefficients import pid_coefficients
import functools
from copy import deepcopy
from scipy import signal
import shlex
import platform
import warnings



class stabilizerClass:
    """ Provides access to Stabilizer's livestreamed data and settings """

    # The magic header half-word at the start of each packet.
    MAGIC_WORD = 0x057B
    MSG_MAGIC_WORD = 0x057C


    # The struct format of the header.
    HEADER_FORMAT = '<HBBL'
    
    # The struct format of the header.
    #https://docs.python.org/3/library/array.html
    MSG_HEADER_FORMAT = '<HBHL'

    # All supported formats by this reception script. The items in this dict 
    # are functions that return the struct deserialization code to unpack 
    # a single batch and the number of different data fields in a batch.
    FORMAT = {
        # dummy entry, information is gathered from application settings
        0: lambda batch_size: \
            (f'<{batch_size}H{batch_size}H{batch_size}H{batch_size}H', 4), 
        # uint16; uint16; uint16; uint16 
        1: lambda batch_size: \
            (f'<{batch_size}H{batch_size}H{batch_size}H{batch_size}H', 4),
        # int16; int16; uint16; uint16   
        2: lambda batch_size: \
            (f'<{batch_size}h{batch_size}h{batch_size}H{batch_size}H', 4),
        
    }
    
    # dictionary of supported gains of Stabilizer's AFEs 
    GAINS = {
        'G1': 1,
        'G2': 2,
        'G5': 5,
        'G10': 10
    }
    
    BASIC_SETTINGS = {
        'ms_control': {0: ['sampling_freq', 1/(10e-9*208)],
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
    }
    
    # dictionary of settings and their types for save/load functions
    SAVE = {
    'ms_control': 
        [['application','str'],
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
    #   list: python list of basic types
    #   lim: plot limits (string or list)
    'ms_control':
      
        [['s-init','cfg_cmd', 's=stabilizerClass("ms_control")\n'],
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
        ['streams', 'auto-cfg_list','streams="{"""stream_set""":'],
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
        ['plot.refresh_ylim','cfg_num','']
        ]
    }
        
    DATA_INFO = { 
        # refer to https://numpy.org/doc/stable/user/basics.types.html for
        # data types
        'ms_control': {
            'Mod': ['DAC', 'np.ushort','Modulation signal', 'Voltage [V]'],
            'Demod': ['DAC', 'np.ushort', 'Demodulation signal', 'Voltage [V]'],            
            'ErrMod': ['VOLT', 'np.short', 'Error signal modulated (ADC input)', 'Voltage [V]'],
            'ErrDemod': ['VOLT', 'np.short', 'Error signal demodulated & filtered', 'Voltage [V]'],
            'CtrlDac': ['DAC', 'np.ushort', 'VCO control signal on DAC', 'Voltage [V]'],
            'CtrlSig': ['DAC', 'np.ushort','VCO control signal from Signal Generator', 'Voltage [V]'],
            'Lines': ['NUM', 'np.short','Spectral feature detection', 'Arbitrary Unit'],
        }
    }
    
    
    def __init__(self, application):
        self.execute_update = 0
        self.commit_opened = 0
        self.committing = False
        self.stream_requesting = False
        self.telemetry_period = 10
        self.broker = '192.168.137.1'
        self.mac = '04-91-62-d9-83-19'
        self.application = application
        self.gain_afe0 = 'G1'
        self.gain_afe0 = 'G1'
        self.frames = []
        self.data = None
        self.received_batch_count = 0
        self.received_frame_count = 0
        self.request_counter = 0
        self.stream_kill = 0
        (filename,line_number,function_name,text)=traceback.extract_stack()[-2]
        def_name = text[:text.find('=')].strip()
        self.name = def_name
        self.run_conf_mod = 0
        self.stream_decimation = 1
        self.stream_format = 0
        self.print_cmd = False
        self.rewrite_conf = 0
        self.stream_request_unit = 'batches'
        self.stream_request_length = 0
        self.dropped_frames = 0
        self.list_dropped_frames = 0
        
        basic_settings = self.BASIC_SETTINGS[application]
        for i in range(0,len(basic_settings)):
            setattr(self, basic_settings[i][0], basic_settings[i][1])
        self.prefix = 'python3 -m miniconf --broker '+self.broker+' dt/sinara/'\
                                    +self.application+'/'+self.mac+' '

        if platform.system() == 'Windows':
            self.islnx = False
        elif platform.system() == 'Linux':
            self.islnx = True
        else:
            warnings.warn("OS cound not be determined, defaulting to Windows. \
                                    This could lead to errors with system commands.")
            self.islnx = False     

    
    def run(self):
        # settings update and data streaming
        if self.stream == 1:
            self.stream_connect()
            self.msg_connect()
            autosetup(self) 
            
            if self.stream_mode == 'Cont':
                conf_init = 0
                while( (self.stream_kill==0)): 
                    # create streaming config file
                    if conf_init == 0:
                        self.write_conf()
                        self.set_run_conf_mod()
                        conf_init = 1
                    
                    # reload config, request and receive data    
                    self.reload_conf()
                    if self.rewrite_conf == 1:
                        self.write_conf()
                    
                    if self.stream_kill == 0:
                        self.stream_clear_buf()
                        self.set_stream_request()
                        self.stream_read()
                        # parse data and convert to respective units
                        self.stream_parse()
                        self.decimate()
                        self.data_from_int()   
                        if self.plot_data == 1:
                            self.plot.update()
                        self.reset_stream_request()
                        self.arbitrary_task()
        
            if self.stream_mode == 'Shot':
                self.stream_clear_buf()
                self.set_stream_request()
                self.stream_read()
                self.reset_stream_request()
                self.stream_parse()
                self.decimate()
                self.data_from_int()
                self.plot.update()
                self.arbitrary_task()
                    
            self.stream_close()
            self.msg_close()
        
        elif self.execute_update == 1:
            self.stream_batch_request = 0
            autosetup(self)
    
        # Saving data as csv
        if (self.save_data == 1):
            self.save(self.save_tag)
    
    def set_stream_length(self, length, unit):
        bpf = self.batches_per_frame()
        if unit == 'samples':
            batches = length/self.batch_size
            self.stream_batch_request = int(int(batches/bpf)*bpf)
        elif unit == 'batches':
            self.stream_batch_request = int(int(length/bpf)*bpf)
        elif unit == 'frames':
            self.stream_batch_request = int(length*bpf)
        elif unit == 's':
            samples = int(length*self.sampling_freq)
            batches = int(samples/self.batch_size)
            self.stream_batch_request = int(int(batches/bpf)*bpf)
        elif unit == 'ms':
            samples = int(length*self.sampling_freq/1e3)
            batches = int(samples/self.batch_size)
            self.stream_batch_request = int(int(batches/bpf)*bpf)
        elif unit == 'us':
            samples = int(length*self.sampling_freq/1e6)
            batches = int(samples/self.batch_size)
            self.stream_batch_request = int(int(batches/bpf)*bpf)
        self.stream_request_unit = unit
        self.stream_request_length = length
    
    def get_stream_length(self, batches):
        bpf = self.batches_per_frame()
        if self.stream_request_unit == 'samples':
            return int(batches*self.batch_size)
        elif self.stream_request_unit == 'batches':
            return int(batches)
        elif self.stream_request_unit == 'frames':
            return int(batches/bpf)
        elif self.stream_request_unit == 's':
            samples = int(batches*self.batch_size)
            return samples/self.sampling_freq
        elif self.stream_request_unit == 'ms':
            samples = int(batches*self.batch_size)
            return samples/self.sampling_freq*1e3
        elif self.stream_request_unit == 'us':
            samples = int(batches*self.batch_size)
            return samples/self.sampling_freq*1e6
            
    def batches_per_frame(self):
        return self.frame_payload/self.batch_size/self.stream_channels/2
    
    def rx_batches_to_frames(self):
        return int(self.received_batch_count/self.batches_per_frame())

    def rx_frames_to_batches(self):
        return int(self.received_frame_count*self.batches_per_frame())
    
    def req_batches_to_frames(self):
        bpf = self.batches_per_frame()
        self.stream_batch_request = int(int(self.stream_batch_request/bpf)*bpf)
        return int(self.stream_batch_request/bpf)    
    
    def get_gain(self, afe):
        if afe == 0:
            return self.GAINS[self.gain_afe0]
        elif afe == 1: 
            return self.GAINS[self.gain_afe1]
    
    
    def commit_open(self):
        if self.committing & (self.commit_opened == 0):       
            cmd = self.prefix+'commit='+'false'
            res = shell_cmd(cmd, self.timeout, self.print_cmd,False, self.islnx)
            if ' OK' in res:
                print('commit=false: OK')
            self.commit_opened = 1

    def commit_close(self):
        if self.committing & (self.commit_opened == 1):       
            cmd = self.prefix+'commit='+'true'
            res = shell_cmd(cmd, self.timeout, self.print_cmd,False, self.islnx)
            if ' OK' in res:
                print('commit=true: OK')
            self.commit_opened = 0
 
    def stream_connect(self):
        if self.stream_port != 0:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET,\
                                                   socket.SO_RCVBUF,0x7FFFFFFF)
            if self.timeout is not None:
                self.socket.settimeout(self.timeout)
            
            try:
                self.socket.bind(("", self.stream_port))
                self.stream_socket_opened = 1
            except:
                logger.error('Cannot bind UDP socket for streaming,'\
                    +' possibibly due to already existing connection.\n'\
                    +'Terminated process. '\
                    +'Try again or restart console if the error persists')
                sys.exit()   
    
    def msg_connect(self):
        if self.msg_port != 0:
            self.msg_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.msg_socket.setsockopt(socket.SOL_SOCKET,\
                                                   socket.SO_RCVBUF,0x7FFFFFFF)
            if self.timeout is not None:
                self.msg_socket.settimeout(self.timeout)
            
            try:
                self.msg_socket.bind(("", self.msg_port))
                self.msg_socket_opened = 1
            except:
                logger.error('Cannot bind UDP socket for messages,'\
                    +' possibibly due to already existing connection.\n'\
                    +'Terminated process. '\
                    +'Try again or restart console if the error persists')
                sys.exit()   
    
    def stream_close(self):
        try: 
            if (self.stream_socket_opened == 1):
                self.socket.close()
                self.stream_socket_opened = 0
        except:
            pass
    
    def msg_close(self):
        try: 
            if (self.msg_socket_opened == 1):
                self.msg_socket.close()
                self.msg_socket_opened = 0
        except:
            pass

    def stream_read(self):
        buf = []
        ready = select.select([self.socket], [], [], self.timeout)
        i = 0
        req = self.req_batches_to_frames()
        if len(ready[0]) > 0:
            while( (len(ready[0]) > 0) & (i<req) ):
                buf.append(self.socket.recv(self.frame_size))
                i = i+1
                if i < req:
                    ready = select.select([self.socket], [], [], self.timeout)
        else:
            self.stream_kill = 1
            print('Stream killed')
        
        self.received_frame_count = i
        if i < req:
            print('No frames received for '+str(self.timeout)+'s')
            self.dropped_frames += req-self.received_frame_count
        self.received_batch_count = self.rx_frames_to_batches() 
        rx_length = self.get_stream_length(self.received_batch_count)
        rx_length = str(round(rx_length,5))
        req_length = self.get_stream_length(self.stream_batch_request) 
        req_length = str(round(req_length,5))
        print('Received '+rx_length+' / '+req_length +' '\
                                      + self.stream_request_unit)  
        if self.list_dropped_frames == 1:
            print('Dropped frames: '+str(self.dropped_frames))       
        
        self.frames = buf    
    
    def stream_clear_buf(self):
        ready = select.select([self.socket], [], [], 0)
        while( len(ready[0]) > 0 ):
            self.socket.recv(self.frame_size)
            ready = select.select([self.socket], [], [], 0)    

    def stream_parse(self):   
        data = None
        
        for k in range(0, len(self.frames)):
            # Check if received packets contain at least header
            if len(self.frames[k]) < struct.calcsize(self.HEADER_FORMAT):
                return
        
            # Parse out the packet header
            magic, format_id, batch_size, sequence_number =\
                        struct.unpack_from(self.HEADER_FORMAT, self.frames[k])
            parse_buf = self.frames[k][struct.calcsize(self.HEADER_FORMAT):]


            if self.stream_format > 0:           
                frame_format = self.FORMAT[format_id](batch_size)[0]
                channels = self.FORMAT[format_id](batch_size)[1]
                bytes_per_batch = struct.calcsize(frame_format)
            else:
                frame_format = '<'+str(batch_size*self.stream_channels)+'H'
                channels = self.stream_channels
                # assuming half words here!
                bytes_per_batch = self.stream_channels*batch_size*2
        
            if magic != self.MAGIC_WORD:
                logging.warning('Encountered bad magic header: %s', hex(magic))
                continue
            
            batch_count = int(len(parse_buf) / bytes_per_batch)
            
            if k==0:
                data_points = int(batch_size*batch_count*len(self.frames))
                data = []
                for i in range(0,len(self.streams)):
                    data.append(np.zeros(data_points, np.uint))
            
            for j in range(batch_count):
                channels_parsed = struct.unpack_from(frame_format, parse_buf)
                parse_buf = parse_buf[struct.calcsize(frame_format):]
                idx0 = j*batch_size+k*batch_count*batch_size
                idx1 = (j+1)*batch_size+k*batch_count*batch_size
                for i in range(0,channels):
                    data[i][idx0:idx1] = channels_parsed[i*batch_size:(i+1)*batch_size]
                
        self.data = data
    
    def decimate(self):     
        if self.decimate == 1:
            return
        if self.data is not None:
            xdim = int(len(self.data[0])/self.stream_decimation)
            
            data_dec = []
            for i in range(0,len(self.streams)):
                data_dec.append(np.zeros(xdim, np.uint))
            
            for j in range(0,xdim):
                for i in range(0,len(self.streams)):
                    data_dec[i][j] = self.data[i][self.stream_decimation*j]
                
            self.data = data_dec
            
    def data_from_int(self):
        if self.data is None:
            logger.error('There is no data to process')
            return None
        
        data_info = self.DATA_INFO[self.application]
                        
        for i in range (0, len(self.streams)):
            # convert data types according to DATA_INFO and not FORMAT
            if self.stream_format == 0:
                self.data[i] = self.data[i].astype(eval(data_info[self.streams[i]][1]))
            
            # convert data according to their unit
            if data_info[self.streams[i]][0] == 'ADC0':
                data_i = volt_from_adc(self.data[i])/self.get_gain(0)
            elif data_info[self.streams[i]][0] == 'ADC1':
                data_i = volt_from_adc(self.data[i])/self.get_gain(1)
            elif data_info[self.streams[i]][0] == 'VOLT':                   
                data_i = volt_from_adc(self.data[i])
            elif data_info[self.streams[i]][0] == 'DAC':
                data_i = volt_from_dac(self.data[i])
            elif data_info[self.streams[i]][0] == 'NUM':
                data_i = self.data[i]
            # set converted data with it's defined name as stabilizer attribute
            setattr(self, self.streams[i], data_i)
            
    def write_conf(self):
        cr = "\n"
        conf_list = self.CONF[self.application]
        conf = open(path+'/run_conf.py','w+')
        conf.write('from stabilizer_if import *\n\n')
        for i in range(0,len(conf_list)):
            conf_type = conf_list[i][1].partition('_')[0]
            if 'cfg' in conf_type:
                conf_type = conf_list[i][1].partition('_')[2]
                if conf_type == 'num': 
                    conf.write('s.'+conf_list[i][0]+'='+str(rgetattr(self,conf_list[i][0]))+cr)    
                elif conf_type == 'str':
                    conf.write('s.'+conf_list[i][0]+"='"+rgetattr(self,conf_list[i][0])+"'"+cr)
                elif 'cmd' in conf_type:
                    conf.write(conf_list[i][2])
                elif conf_type == 'cmt':
                    conf.write(conf_list[i][2])
                elif conf_type == 'list':
                    conf.write('s.'+conf_list[i][0]+"="+str(rgetattr(self,conf_list[i][0]))+cr)
                elif conf_type == 'lim':
                    if isinstance(rgetattr(self,conf_list[i][0]),str):
                        conf.write('s.'+conf_list[i][0]+"='"+rgetattr(self,conf_list[i][0])+"'"+cr)
                    else:
                        conf.write('s.'+conf_list[i][0]+"="+str(rgetattr(self,conf_list[i][0]))+cr)
                    
        
    def set_run_conf_mod(self):
        # single set time function because calling it within
        # write_conf seems return the wrong value
        self.run_conf_mod = os.path.getmtime('run_conf.py')

    def reload_conf(self):
        if self.run_conf_mod != os.path.getmtime('run_conf.py'):
            self.set_run_conf_mod()
            import run_conf
            reload(run_conf)
            conf_list = self.CONF[self.application]
            
            paused_stream = 0 
            for i in range(0, len(conf_list)):
                conf_type = conf_list[i][1].partition('_')[0]
                # check if conf_list entry is selected for updates
                type_check = ('cfg' in conf_type) | ('sha' in conf_type)
                type_check = type_check & (not ('cmd' in conf_list[i][1]))
                type_check = type_check & (not ('cmt' in conf_list[i][1]))
                type_check = type_check & (not ('chld' in conf_list[i][1]))
                if type_check:
                    var = conf_list[i][0]
                    update = False
                    # check if one attribute in conf_list 
                    # included in an object has changed
                    if 'obj' in conf_type:            
                        obj = rgetattr(run_conf.s,var)
                        attr_list = list(vars(obj))
                        for j in range(0, len(attr_list)):
                            for k in range(0, len(conf_list)):
                                if conf_list[i][0]+'.'+attr_list[j] == conf_list[k][0]:
                                    if rgetattr(obj,attr_list[j]) != rgetattr(self,conf_list[k][0]):
                                        # set changed attribute to stabilizer object in interface
                                        rsetattr(self,conf_list[k][0], rgetattr(obj,attr_list[j]))
                                        update = True
                    # check if regular attribute in conf_list has changed
                    else:
                        update = rgetattr(run_conf.s,var) != rgetattr(self,var)
                    
                    # update stabilizer setting in interface and onboard
                    if update:
                        if paused_stream == 0:
                            self.reset_stream_request()    
                            paused_stream = 1
                        rsetattr(self,var, rgetattr(run_conf.s,var))
                        if 'auto' in conf_type:
                            self.commit_open()
                            self.update_setting(conf_list[i])
                    
            if self.stream_kill == 1:
                print('Terminating stream')
                self.commit_open() 
                cmd = self.prefix+'stream_mode="""Stop"""'
                shell_cmd(cmd, self.timeout, self.print_cmd, True, self.islnx)
                self.reset_stream_request()
            self.commit_close() 
        
    def update_setting(self, conf_list_i):
        if self.islnx:
            ap = '\'"'
            ep = '"\''
            sp = '\''
            dp = '"'
        else:
            ap = '"""'
            ep = '"""'
            sp = '"'
            dp = '"""'
        conf_type = conf_list_i[1].partition('_')[2]
        if conf_type == 'num':
            cmd = self.prefix+conf_list_i[2]+str(rgetattr(self,conf_list_i[0]))
        elif conf_type == 'str':
            cmd = self.prefix+conf_list_i[2]+ap+rgetattr(self,conf_list_i[0])+ep
        elif conf_type == 'tar':
            broker = self.broker.replace(".",",")
            cmd = self.prefix+conf_list_i[2]+sp+'{'+dp+'ip'+dp+':['+broker\
                                        +'], '+dp+'port'+dp+':'+\
                                        str(rgetattr(self,conf_list_i[0]))+'}'+sp
        elif conf_type == 'iir':
            iir = rgetattr(self,conf_list_i[0])          
            cmd = self.prefix+conf_list_i[2]+sp+'{'+dp+'ba'+dp+':'+ str(rgetattr(iir,'ba'))\
                    +','+dp+'y_min'+dp+':'+str(iir.y_min)+', '+dp+'y_max'+dp+':'\
                    +str(iir.y_max)+', '+dp+'y_offset'+dp+':'+str(iir.y_offset)+'}'+sp 
        elif conf_type == 'list':
            lst = rgetattr(self, conf_list_i[0])
            cmd = self.prefix+conf_list_i[2]+"["
            for i in range(0,len(lst)):
                if isinstance(lst[i], str):
                    cmd = cmd+ap+lst[i]+ep+','
                else: #assume that it is a number that can be converted to str
                    cmd = cmd+str(lst[i]+',')
            # remove last comma and add closing characters
            cmd = cmd[0:-1]+']}"'
        else:
            return
        
        shell_cmd(cmd, self.timeout, self.print_cmd, True, self.islnx)
        
     
    def set_stream_request(self):
        if self.stream_requesting:
            cmd = self.prefix+'stream_request='+'true'
            if self.stream_mode == 'Cont':
                res = shell_cmd(cmd, self.timeout, self.print_cmd, False, self.islnx)
                print('('+str(self.request_counter)+') '+res[0:-1])
                self.request_counter += 1 
            else:
                shell_cmd(cmd, self.timeout, self.print_cmd, True, self.islnx)
                
    def reset_stream_request(self):
        asd = 1
        if self.stream_requesting:
            cmd = self.prefix+'stream_request='+'false'
            shell_cmd(cmd, self.timeout, 0, False, self.islnx)
            
    def add_plot(self):
        self.plot = plotClass(self)
        return self.plot
        
    def add_iir(self, name=None, sampling_freq=None):
        if sampling_freq is None:
            sample_period = 1/self.sampling_freq
        else:
            sample_period = 1/sampling_freq
            
        iir = iirClass(sample_period, self)
        if name is None:
            name = 'iir'
        setattr(self, name, iir)
        return iir
    
    def save(self, tag, print_date = False):
        application = ''
        try:
            application = self.application
        except:
            logger.error('Stabilizer application is not specified')
            return
        
        if os.path.exists('savedData')==False:
            os.mkdir('savedData')
            sys.path.insert(0, path+'/savedData')
            
        # create timestamp
        stamp = ''
        u = '_'
        if print_date:
            time = datetime.now()
            # year-month-day_hour-min_index
            stamp = '{:%Y-%m-%d_%H%M}'.format(time)
            stamp = stamp + u
        
        filename = 'savedData/'+application+u+tag+u+stamp#+idx later
        
        # check if timestamp aleady exist and increase index if so
        idx=1
        while os.path.exists(filename+str(idx)+'.csv'):
            idx = idx+1
        filename = filename+str(idx)+'.csv'
        
        s = self
        with open(filename, mode='w', newline='') as csvfile:
            dataWriter = csv.writer(csvfile, delimiter=';', quotechar='',\
                                            quoting=csv.QUOTE_NONE)
            try:
                attrs = self.SAVE[s.application]
            except:
                logger.error('Unknown application "'+s.application\
                          + '". Cannot save data.')
                return

            for i in range(0,len(attrs)):
                row = ['']
                if attrs[i][1] == 'cmd':
                    row =  [attrs[i][0], attrs[i][1], attrs[i][2]]
                    dataWriter.writerow(row)
                elif attrs[i][1][0:2] == 'np':
                    # process numpy array
                    if not (rgetattr(self,attrs[i][0]) is None):
                        rows = int(attrs[i][1][7:])
                        try:
                            # try to get a list containing stream names
                            stream_list = rgetattr(self, attrs[i][2])
                        except:
                            # setup a numbered stream list
                            stream_list = list(map(str, range(rows)))
                        for j in range(0,rows):
                            row = ['','',0]
                            row[0] = attrs[i][0]+'.'+stream_list[j]
                            row[1] = attrs[i][1]
                            row[2:] = np.int_(rgetattr(self,attrs[i][0])[j])
                            dataWriter.writerow(row)
                elif attrs[i][1][0:4] == 'list':
                    lst = rgetattr(self, attrs[i][0])
                    row =  [attrs[i][0], attrs[i][1]]
                    for j in range(0,len(lst)):
                        row.append(str(lst[j]))
                    dataWriter.writerow(row)
                else:
                    # process regular primitive attributes
                    row =  [attrs[i][0], attrs[i][1], rgetattr(self,attrs[i][0])]
                    dataWriter.writerow(row)
                
        print('Saved data as '+filename)
        
    def arbitrary_task(self):
        task = 'arb_'+self.application+'(self)' 
        try:
            exec(task)        
        except:
            logger.error('Arbitrary task '+'arb_'+self.application+' failed.')


        
        
def shell_cmd(cmd, timeout, print_cmd, print_return, islnx):
    if print_cmd:
        print(cmd)
    if islnx:
        cmd = shlex.split(cmd)
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    try:
        p.wait(timeout)
        stdout, stderr = p.communicate()
        res = stdout.decode("utf-8")
        if print_return:
            print(res[0:-1])
        return res
    except subprocess.TimeoutExpired:
        logger.error('Shell command timeout trying:\n'+cmd)
        return '[ERROR]: shell command'
        p.kill()
    


class iirClass:
    def __init__(self, sample_period, _parent=None):
        self.parent = _parent
        self.sample_period = sample_period
        self.Kp = 0
        self.Ki = 0
        self.Kii = 0
        self.Kd = 0
        self.Kdd = 0
        self.y_min = 0
        self.y_max = 0
        self.y_offset = 0
        self.ba = [0, 0, 0, 0, 0]
        
    def compute_coeff(self):
        ba = pid_coefficients(self)
        self.ba = ba
        return ba
        
    def freqz(self, points, xlim=None):
        b = np.zeros(int((len(self.ba)+1)/2))
        b = self.ba[0:len(b)]
        # calculate_pid_coefficients defines negative a coefficients
        # for stabilizer onboard filter -> convert them back
        a = (-1)*np.ones(len(self.ba)-len(b)+1)
        a[1:] = self.ba[len(b):]
        a = a*(-1)

        freq = np.logspace(0,np.log10(self.parent.sampling_freq/2),points)
        f, h = signal.freqz(b, a, freq, fs=self.parent.sampling_freq)
    
        fig, ax = plt.subplots(2, figsize=(12,8),constrained_layout=True)
        ax[0].semilogx(f, 20 * np.log10(abs(h)))
        phi = np.unwrap( 2*np.angle(h, deg=False))/2
        ax[1].semilogx(f, 360/(2*np.pi)*phi)

        if xlim is not None:
            ax[0].set_xlim(xlim)

        if xlim is not None:
            ax[1].set_xlim(xlim)
        
        ax[0].set_title('IIR filter frequency response')
        ax[0].set_ylabel('Amplitude [dB]')
        ax[1].set_ylabel('Phase [deg]')
        ax[1].set_xlabel('Frequency [Hz]')
        ax[0].grid(which='both', axis='both')
        ax[1].grid(which='both', axis='both')
        plt.show()

class plotClass:
    def __init__(self, _parent=None):
        self.parent = _parent
        self.xtype = 'time_s'
        self.xlim = 'auto'
        self.xlabel = 'Samples'
        self.ylim = 'auto'
        self.init = 0    
        self.tolerance = 0.1
        self.refresh_ylim = 1
        self.persistent = False
        self.persistent_color = False
        self.plots = self.parent.streams

    def update(self):  
        if self.parent.data is None:
            return

        #self.parent.data_shape = self.parent.data.shape
        xlen = len(self.parent.data[0])
        
        data_info = self.parent.DATA_INFO[self.parent.application]
            
        # get xdata for plots and the xlabels
        if self.xtype == 'time_s':
            sampling_freq = self.parent.sampling_freq
            xdata = np.arange(0,xlen)/sampling_freq
            self.xlabel = 'Time [s]'
        elif self.xtype == 'time_ms':
            sampling_freq = self.parent.sampling_freq
            xdata = np.arange(0,xlen)/sampling_freq\
                                    *1e3*self.parent.stream_decimation
            self.xlabel = 'Time [ms]'
        elif self.xtype == 'time_us':
            sampling_freq = self.parent.sampling_freq
            xdata = np.arange(0,xlen)/sampling_freq\
                                    *1e6*self.parent.stream_decimation
            self.xlabel = 'Time ['+chr(956)+'s]'
        
        elif type(self.xtype) == str: 
            try:
                xdata = rgetattr(self.parent,self.xtype)
                for i in range(0,len(self.plots)):
                    if self.xtype == self.plots[i]:
                        self.xlabel = data_info[self.plots[i]][2]+\
                            ' as ' + data_info[self.plots[i]][3]
            except: 
                xdata = np.arange(0,xlen)
        else: 
            xdata = np.arange(0,xlen)
        
        # initialize figure
        if self.init == 0:
            self.fig, self.ax = plt.subplots(len(self.plots), figsize=(12,6), 
                                                     constrained_layout=False)

            left = 0.075  # the left side of the subplots of the figure
            right = 0.99   # the right side of the subplots of the figure
            bottom = 0.1  # the bottom of the subplots of the figure
            top = 0.95     # the top of the subplots of the figure
            wspace = 0.2  # the amount of width reserved for space between subplots,
                          # expressed as a fraction of the average axis width
            hspace = 0.6  # the amount of height reserved for space between subplots,
                          # expressed as a fraction of the average axis height
            plt.subplots_adjust(left, bottom, right, top, wspace, hspace)
            self.lines = []
            
            # create subplot lines
            data = np.zeros_like(xdata)
            for i in range(0,len(self.plots)):
                try:
                    data = rgetattr(self.parent, self.plots[i])
                except:
                    pass
                self.lines.append(self.ax[i].plot(xdata, data)[0])
                self.ax[i].grid()    
            
            self.init = 1
        
        # set the xlabel to the bottom subplot
        self.ax[len(self.plots)-1].set_xlabel(self.xlabel, fontsize=10)    

        # get x limits
        if isinstance(self.xlim, str):
            if self.xlim == 'auto':
                xlim = get_lim(xdata,0)
        else:
            xlim = self.xlim  

        # set y labels and titles
        for i in range(0,len(self.plots)):
            self.ax[i].set_title(data_info[self.plots[i]][2], fontsize=10)
            self.ax[i].set_ylabel(data_info[self.plots[i]][3], fontsize=10) 


        # set new x and y data to subplot lines
        for i in range(0, len(self.plots)):
            self.lines[i].set_xdata(xdata)
            self.ax[i].set_xlim(xlim)
  
            ydata = rgetattr(self.parent, self.plots[i])
           
            # get y limits
            if  isinstance(self.ylim, str):
                if (self.ylim == 'auto') & (self.refresh_ylim == 1):
                    self.ax[i].set_ylim(get_lim(ydata,self.tolerance))
            else:
                self.ax[i].set_ylim(self.ylim[i])  
            
             # uncomment following line for saving all lines in figure
            if self.persistent:
                if self.persistent_color:
                    clr = '#1f77b4'
                    self.lines.append(self.ax[i].plot(xdata, ydata, clr)[0]) 
                else:
                    self.lines.append(self.ax[i].plot(xdata, ydata)[0]) 
            else:
                self.lines[i].set_ydata(ydata)
            
        self.fig.canvas.draw()
        plt.pause(0.0001)
        self.fig.canvas.flush_events()
            
def load(filename):
    dir_path = os.path.dirname(os.path.realpath(__file__)) 
    sys.path.append(dir_path+'\\savedData')
    sys.path.append(dir_path)
    fail = 0
    path=filename    
    if os.path.exists(path)==False:
        fail = fail+1
        path='savedData\\'+filename 
    if os.path.exists(path)==False:
        fail = fail+1
    if fail == 2:
        logger.error('File for CSV read not found in directories\n'+dir_path\
                         +'\\'+filename+'\n'+dir_path+'\\savedData\\'+filename)
        return None    
    with open(path, mode='r', newline='') as csvfile:
        # Read all rows into a list
        rows = list(csv.reader(csvfile, delimiter=';'))      
        
        for i in range (0,len(rows)):
            if  rows[i][0] == 'application':
                app = rows[i][2]
                break
        
        s = stabilizerClass(app)
        i = 0
        while i < len(rows):
            if rows[i][1] == 'str':
                rsetattr(s, rows[i][0], rows[i][2])
            elif rows[i][1] == 'float':
                rsetattr(s, rows[i][0], float(rows[i][2]))
            elif rows[i][1] == 'int':
                rsetattr(s, rows[i][0], int(rows[i][2]))
            elif 'list' in rows[i][1]:
                lst = []
                lst_type = rows[i][1].split('_')
                data = rows[i][2].split(',')
                for j in range(2,2+int(lst_type[2])):
                    if lst_type[1] == 'str':
                        lst.append(rows[i][j])
                    elif lst_type[1] == 'float':
                        lst.append(float(rows[i][j]))
                    elif lst_type[1] == 'int':
                        lst.append(int(rows[i][j]))
                rsetattr(s, rows[i][0], lst)
                        
            elif rows[i][1][0:2] == 'np':
                if rows[i][1][3:6] == 'int':
                    data = []
                    channels = int(rows[i][1][7:])
                    for j in range(0, channels):
                        data.append(np.int_(rows[i+j][2:]))
                # skip the already processed data rows
                i = i + channels-1
                rsetattr(s, rows[i][0].partition('.')[0], data)
            elif rows[i][1] == 'cmd':
                try:
                    rsetattr(s, rows[i][0], eval(rows[i][2]))    
                except:
                    logger.error("Couldn't interpret command during device loading")
                    return
            i = i+1
                
        return s                

def autosetup(stabilizer):
    s = stabilizer
    
    if s.execute_update == 0:
        return
    
    print('Executing autosetup...')
    
    conf_list = s.CONF[s.application]
    if s.commit_opened == 0:
        s.commit_open()
    
    for i in range(1,len(conf_list)):
        conf_type = conf_list[i][1].partition('_')[0]
        if 'auto' in conf_type:     
            s.update_setting(conf_list[i])
            
    s.commit_close()
    print('Autosetup finished')   

def volt_from_dac(dac_code):
    """ Returns voltage from Stabilizer uint DAC code"""
    dac_range = 4.096 * 2.5
    dac_code = dac_code.astype(np.float64)
    return dac_range*(dac_code/0x8000-np.ones_like(dac_code))
    
def volt_from_adc(adc_code):
    """ Returns voltage from Stabilizer int ADC code"""
    """ Does not regard AFE gains"""
    adc_range = 4.096 * 2.5
    adc_code = adc_code.astype(np.float64)
    return adc_range*(adc_code/0x8000)

def get_lim(data, tolerance):
    upper = max(data)
    lower = min(data)
    diff = abs((upper-lower)/2)
    if diff == 0:
        diff = 1e-9
        lower = lower-diff
        upper = upper+diff
    if upper > 0:
        upper = upper+diff*tolerance
    else:
        upper = upper-diff*tolerance
        
    lower = lower-diff*tolerance
    
    return (lower,upper)

def save_plot(idx, filename):
    if plt.fignum_exists(idx):
        plt.figure(idx)
    else:
        print('[ERROR savePlotToPdf]: Figure '+str(idx)+' does not exist')
        return None
    try:
        plt.savefig(filename+'.pdf')
    except:
        print('[ERROR savePlotToPdf]: Figure '+str(idx)+' cannot be saved.'+\
              'Maybe a different program blocks the process')
        return None
       
# With kind regards to 
# https://stackoverflow.com/questions/31174295/...
# getattr-and-setattr-on-nested-subobjects-chained-properties
def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition('.')
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)

def rgetattr(obj, attr, *args):
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)
    return functools.reduce(_getattr, [obj] + attr.split('.'))

def arb_ms_control(s):
    
    if s.app_mode != 'SrchMan':
        return
    
    cmd = s.prefix+'msg_transmit='+'true'
    shell_cmd(cmd, s.timeout, s.print_cmd, True, platform.system=='Linux')
    
    buf = b'x00'
    ready = select.select([s.msg_socket], [], [], s.timeout)
    if len(ready[0]) > 0:
        buf = s.msg_socket.recv(s.msg_size)
    else:
        print('No line locations received for '+str(s.timeout)+'s')
    
    cmd = s.prefix+'msg_transmit='+'false'
    shell_cmd(cmd, s.timeout, s.print_cmd, True, platform.system=='Linux')
    
    try:
       # Parse out header data
        magic, format_id, msg_length, sequence_number =\
                        struct.unpack_from(s.MSG_HEADER_FORMAT, buf)
        parse_buf = buf[struct.calcsize(s.MSG_HEADER_FORMAT):]

        if magic != s.MSG_MAGIC_WORD:
            logging.warning('Encountered bad magic header: %s for'+\
                            ' line locations', hex(magic))
            return
        
        data = np.zeros(msg_length, np.uint)
        
        if msg_length == 0:
            print('Found no lines')
        else:
            for i in range(0, msg_length):
                data[i] = int(struct.unpack_from('<H', parse_buf)[0])
                parse_buf = parse_buf[struct.calcsize('<H'):]
        
            print('Found lines at CTRL DAC outputs [V]:')
            for i in range(0,msg_length):
                line = volt_from_dac(data[i])
                print('['+str(i)+']: '+str(line))
    except:
        logger.error('Failed at processing line locations')



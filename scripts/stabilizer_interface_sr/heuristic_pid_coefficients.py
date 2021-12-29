
Ku = 0.13
Tu = 301e-6

TUNING = {
# factors for [Kc, Ti, Tu] 
    'ZN': [0.6, 0.5, 0.125], # Ziegler-Nichols
    'ZN-SO': [0.33, 0.5, 0.33], # Ziegler-Nichols, some overshoot
    'ZN-NO': [0.2, 0.5, 0.33], # Ziegler-Nichols, no overshoot
    'PIAE':  [0.7, 0.4, 0.15], # Pessen Integral of Absolute Error
}

ttype = 'ZN-NO'
conv2par = 1

Kc, Ti, Td = TUNING[ttype]


Kp = Kc*Ku
Ki = 1/(Ti*Tu)
Kd = Td*Tu
if conv2par == 1:
    Ki = Ki*Kp
    Kd = Kd*Kp
    
if conv2par ==1:
    print('Tuning '+ttype+' converted to parallel controller')
else:
    print('Tuning '+ttype+' unconverted')

print('Kp = '+str(Kp))
print('Ki = '+str(Ki))
print('Kd = '+str(Kd))


# by hand (gains=1, vmod = 1.0):
#iir_ctrl.Kp=-0.220
#iir_ctrl.Ki=-10
#iir_ctrl.Kd=-5e-05

# by hand (gains=1, vmod = 2.0):
#iir_ctrl.Kp=-0.12
#iir_ctrl.Ki=-20
#iir_ctrl.Kd=-2e-05

# by hand (gains=2, vmod = 2.0):
#iir_ctrl.Kp=-0.05
#iir_ctrl.Ki=-20
#iir_ctrl.Kd=-1e-06

# by hand (gains=2, vmod = 0.5):
#iir_ctrl.Kp=-0.14
#iir_ctrl.Ki=-60
#iir_ctrl.Kd=-10e-06

# ZN-NO (gains=2, vmod = 0.5):
    
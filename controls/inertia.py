import numpy as np

# yawdata = {
#     'mass': 0.077,
#     'height': 25.5*0.0254, # rope hang length
#     'distance': 1.625*0.0254, # distance between rope anchors
#     'Tswing':1.6147, # (10 oscillations) period of swinging pendulum 
#     'Trot':2.3058, # (10 oscillations) period of swinging pendulum
#         }


# rolldata = {
#     'mass': 0.077,
#     'height': 25.5*0.0254, # rope hang length
#     'distance': 1.625*0.0254, # distance between rope anchors
#     'Tswing':1.6147, # period of swinging pendulum
#     'Trot':2.3058,
#         }
mass = 1.51
pitchdata = {
    'mass': mass,
    'height': 0.315, # rope hang length
    'distance': 0.067, # distance between rope anchors
    'Tswing':(11.26 + 11.14 + 11.18)/30, # period of swinging pendulum
    'Trot':(21.83+22.32+22.17)/30,
        }

yawdata = {
    'mass': mass,
    'height': 0.507, # rope hang length
    'distance': 0.152, # distance between rope anchors
    'Tswing':(14.17 + 15.20 + 14.73)/30, # period of swinging pendulum
    'Trot':(14.45 + 14.52 +14.65)/30,
        }

# try to use data to calc gravity to verify data quality
def gtest(data : dict):
    Tswing = data['Tswing']
    h = data['height']
    omega = 2*np.pi/Tswing
    gcalc = omega**2 * h
    greal = 9.81
    error_p = np.abs((greal - gcalc)/greal)*100
    print("\n[==== Testing for data integrity ====]")
    print(f"Gravity from data obtained is {gcalc:.5g} m/s^2. Error: {error_p:.5g} %\n")

def calcmoment(data : dict):
    g = 9.81
    m = data['mass']
    d = data['distance']
    h = data['height']
    Trot1 = data['Trot']/10.0
    inertia = m*g*(d**2) * (Trot1**2)/(16*(np.pi**2) * h)
    print("\n[==== Moment of Inertia ====]")
    print(f"I = {inertia:.5g}\n")
    
    


gtest(pitchdata)
calcmoment(pitchdata)

gtest(yawdata)
calcmoment(yawdata)
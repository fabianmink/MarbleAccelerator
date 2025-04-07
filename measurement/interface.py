import serial
import json
import numpy as np
#import time
from matplotlib import pyplot as plt 

ser = serial.Serial('COM9')  # open serial port
ser.baudrate = 256000
#ser.timeout = 0.1
print(ser.name)         # check which port was really used

ser.flush()

fig, (ax_i, ax_u) = plt.subplots(2, 1)

t = np.linspace(0.00, 800*62.5e-3, 800)


line_ia,  = ax_i.plot(t,t, 'r-', linewidth=0.5);
line_ib,  = ax_i.plot(t,t, 'g-', linewidth=0.5);
line_ic,  = ax_i.plot(t,t, 'b-', linewidth=0.5);

ax_i.set_ylim(-20, 20)

line_ua,  = ax_u.plot(t,t, 'r-', linewidth=0.5);
line_ub,  = ax_u.plot(t,t, 'g-', linewidth=0.5);
line_uc,  = ax_u.plot(t,t, 'b-', linewidth=0.5);

ax_u.set_ylim(-20, 20)

plt.show()

#while(True):
#    time.sleep(1)
line = ser.readline()

str = line.decode()
#if(str != ''):
x = json.loads(str)   

ia = np.array(x['ia'] )/256
ib = np.array(x['ib'] )/256
ic = np.array(x['ic'] )/256

ua = np.array(x['ua'] )/256
ub = np.array(x['ub'] )/256
uc = np.array(x['uc'] )/256

ucd = x['udc']

line_ia.set_ydata( ia );
line_ib.set_ydata( ib );
line_ic.set_ydata( ic );

line_ua.set_ydata( ua );
line_ub.set_ydata( ub );
line_uc.set_ydata( uc );

fig.canvas.draw_idle()

ser.close()             # close port
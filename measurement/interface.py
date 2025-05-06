import serial
import json
import numpy as np
import time
import threading
from matplotlib import pyplot as plt 

global do_exit;
global ser_open;
global ser;

data = [];
sema_newData = threading.Semaphore(0)  #0x (=not) available from startup
sema_dataRead = threading.Semaphore(1) #1x available from startup

do_exit = False;
ser_open = False;

def on_close(event):
    #print('Closed Figure!')
    #plt.close('all')
    global do_exit
    print("close")
    do_exit = True
    event.canvas.stop_event_loop()
    

def thread_function():
    global do_exit
    global data
    global sema_newData, sema_dataRead
    global ser_open;
    global ser;
    
    print("Starting")
    #time.sleep(2)

    ser_open = False
    try:
        ser = serial.Serial('COM9')  # open serial port
        #ser = serial.Serial('COM7')  # open serial port
        ser_open = True
        ser.baudrate = 256000
        #ser.timeout = 0.1
        print(ser.name)         # check which port was really used
        
        ser.flush()
        
    except serial.serialutil.SerialException:
        print("Serial Exeption")
        do_exit = True
        
    while(not do_exit):
        #print("Running")
        try:
            line = ser.readline()
            dataRead = sema_dataRead.acquire(False)
            if(dataRead):
                str = line.decode()
                data = json.loads(str)    
                
        except (json.decoder.JSONDecodeError, UnicodeDecodeError):
            print("Decode Exception")
            data = [];
            
        sema_newData.release()
            
            
     
fig, (ax_i, ax_u) = plt.subplots(2, 1)
fig.canvas.mpl_connect('close_event', on_close)

t = np.linspace(0.00, 800*62.5e-3, 800)


line_ia,  = ax_i.plot(t,0*t, 'r-', linewidth=1);
line_ib,  = ax_i.plot(t,0*t, 'g-', linewidth=1);
line_ic,  = ax_i.plot(t,0*t, 'b-', linewidth=1);

line_iaref,  = ax_i.plot(t,0*t, 'r:', linewidth=1);
line_ibref,  = ax_i.plot(t,0*t, 'g:', linewidth=1);
line_icref,  = ax_i.plot(t,0*t, 'b:', linewidth=1);

ax_i.set_xlim(0, 50)
ax_i.set_ylim(-20, 20)
ax_i.grid(1)
ax_i.set_xticks(np.arange(0, 50, step=2))
ax_i.set_xlabel("$t  /  \mathrm{ms}$")
ax_i.set_ylabel("$i  /  \mathrm{A}$")

line_ua,  = ax_u.plot(t,0*t, 'r-', linewidth=1);
line_ub,  = ax_u.plot(t,0*t, 'g-', linewidth=1);
line_uc,  = ax_u.plot(t,0*t, 'b-', linewidth=1);

line_udc,  = ax_u.plot(t,0*t, 'k-', linewidth=1);

ax_u.set_xlim(0, 50)
ax_u.set_ylim(-20, 20)
ax_u.grid(1)
ax_u.set_xticks(np.arange(0, 50, step=2))
ax_u.set_xlabel("$t  /  \mathrm{ms}$")
ax_u.set_ylabel("$u  /  \mathrm{V}$")


#x = threading.Thread(target=thread_function)
x = threading.Thread(target=thread_function, daemon=True)
x.start()

while (not do_exit):
    #check for new data
    newData = sema_newData.acquire(False)
    if(newData and (data != [])):
        ia = np.array(data['ia'] )/256
        ib = np.array(data['ib'] )/256
        ic = np.array(data['ic'] )/256
        
        iaref = np.array(data['iaref'] )/256
        ibref = np.array(data['ibref'] )/256
        icref = np.array(data['icref'] )/256

        ua = np.array(data['ua'] )/256
        ub = np.array(data['ub'] )/256
        uc = np.array(data['uc'] )/256

        udc = np.array(data['udc'] )/256
                
        line_ia.set_ydata( ia );
        line_ib.set_ydata( ib );
        line_ic.set_ydata( ic );
        
        line_iaref.set_ydata( iaref );
        line_ibref.set_ydata( ibref );
        line_icref.set_ydata( icref );

        line_ua.set_ydata( ua );
        line_ub.set_ydata( ub );
        line_uc.set_ydata( uc );

        line_udc.set_ydata( udc/2 );
        
        sema_dataRead.release()
        

    plt.pause(0.5)
    

plt.close('all')
if(ser_open):
    ser.close()

print("exit")


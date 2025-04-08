import serial
import json
import numpy as np
import time
import threading
from matplotlib import pyplot as plt 

data = [];
sema_newData = threading.Semaphore(0)  #0x (=not) available from startup
sema_dataRead = threading.Semaphore(1) #1x available from startup
do_exit = False;

def on_close(event):
    #print('Closed Figure!')
    #plt.close('all')
    global do_exit
    print("close")
    do_exit = True
    event.canvas.stop_event_loop()
    

def thread_function():
    global data
    global sema_newData, sema_dataRead
    
    print("Starting")
    time.sleep(2)

    ser = serial.Serial('COM9')  # open serial port
    ser.baudrate = 256000
    #ser.timeout = 0.1
    print(ser.name)         # check which port was really used
    
    ser.flush()
    
    while(not do_exit):
        #print("Running")
        line = ser.readline()
        dataRead = sema_dataRead.acquire(False)
        if(dataRead):
            str = line.decode()
            data = json.loads(str)    
            sema_newData.release()
            
    ser.close()
       
    
    
    
    

fig, (ax_i, ax_u) = plt.subplots(2, 1)
fig.canvas.mpl_connect('close_event', on_close)

t = np.linspace(0.00, 800*62.5e-3, 800)


line_ia,  = ax_i.plot(t,t, 'r-', linewidth=0.5);
line_ib,  = ax_i.plot(t,t, 'g-', linewidth=0.5);
line_ic,  = ax_i.plot(t,t, 'b-', linewidth=0.5);

ax_i.set_ylim(-20, 20)

line_ua,  = ax_u.plot(t,t, 'r-', linewidth=0.5);
line_ub,  = ax_u.plot(t,t, 'g-', linewidth=0.5);
line_uc,  = ax_u.plot(t,t, 'b-', linewidth=0.5);

line_udc,  = ax_u.plot(t,t, 'k-', linewidth=0.5);

ax_u.set_ylim(-20, 20)


x = threading.Thread(target=thread_function)
#x = threading.Thread(target=thread_function, daemon=True)
x.start()

while (not do_exit):
    #check for new data
    newData = sema_newData.acquire(False)
    if(newData):
        ia = np.array(data['ia'] )/256
        ib = np.array(data['ib'] )/256
        ic = np.array(data['ic'] )/256

        ua = np.array(data['ua'] )/256
        ub = np.array(data['ub'] )/256
        uc = np.array(data['uc'] )/256

        udc = np.array(data['udc'] )/256
                
        line_ia.set_ydata( ia );
        line_ib.set_ydata( ib );
        line_ic.set_ydata( ic );

        line_ua.set_ydata( ua );
        line_ub.set_ydata( ub );
        line_uc.set_ydata( uc );

        line_udc.set_ydata( udc/2 );
        
        sema_dataRead.release()
        

    plt.pause(0.5)
    

print("exit")


#              # close port
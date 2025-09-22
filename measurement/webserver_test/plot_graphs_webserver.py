import json
import serial
import time
#import math
import numpy as np
import asyncio
import threading


# ==== FastAPI imports ====
from fastapi import FastAPI, WebSocket
from fastapi.responses import HTMLResponse
import uvicorn


# =======================
# Serial config
SERIAL_PORT = 'COM3'   # Update port
BAUD_RATE = 115200
# =======================

app = FastAPI()
clients = set()

@app.get("/")
async def get():
    with open("html/plot_graphs.html", "r") as f:  # OPEN INDEX FILE HTML
        return HTMLResponse(f.read())

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    clients.add(websocket)
    print("added client: " + str(websocket.client.host) + ":" + str(websocket.client.port))
    try:
        while True:
            text = await websocket.receive_text()  # keep alive
            print("Text from " + str(websocket.client.host) + ":" + str(websocket.client.port) + " = " + text)
    except:
        pass
    finally:
        clients.remove(websocket)

async def broadcast(data: dict):
    """Send JSON data to all connected WebSocket clients"""
    living = set()
    #print("broadcast data to...")
    for ws in clients:
        print("trying to send data to client " + ws.client.host + ":" + str(ws.client.port))
        try:
            await ws.send_json(data)
            living.add(ws)
        except:
            pass
    clients.clear()
    clients.update(living)

def start_server():
    """Run FastAPI server in background thread"""
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")


# ---- Dummy Serial Reader, can be used without STM32 hardware ----
class DummyReader():
 
    def __init__(self):
        self.data_received = threading.Semaphore(0)
        self.data_consumed = threading.Semaphore(1)
        self.thread = threading.Thread(target = self.run)
        self.running = True
        self.data = None
        self.startvalue = 0
        self.upcounting = True
        super().__init__()

    def run(self):
        #print("in run")
        while self.running:
            time.sleep(1)
            #print("in loop")
            
            if(self.data_consumed.acquire(False)) :
                #print("data preparing")
                
                if(self.upcounting):
                    endval = self.startvalue+100
                    x = list(range(self.startvalue,endval))
                    self.startvalue = endval
                else:
                    endval = self.startvalue-100
                    x = list(range(self.startvalue,endval,-1))
                    self.startvalue = endval
                    
                if(endval > 800):
                    self.upcounting = False
                    
                if(endval <-250):
                    self.upcounting = True
                    
                self.data =  {
                      "D1": x,
                      "D2": (np.sin(2.0*np.pi * 12.3 * np.array(x) * 0.001)).tolist(),
                      "D3": (np.random.randint(100, size=(100))).tolist()
                }

                self.data_received.release()
    
    def start(self):
        self.thread.start()
    
    def stop(self):
        self.running = False
        #self.wait()

# ---- Serial Reader ----
class SerialReader:
 
    def __init__(self, port, baudrate):
        self.data_received = threading.Semaphore(0)
        self.data_consumed = threading.Semaphore(1)
        self.thread = threading.Thread(target = self.run)
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.data = None
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=2)
            print(f"Connected to {self.port}")
        except Exception as e:
            print(f"Error opening serial port: {e}")
            self.ser = None

    def run(self):
        if not self.ser:
            return

        buffer = b""
        while self.running:
            try:
                buffer += self.ser.read(self.ser.in_waiting or 1)

                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    try:
                        line_str = line.decode("utf-8")
                    except UnicodeDecodeError:
                        continue

                    try:
                        data = json.loads(line_str)
                        if all(len(data.get(k, [])) == 100 for k in ["D1", "D2", "D3"]):
                            if(self.data_consumed.acquire(False)) : 
                                self.data = data
                                self.data_received.release()
                        
                        time.sleep(0.05) # original delay
                        # time.sleep(3)
                    except json.JSONDecodeError:
                        continue

            except Exception as e:
                print(f"Serial read error: {e}")
                break
            
    def start(self):
        self.thread.start()            

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.wait()


def update_data(data):
    # Latency = 0
    # t0 = time.time()  # mark time when JSON received  #LATENCY CALCULATION
    d1, d2, d3 = data.get("D1", []), data.get("D2", []), data.get("D3", [])
    if not all(len(arr) == 100 for arr in [d1, d2, d3]):
        return

    # mass  = 0.008 #Mass asumed to be 8g
    # d4 = [0.5 * 0.008 * ((1.61 + mass * math.sin(i * 0.1)) ** 2) for i in range(100)]
    #KINETIC ENERGY
    mass = 0.008  # Mass in kg (8 g)
    # D4 = Kinetic Energy = 0.5 * m * v^2, where v = D1
    d4 = [0.5 * mass * (v ** 2) for v in d1]
    
  
    # Broadcast to web clients
    try:
        payload = {                             #BROADCAST LOOP
        "D1": d1,
        "D2": d2,
        "D3": d3,
        "D4": d4   # add D4
        }
        #print("updating data")
        
        print("data update")
        
        asyncio.run(broadcast(payload))
      
    except RuntimeError:
        pass
#   # Measure latency and add it to data
#     t1 = time.time()
#     Latency= round((t1 - t0) * 1000, 1)  # in milliseconds
#     print(Latency)

# ---- Main ----
def main():
    # Start FastAPI server in background thread
    server_thread = threading.Thread(target=start_server, daemon=True)
    server_thread.start()

    mySerialReader = DummyReader()  #use this if no STM32 Board is available
    #mySerialReader = SerialReader(SERIAL_PORT, BAUD_RATE)

    mySerialReader.start()
    
    
    while (True): #currently endless loop. TODO: Implement a way to stop, e.g. via Keyboard
        #check for new data
        mySerialReader.data_received.acquire(True)
        print("data received")
        update_data(mySerialReader.data)
        mySerialReader.data_consumed.release()
                
       

    def on_exit():
        mySerialReader.stop()


if __name__ == "__main__":
    main()

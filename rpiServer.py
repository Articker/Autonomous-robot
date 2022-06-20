import zmq.green as zmq
import imagezmq
import imutils
from imutils.video import VideoStream
import cv2

import time
from datetime import datetime
import sys
import multiprocessing 

import serial
import time
import re
#python linux lab_rob: workon cv_p2
#python linux my_robot: workon robot
IMG_PORT = 5555
TXT_PORT = 5556

SRV_IP = '192.168.0.17' #ip of computer, not raspberry!
context = zmq.Context.instance()
#manager = multiprocessing.Manager()
#data_list = manager.list(range(3)) #list with shared DATA (3) [distance,engine_1,engine_2] #in future: actual_speed?
#data_list[0]=int(0)
#data_list[1]=int(0)  
#data_list[2]=int(0)        
def worker_img(port, q):
    
    sender = imagezmq.ImageSender(connect_to="tcp://{}:{}".format(SRV_IP, IMG_PORT))
    vs = VideoStream(src=0, usePiCamera=False, resolution=(640, 480), framerate=20).start() #VideoStream(src=0, usePiCamera=False, resolution=(320, 240), framerate=32, **kwargs) vs = VideoStream(framerate=25).start()
   # vs = VideoStream(src=0,framerate = 60).start()
   # vs=VideoStream(usePiCamera=True).start()
    
    time.sleep(2) #2.0?
    
    print('Started IMG sending.')

    while True:
        frame = vs.read()
        #frame = imutils.resize(frame, width=320) #resizing frames this way
        sender.send_image('rpiName', frame)
        #print('Sent image - {}'.format(datetime.now()))
        time.sleep(0.03)
        if not q.empty() and q.get() == "[Finish]":
            print('Closing')
            break
    
    print('Closing port {}'.format(port))  

def worker_thr(port, q,data_list):
    socket = context.socket(zmq.REP)
    socket.bind('tcp://*:{}'.format(port))
    
    print('Started TXT communication.')
    last_line = ''
    while True:
        if not q.empty() and q.get() == "[Finish]":
            print('Closing')
            break
        msg_from_computer = socket.recv_string()
        message=str(msg_from_computer)
        temp1 = re.findall(r'[+-]?\d+', message) #change to [int]
        res1 = list(map(int, temp1)) #change to [int]
        try:
            line_out1=res1[0]
            line_out_int1=int(line_out1)
            #print(line_out_int) #data to send furhter
        except:
            line_out_int1=int(0)
            print("error line_out_int1")
        try:
            line_out2=res1[1]
            line_out_int2=int(line_out2)
        except:
            line_out_int2=int(0)
            print("error line_out_int2") #no data
        try:
            line_out3=res1[2]
            line_out_int3=int(line_out3)
        except:
            line_out_int3=int(0)
            print("error line_out_int3") #no data
        #print("Instructions from computer: ",line_out1,line_out2,line_out3)

        data_list[1]=line_out_int1
        data_list[2]=line_out_int2
        data_list[3]=line_out_int3
        time.sleep(0.01)
    
        if msg_from_computer == "[Finish]":
            q.put(msg_from_computer)
        line = data_list[0]
        if line != last_line:
            print(line)
            last_line = line
        msg_to_computer=str(line)
        socket.send_string(msg_to_computer)

        if msg_from_computer == "[Finish]":
            break

        #print('Responded')

    print('Closing port {}'.format(port))

def worker_ard(data_list,q):
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout = 1) # ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
    ser.flush()   
    print('Started ARDUINO communication')
    while True:
        try:
            data_ard_1=str(data_list[1])
        except:
            print("error data_ard_1")
            data_ard_1='0'
        try:
            data_ard_2=str(data_list[2]) 
        except:
            print("error data_ard_2")
            data_ard_2='0'
        try:
            data_ard_3=str(data_list[3])
        except:
            print("error data_ard_3")
            data_ard_3='0'
        data_ard='<'
        data_ard+=data_ard_1
        data_ard+=','
        data_ard+=data_ard_2
        data_ard += ','
        data_ard +=data_ard_3
        data_ard+='>'
        #print("Order_to_arduino: ",data_ard)
        if not q.empty() and q.get() == "[Finish]":
            data_ard = '<4,0,0>' #order stop
            break
        ser.write(data_ard.encode()) #sending message to arduino (bytes)
        #time.sleep(0.1)
        line = ser.readline() #listening from arduino
        line=str(line) #data from arduino
        arduino_message = line

        #temp = re.findall(r'\d+', line) #change to [int]
        #res = list(map(int, temp)) #change to [int]
        # try:
        #     line_out=res[0]
        #     line_out_int=int(line_out)
        #     #print(line_out_int) #data to send furhter
        # except:
        #     line_out_int=int(-1)
        #     print("error formating data from arduino") #no data
        try:
            data_list[0]=arduino_message #sending data to another thread (communication with computer)
        except:
            data_list[0]=int(-1)
            print("error getting data from arduino to computer")
       # print("Data from arduino: ",line_out_int)
        #time.sleep(0.01)
        if not q.empty() and q.get() == "[Finish]":
            print('Closing')
            break
    print('Closing arduino')
def main():
    stop_threads = False
    workers = []
 
    q = multiprocessing.Queue()
    thread = multiprocessing.Process(target=worker_img, args=(IMG_PORT,q))
    thread.start()
    workers.append(thread)

    thread = multiprocessing.Process(target=worker_thr, args=(TXT_PORT,q,data_list))
    thread.start()
    workers.append(thread)
    
    thread = multiprocessing.Process(target=worker_ard, args=(data_list,q))
    thread.start()
    workers.append(thread)

    while True:
        #time.sleep(3)
        if q.get() == "[Finish]":
            break
    
    print('main: done sleeping; time to stop the threads.')
    
    stop_threads = True
    for worker in workers:
        worker.terminate()
        worker.join()
    
    print('Finish.')

if __name__ == '__main__':
    manager = multiprocessing.Manager()
    data_list = manager.list(range(4)) #list with shared DATA (3) [distance,engine_1,engine_2] #in future: actual_speed?
    data_list[0]=int(0)
    data_list[1]=int(0)  
    data_list[2]=int(0)
    data_list[3]=int(0)
    main()

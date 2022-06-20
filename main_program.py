0# import the necessary packagesfrom imutils import build_montages
from datetime import datetime
import numpy as np
import imagezmq
import argparse
import imutils
import cv2
import cv2.aruco as aruco
import glob, time, sys, math
# import zmq.green as zmq
import zmq
import sys
import time
import multiprocessing
import msvcrt
import re
import keyboard
#import Arobot as robo
#context = zmq.Context.instance()
context = zmq.Context()

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def video_comm_thr(q,data_list):

    R_flip = np.zeros((3, 3), dtype=np.float32)
    R_flip[0, 0] = 1.0
    R_flip[1, 1] = -1.0
    R_flip[2, 2] = -1.0
    # ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    ret = 0.22432126435808974
    mtx = np.array([[575.1840254, 0., 318.8401027],
                    [0., 574.75627059, 268.86358793],
                    [0., 0., 1.]])
    dist = np.array([[-4.94156815e-01, 4.57936151e-01, -3.71228703e-03, -2.77531283e-04,
                      -4.97755770e-01]])
    rvecs = np.array([np.array([[0.06695983],
                                [0.03120983],
                                [-0.1157639]]), np.array([[0.04030512],
                                                          [0.03066501],
                                                          [-0.1144445]]), np.array([[0.06484676],
                                                                                    [-0.07599645],
                                                                                    [1.49799877]]),
                      np.array([[0.02261672],
                                [-0.03850567],
                                [1.48587337]]), np.array([[-0.52780206],
                                                          [0.2221456],
                                                          [1.39610861]]), np.array([[0.11043421],
                                                                                    [-0.11365745],
                                                                                    [1.4873965]]),
                      np.array([[0.19172639],
                                [-0.20295974],
                                [1.49661345]]), np.array([[-0.28404767],
                                                          [-0.27072491],
                                                          [1.33965773]]), np.array([[-0.0782062],
                                                                                    [-0.32745569],
                                                                                    [1.42261785]]),
                      np.array([[0.52267484],
                                [0.32706446],
                                [1.37220732]]), np.array([[0.06669331],
                                                          [0.77638725],
                                                          [1.25141762]]), np.array([[0.07616305],
                                                                                    [0.77643423],
                                                                                    [1.25142059]]),
                      np.array([[-0.56915586],
                                [0.10175714],
                                [1.89859912]]), np.array([[-0.57697809],
                                                          [0.11318398],
                                                          [1.87315624]]), np.array([[-0.5829132],
                                                                                    [0.12905593],
                                                                                    [1.88207817]]),
                      np.array([[-0.66788936],
                                [-0.07406528],
                                [2.97984278]]), np.array([[0.61721982],
                                                          [-0.21609288],
                                                          [2.94051962]]), np.array([[0.60628216],
                                                                                    [-0.22564793],
                                                                                    [2.93725292]]),
                      np.array([[0.12033113],
                                [-0.07446435],
                                [2.3569586]]), np.array([[-0.56250836],
                                                         [0.00896625],
                                                         [2.37692046]]), np.array([[-0.65854925],
                                                                                   [0.64557449],
                                                                                   [-2.83808166]]),
                      np.array([[0.46994537],
                                [0.84636036],
                                [-2.58396649]]), np.array([[0.47263953],
                                                           [0.83428002],
                                                           [-2.58935772]]), np.array([[0.01146103],
                                                                                      [0.23293901],
                                                                                      [-2.57293806]]),
                      np.array([[-0.65947161],
                                [-0.1685095],
                                [-2.3787672]]), np.array([[0.98740375],
                                                          [-0.30845663],
                                                          [2.81377561]]), np.array([[0.98907452],
                                                                                    [-0.302119],
                                                                                    [2.81037725]]),
                      np.array([[-0.78603683],
                                [-0.03612425],
                                [2.81637964]]), np.array([[-0.37869999],
                                                          [0.66263621],
                                                          [2.94276166]]), np.array([[-0.65895689],
                                                                                    [0.71466173],
                                                                                    [2.7044974]]),
                      np.array([[0.96233231],
                                [-0.71894132],
                                [2.80393219]]), np.array([[-0.67870248],
                                                          [0.77129621],
                                                          [-2.67442917]]), np.array([[-0.83997794],
                                                                                     [-0.91253855],
                                                                                     [2.56985178]]),
                      np.array([[-0.56780244],
                                [-0.97813994],
                                [2.28089276]]), np.array([[0.12547873],
                                                          [-0.98333392],
                                                          [1.98578189]]), np.array([[-0.334935],
                                                                                    [0.87974602],
                                                                                    [-2.40462265]]),
                      np.array([[-0.70696809],
                                [0.28757324],
                                [-2.34132328]]), np.array([[-0.37909851],
                                                           [0.49401067],
                                                           [-1.9739038]]), np.array([[0.09859815],
                                                                                     [0.70663783],
                                                                                     [-2.87592751]]),
                      np.array([[0.87650956],
                                [0.73751016],
                                [-2.8857136]]), np.array([[-0.33111367],
                                                          [-0.7316446],
                                                          [2.5218657]]), np.array([[0.98626331],
                                                                                   [0.63208304],
                                                                                   [-2.91657999]]),
                      np.array([[0.18866022],
                                [0.31876099],
                                [-3.03920905]]), np.array([[-0.85660662],
                                                           [0.42666417],
                                                           [-2.97032846]]), np.array([[-0.94760053],
                                                                                      [0.38518201],
                                                                                      [-2.88956149]]),
                      np.array([[-0.96145883],
                                [0.47189386],
                                [-2.53148925]])])
    tvecs = np.array([np.array([[-103.98825769],
                                [-47.69352788],
                                [420.50932287]]), np.array([[-105.44253287],
                                                            [-49.10345404],
                                                            [423.38178542]]), np.array([[50.32284621],
                                                                                        [-74.48290171],
                                                                                        [410.62423883]]),
                      np.array([[49.4001095],
                                [-150.9007306],
                                [412.66465297]]), np.array([[34.26537953],
                                                            [-71.31044525],
                                                            [540.4944085]]), np.array([[53.20333403],
                                                                                       [-56.80074117],
                                                                                       [421.04468095]]),
                      np.array([[57.51790013],
                                [-49.80948156],
                                [409.02204954]]), np.array([[5.44209462],
                                                            [-113.20836949],
                                                            [465.23362407]]), np.array([[28.83879048],
                                                                                        [-116.83766368],
                                                                                        [454.76832199]]),
                      np.array([[69.47317974],
                                [-137.03215328],
                                [402.14082813]]), np.array([[103.24566333],
                                                            [-91.50707586],
                                                            [512.36597603]]), np.array([[102.54776005],
                                                                                        [-91.44353591],
                                                                                        [510.72838972]]),
                      np.array([[82.47902977],
                                [-34.95215486],
                                [543.53031937]]), np.array([[78.84399501],
                                                            [-39.04806255],
                                                            [547.79536395]]), np.array([[78.7745412],
                                                                                        [-37.07169774],
                                                                                        [549.67076962]]),
                      np.array([[78.33899619],
                                [66.75398444],
                                [487.07927349]]), np.array([[107.45591545],
                                                            [49.4617156],
                                                            [375.94615883]]), np.array([[112.30734817],
                                                                                        [48.54224802],
                                                                                        [377.32105361]]),
                      np.array([[152.99811553],
                                [-26.91320696],
                                [425.04813074]]), np.array([[105.07375819],
                                                            [1.54642065],
                                                            [491.73083905]]), np.array([[108.01415163],
                                                                                        [107.57714609],
                                                                                        [423.93540438]]),
                      np.array([[37.46820726],
                                [53.55934149],
                                [504.65597114]]), np.array([[37.96449489],
                                                            [53.06326675],
                                                            [503.96618243]]), np.array([[41.44780136],
                                                                                        [83.6021579],
                                                                                        [453.669535]]),
                      np.array([[67.31748409],
                                [92.14234534],
                                [427.66670271]]), np.array([[134.69254301],
                                                            [75.23834966],
                                                            [460.79836164]]), np.array([[135.95366254],
                                                                                        [73.27104158],
                                                                                        [463.50941106]]),
                      np.array([[65.15046312],
                                [60.30334742],
                                [549.76441817]]), np.array([[102.99924914],
                                                            [111.31666768],
                                                            [519.60790057]]), np.array([[78.1279045],
                                                                                        [91.40082074],
                                                                                        [559.40576545]]),
                      np.array([[157.58989917],
                                [39.82941199],
                                [446.80927494]]), np.array([[126.25100481],
                                                            [70.36654204],
                                                            [492.81919077]]), np.array([[53.74373587],
                                                                                        [-73.77976119],
                                                                                        [552.72933107]]),
                      np.array([[89.51677991],
                                [-124.19331118],
                                [528.59460189]]), np.array([[106.27236418],
                                                            [-132.48370603],
                                                            [454.40665225]]), np.array([[76.75768679],
                                                                                        [53.8376027],
                                                                                        [526.01965549]]),
                      np.array([[49.59937799],
                                [85.03843398],
                                [441.48189115]]), np.array([[41.10011206],
                                                            [111.06111674],
                                                            [442.64870642]]), np.array([[94.07685719],
                                                                                        [21.75128433],
                                                                                        [457.94640451]]),
                      np.array([[39.19650385],
                                [14.05838638],
                                [465.53239119]]), np.array([[97.67791196],
                                                            [-68.65656017],
                                                            [425.59313928]]), np.array([[40.8073186],
                                                                                        [31.00957634],
                                                                                        [460.26324991]]),
                      np.array([[75.01185031],
                                [54.2195818],
                                [501.67848399]]), np.array([[133.93259885],
                                                            [42.33803617],
                                                            [345.39892497]]), np.array([[117.54502116],
                                                                                        [120.20073433],
                                                                                        [405.45027913]]),
                      np.array([[90.43243516],
                                [135.30914332],
                                [430.72938238]])])
    # ^calibrated for my pi camera^
    #imageHub = imagezmq.ImageHub(open_port='tcp://192.168.0.17:5555')  # ip computer, not raspberry! #VIDEO COMM
    imageHub = imagezmq.ImageHub(open_port='tcp://192.168.157.1:5555')  # ip computer, not raspberry! #VIDEO COMM
    lastActive = {}
    lastActiveCheck = datetime.now()

    ACTIVE_CHECK_PERIOD = 1
    ACTIVE_CHECK_SECONDS = ACTIVE_CHECK_PERIOD

    print("START VIDEO CLIENT")
    start = time.time()

    # rotate a markers corners by rvec and translate by tvec if given
    # input is the size of a marker.
    # In the markerworld the 4 markercorners are at (x,y) = (+- markersize/2, +- markersize/2)
    # returns the rotated and translated corners and the rotation matrix

    def rotate_marker_corners(rvec, markersize, tvec=None):

        mhalf = markersize / 2.0
        # convert rot vector to rot matrix both do: markerworld -> cam-world
        mrv, jacobian = cv2.Rodrigues(rvec)

        # in markerworld the corners are all in the xy-plane so z is zero at first
        X = mhalf * mrv[:, 0]  # rotate the x = mhalf
        Y = mhalf * mrv[:, 1]  # rotate the y = mhalf
        minusX = X * (-1)
        minusY = Y * (-1)

        # calculate 4 corners of the marker in camworld. corners are enumerated clockwise
        markercorners = []
        markercorners.append(np.add(minusX, Y))  # was upper left in markerworld
        markercorners.append(np.add(X, Y))  # was upper right in markerworld
        markercorners.append(np.add(X, minusY))  # was lower right in markerworld
        markercorners.append(np.add(minusX, minusY))  # was lower left in markerworld
        # if tvec given, move all by tvec
        if tvec is not None:
            C = tvec  # center of marker in camworld
            for i, mc in enumerate(markercorners):
                makercorners[i] = np.add(C, mc)  # add tvec to each corner
        # print('Vec X, Y, C, dot(X,Y)', X,Y,C, np.dot(X,Y)) # just for debug
        markercorners = np.array(markercorners, dtype=np.float32)  # type needed when used as input to cv2
        return markercorners, mrv
    #flaga = True
    marker_visivility_count = int(0)

    def points(x_dest, y_dest):
        point_marker_world = np.array([x_dest, 0, y_dest])  # point in marker world where center of it is (0,0,0)
        point_marker_world = point_marker_world.T
        # G = np.array([0,0,0.3])
        destination_all = rot_mat.dot(point_marker_world) + tvec[0]
        # print("DEST ALL: ",destination_all)
        destination_all = np.array(destination_all)
        destination_x = destination_all[0][2]
        destination_y = - destination_all[0][0]  # adjusting for robot odometry world
        x = int(destination_x * 100)  # destination for robot in camera world
        y = int(destination_y * 100)  # destination of point in front of marker for robot in camera world
        return x, y

    while True:
        (rpiName, frame) = imageHub.recv_image()
        imageHub.send_reply(b'OK')
        time.sleep(0.05)
        # if a device is not in the last active dictionary then it means
        # that its a newly connected device
        if rpiName not in lastActive.keys():
            print("[INFO] receiving data from {}...".format(rpiName))
            data_list[33] = 1
        # record the last active time for the device from which we just
        # received a frame
        lastActive[rpiName] = datetime.now()

        key = cv2.waitKey(1)

        if (datetime.now() - lastActiveCheck).seconds > ACTIVE_CHECK_SECONDS:
            # loop over all previously active devices
            for (rpiName, ts) in list(lastActive.items()):
                # remove the RPi from the last active and frame
                # dictionaries if the device hasn't been active recently
                if (datetime.now() - ts).seconds > ACTIVE_CHECK_SECONDS:
                    print("[INFO] lost connection to {}".format(rpiName))
                    lastActive.pop(rpiName)
                    frameDict.pop(rpiName)

            # set the last active check time as current time
            lastActiveCheck = datetime.now()
        end = time.time()
        # ret, frame = cap.read()

        # image = np.frombuffer(frame, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))

        # operations on the frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # set dictionary size depending on the aruco marker selected
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

        # detector parameters can be set here (List of detection parameters[3])
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 10

        # lists of ids and the corners belonging to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # font for displaying text (below)
        font = cv2.FONT_HERSHEY_SIMPLEX

        # check if the ids list is not empty
        # if no check is added the code will crash
        
        if np.all(ids != None):
            data_list[30] = 1
            if data_list[32] == 0:
                data_list[29] = 1 #first time marker found : true
                data_list[32] = 1
            if data_list[28] == 1: #if check for marker is true then stop the robot by sending 4,0,0 command stop_robot
                data_list[27] = 4 #marker is in sight and it can stop rotating
                data_list[1] = 0
                data_list[2] = 0
                data_list[28] = 0
            # estimate pose of each marker and return the values
            # rvet and tvec-different from camera coefficients
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.099, mtx, dist) #for marker of size 9,9cm
            (rvec - tvec).any()  # get rid of that nasty numpy value array error

            rvec = rvec[0, :]  # unpacking rvec
            Ttvec = tvec[0,:] #unpaing tvecs as well?? for camera position
            Ttvec = Ttvec.T
            #print("tvec", Ttvec)
            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            rot_mat = R_ct
            #x_dest = 0
            #y_dest = np.array([0.20,0.20,0.20,0.20])  #qfind point 36cm in fron of marker ### can be change for more smooth route


            xy_dest = points(0,0.40)
            data_list[19] = xy_dest[0] # actual point X 45cm in front of the marker 30 cm from camera best
            data_list[20] = xy_dest[1]  # actual point X 45cm in front of the marker

            R_tc = R_ct.T

            cameraPosition = -R_tc.dot(Ttvec)
            cameraPosition = np.array(cameraPosition)
            data_list[13] = cameraPosition[0]
            data_list[14] = cameraPosition[1]
            data_list[15] = cameraPosition[2]

            marker_coordinates = tvec
            marker_coordinates = marker_coordinates[0]
            marker_coordinates = marker_coordinates[0]
            vector = math.degrees(math.atan(math.tan((marker_coordinates[0] - data_list[20]) / (marker_coordinates[2] - data_list[19])))) #angle between vector in front of marker and robot
            data_list[18] = int(vector)
            data_list[16] = int(marker_coordinates[0]*100)
            data_list[17] = int(marker_coordinates[2]*100)

            role_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(
                R_flip * R_tc)  # roll - 0, pitch - 1 , yaw - 2

            try:
                data_list[4] = math.degrees(role_marker) # angle of the marker [role_marker]
            except:
                data_list[4] = int(0)
                print("error angle role_marker")
            try:
                data_list[5] = math.degrees(pitch_marker)  # marker angle [pitch marker] [main angle]
            except:
                data_list[5] = int(0)
                print("error MAIN angle pitch_marker")
            try:
                data_list[6] = math.degrees(yaw_marker)  # marker angle [yaw_marker]
            except:
                data_list[6] = int(0)
                print("error angle yaw_marker")

            strAttitude = "MARKER Attitude r=%4.1f, p=%4.1f y=%4.1f" % (
                math.degrees(role_marker), math.degrees(pitch_marker), math.degrees(yaw_marker))

            #for i in range(0, ids.size):
                # draw axis for the aruco markers
                #aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1)

            # draw a square around the markers
            aruco.drawDetectedMarkers(frame, corners)

            # code to show ids of the marker found
            strg = ''
            for i in range(0, ids.size):
                strg += str(ids[i][0]) + ', '

            tvecU = tvec[0, 0, :]
            #cv2.putText(frame, strAttitude, (0, 50), font, 0.4, (0, 255, 0), 2, cv2.LINE_AA)

            #str_position = "MARKER Position x=%4.1f y=%4.1f z=%4.1f" % (tvecU[0] * 100, tvecU[1] * 100, tvecU[2] * 100)
            # str_position = "MARKER Position x=%4.4f y=%4.4f z=%4.4f" % (R_tc[0], R_tc[1], R_tc[2])
            #cv2.putText(frame, str_position, (0, 80), font, 0.4, (0, 255, 255), 2, cv2.LINE_AA)
            #data_list[3] = 1  #ID visible , but still NO data which one

            corners_processing = np.array(corners)
            corners_processed = corners_processing[0][0]  # unpacking coordinations of corners

            center_of_marker = int((corners_processed[0][0] + corners_processed[1][0]) / 2)
            width_of_marker = int((corners_processed[0][0] - corners_processed[1][0]))
            #visual_distance = tvecU[2] * 100 #marker z data_list[12]
            # width of marker for distance measure
            data_list[7] = center_of_marker
            data_list[8] = width_of_marker
            data_list[10] = tvecU[0] * 100 #position x marker
            data_list[11] = tvecU[1] * 100 #position y marker
            data_list[12] = tvecU[2] * 100 #position z marker #visual distance
        else:
            cv2.putText(frame, "No Marker", (0, 64), font, 0.4, (0, 255, 0), 2, cv2.LINE_AA)
            data_list[3] = -1 #no ID visible
            data_list[30] = 0 #marker is not visible at the moment
        cv2.imshow("robot", frame)
        if data_list[29] == 1:
            data_list[31] = 1 #is not first blind search
        if key == ord("q"):
            data_list[9] = "q"
        if key == ord("p"):
            data_list[9] = "p"

def txt_comm_thr(q,data_list):
    socket = context.socket(zmq.REQ)
    socket.connect('tcp://192.168.157.31:5556')  # ip raspberry pi, port txt
    #socket.connect('tcp://192.168.162.31:5556')  # ip raspberry pi, port txt
    start = time.time()
    print('START TXT CLIENT.')
    switch_flag = False
    last_msg_xyf = "<0,0,0>"
    while True:
        end = time.time()
        if end - start >= 0.5: #every one 0.1 second #0.5??
            try:
                msg_x = str(data_list[1])
            except:
                msg_x = '0'
                print("error msg_x")
            try:
                msg_y = str(data_list[2])
            except:
                msg_y = '0'
                print("error msg_y")
            try:
                msg_f = str(data_list[27])
            except:
                msg_f = '0'
                print("error msg_f")
            msg_xyf = '<'
            msg_xyf += msg_f
            msg_xyf += ','
            msg_xyf += msg_x
            msg_xyf += ','
            msg_xyf += msg_y
            msg_xyf += '>'
            if msg_xyf != last_msg_xyf:
                print(msg_xyf)
                last_msg_xyf = msg_xyf
            if data_list[9] =="q":
                socket.send_string("[Finish]")
            else:
                socket.send_string(str(msg_xyf))
                msg_dist = socket.recv_string() #distance data
            try:
                data_list[0] = msg_dist
            except:
                data_list[0] = int(0)
                print("error msg_dist")
            start = time.time()
        #if data_list[9] == "q":
            #break

def control_thr(q,data_list): #Control system
    print("Started ROBOT CONTROL")
    last_time_marker = last_time_data = last_time_recive_data  = last_time_recive_data_ard_state = time.time()
    center_of_camera = 320  # (width of camera image / 2)  640x480 resolution
    visible = False
    index = int(0)
    ammount_of_data = int(2)
    serie_len = int(10)
    res_times = next_numbers = datas = no_zeros = medians = np.zeros(ammount_of_data)
    data_tables = np.zeros((ammount_of_data, serie_len))
    last_times = np.zeros(ammount_of_data, dtype='float64')
    arduino_ready = False
    global order_sent_nb
    order_sent_nb = int(0)# how many message was sent to arduino
    global comma_sent_nb
    comma_sent_nb = int(0)
    global objective_sent_nb
    objective_sent_nb = int(0)
    global stage
    stage = "idling"
    last_movement = "left"
    first_time_marker = 0
    next_stage = "watching"
    marker_was_found = False
    automatic_mode = True
    last_stage = stage
    last_turning = "left"

    keyboard_speed = int(90)
    keyboard_stop_flag = True
    keyboard_w_flag = True
    keyboard_s_flag = True
    keyboard_a_flag = True
    keyboard_d_flag = True

    approaching_stop_flag = True
    for i in range(ammount_of_data):
        last_times[i] = time.time()
        res_times[i] = 0.2

    def check_zeros(table):  # to check if is enough data for first decision
        for i in range(len(table)):
            if table[i] == 0:
                checked_zeros = False
                break
            else:
                checked_zeros = True
        return checked_zeros

    def marker_visibility(last_time_marker,id_marker,visible):
        if time.time() - last_time_marker >= res_times[1]:
            if data_list[30] == 1:  # if marker is visible - id is from 0 to ~1000
                visible = True
            else:  # no Id in camera
                visible = False
        last_time_marker = time.time()
        return visible,last_time_marker

    def distance_correct(distance,center_of_marker): #correcting distance due to some camera distortion effects
        #if distance > 100 and abs(center_of_marker - center_of_camera) > (2*(center_of_camera/3): #above 1m on edge of camera error is becoming significant
        if distance > 70 and abs(center_of_marker - center_of_camera) > (4 * (center_of_camera/5)): #close to the edge distortion is more significant
            distance = distance + distance * 0.07
        if distance > 150 and abs(center_of_marker - center_of_camera) < (3 * (center_of_camera/5)):
            distance = distance - distance * 0.06 #correcting overestimation on the center
        return distance

    def send_order(type,data_1,data_2): #sending data to txt communiaction thread// type = state, data_1 = X / angle, data_2 = Y
        data_list[27] = type
        data_list[1] = data_1
        data_list[2] = data_2
        return
    def decode_message(raw_text):
        arduino_message = data_list[0]
        temp = re.findall(r'[+-]?\d+', raw_text) #change to [int]
        message_list = list(map(int, temp)) #change to [int]
        return message_list
    def order_turn(angle):
        angle = int(angle)
        send_order(3,angle,0)
        return
    def order_global_xy(x,y):
        #time.sleep(0.2)  # wait before ordering
        x = int(x)
        y = int(y)
        send_order(1,x,y)
        return
    def order_local_xy(x,y):
        #time.sleep(0.2)  # wait before ordering
        x = int(x)
        y = int(y)
        send_order(2, x, y)
        return
    def order_stop():
        global stage
        send_order(4,0,0)
        stage = 'after_stop'
        return
    def order_comma(): #to communicate arduino that new message will come
        time.sleep(0.1)
        send_order(9,0,0)
        time.sleep(0.4)
        comma_sent()
        return
    def last_marker():
        difference = center_of_camera - center_of_marker
        if difference >= 0:
            side = 0
        else:
            side = 1
        return side

    def order_turning(speed):
        #time.sleep(0.5)  # wait before ordering
        send_order(5,speed,0)
        return
    def order_center_correction(center_of_marker):
        center_diff = center_of_marker - center_of_camera
        flag_center_corr = True
        center_correction_process = True
        while center_correction_process == True:
            if flag_center_corr == True:
                time_of_center_correction = time.time()
                flag_center_corr = False
            if center_diff > 5 and center_diff < 15:
                send_order(7,-1,1)
            elif center_diff > 15:
                send_order(7,-2,2)
            elif center_diff < -5 and center_diff > -15:
                send_order(7, 1, -1)
            elif center_diff < -15:
                send_order(7, 2, -2)
            elif abs(center_diff) <= 5:
                send_order(7, 0, 0)
                print("robot is set with center of marker")
                center_correction_process = False
            if time.time() - time_of_center_correction > 10:
                center_correction_process = False
                stage = "watching"
        return

    def order_center_correction_angle(center_of_marker):
        global stage
        center_diff = center_of_marker - center_of_camera
        if center_diff > 10:
            order_turn(-2)
        elif center_diff < -10:
            order_turn(2)
        else:
            print("error order center corr angle")
        stage = "watching"
        return


##### COMUNICATION METHODS #####

    def order_sent():
        global order_sent_nb
        global stage
        order_sent_nb += 1
        stage = "sending_order"
        return

    def comma_sent():
        global comma_sent_nb
        global stage
        comma_sent_nb += 1
        stage = "waiting_for_comma_confirmation"
        return

    def listen_confirmation_order():
        global order_sent_nb
        global objective_sent_nb
        if order_sent_nb == arduino_message:
            global stage
            stage = "waiting_for_objective_confirmation"
            objective_sent_nb += 1
        return

    def listen_confirmation_objective():
        global objective_sent_nb
        if objective_sent_nb == arduino_objective:
            global stage
            stage = "sending_comma"
        return

    def listen_confirmation_comma():
        global comma_sent_nb
        if comma_sent_nb == arduino_comma:
            global stage
            stage = "back_to_control"
        return
##### END OF COMUNICATION METHODS #####

#### DATA METHODS ####

    def to_point_distance(x, y):
        p1 = [0, 0]
        p2 = [x, y]
        distance = math.sqrt(((p1[0] - p2[0]) ** 2) + ((p1[1] - p2[1]) ** 2))
        return distance

    def to_line_distance(x, y, x_15):
        p1 = np.array([0, 0])
        p2 = np.array([x, y])
        p3 = np.array([x_15, y])
        distance = np.cross(p2 - p1, p3 - p1) / np.linalg.norm(p2 - p1)
        return distance

#### END OF DATA METHODS ####

    def find_marker(side_of_screen):
        if side_of_screen == "no":
            order_turning(15)
            data_list[28] = 1 #set to find order to stop when it will be seen
        if side_of_screen == "left":
            order_turning(10)
            data_list[28] = 1  # set to find order to stop when it will be seen
        if side_of_screen == "right":
            order_turning(-10)
            data_list[28] = 1  # set to find order to stop when it will be seen
        return

    def find_marker_angle():
        order_turn(30)
        data_list[28] = 1  # set to find order to stop when it will be seen
        return

    def print_data():
        try:
            print("X: ", arduino_X_global_pos)
            print("Y: ", arduino_Y_global_pos)
            print("X_local: ", arduino_X_local_pos)
            print("Y_local: ", arduino_Y_local_pos)
            print("arduino state: ", arduino_state)
            print("arduino message: ", arduino_message)
            print("arduino objective: ", arduino_objective)
            print("arduino comma: ", arduino_comma)
            print("X_global_wanted: ", X_global_wanted)
            print("Y_global_wanted: ", Y_global_wanted)
            print("angle(marker_x): ", marker_x)
            print("angle(angle): ", angle)
            print("center_of_marker: ", center_of_marker)
            print("arduino_theta_global: ", arduino_theta_global)
            print("theta_local: ", arduino_theta_local)
            print("distance: ", distance)
            print("visible: ", visible)
            print("arduino_dist_obs: ", arduino_dist_obs)
            print("objective_sent: ", objective_sent_nb)
            print("comma_sent: ", comma_sent_nb)
            print("Stage: ",stage)
            print("arduino_xd_local: ",arduino_xd_local)
            print("arduino_yd_local: ",arduino_yd_local)
            #print("vector: ",vector)
            print(" ")
        except:
            print("error display xy")
        return

    def stage_hist(last_stage,new_stage):
        if new_stage != last_stage:
            print("Stage: ", stage)
            last_stage = new_stage
        return last_stage

    def manual_control(keyboard_speed,keyboard_stop_flag,keyboard_w_flag,keyboard_s_flag,keyboard_a_flag,keyboard_d_flag):
        if keyboard.is_pressed("1"):
            keyboard_speed = 40
        if keyboard.is_pressed("2"):
            keyboard_speed = 80
        if keyboard.is_pressed("3"):
            keyboard_speed = 120
        if keyboard.is_pressed("4"):
            keyboard_speed = 160

        if keyboard.is_pressed("w+a"):
            send_order(7, keyboard_speed, int(keyboard_speed/2))
            keyboard_w_flag = False
            keyboard_a_flag = False
        elif keyboard.is_pressed("w+d"):
            send_order(7, int(keyboard_speed/2), keyboard_speed)
            keyboard_w_flag = False
            keyboard_d_flag = False
        elif keyboard.is_pressed("w") and keyboard_w_flag == True:
            send_order(7, keyboard_speed, keyboard_speed)

        elif keyboard.is_pressed("s+a"):
            send_order(7, -keyboard_speed, -int(keyboard_speed/2))
            keyboard_s_flag = False
            keyboard_a_flag = False
        elif keyboard.is_pressed("s+d"):
            send_order(7, -int(keyboard_speed/2), -keyboard_speed)
            keyboard_s_flag = False
            keyboard_d_flag = False
        elif keyboard.is_pressed("s") and keyboard_s_flag == True:
            send_order(7, -keyboard_speed, -keyboard_speed)

        elif keyboard.is_pressed("a") and keyboard_a_flag == True:
            send_order(7, keyboard_speed, 0)
        elif keyboard.is_pressed("d") and keyboard_d_flag == True:
            send_order(7, 0, keyboard_speed)
        else:
            if keyboard_stop_flag == True:
                send_order(9, 0, 0)
                keyboard_stop_flag = False
            else:
                send_order(4, 0, 0)
                keyboard_stop_flag = True
        keyboard_a_flag = True
        keyboard_d_flag = True
        keyboard_w_flag = True
        keyboard_s_flag = True
        return keyboard_stop_flag,keyboard_w_flag,keyboard_s_flag,keyboard_a_flag,keyboard_d_flag

    def approaching(angle,distance,center_of_marker,approaching_stop_flag):
        if distance > 20:
            if abs(center_of_marker - center_of_camera < 10):
                send_order(7, 50,50)
            else:
                if center_of_marker < center_of_camera:
                    send_order(7, 50, 45)
                else:
                    send_order(7, 45, 50)
        else:
            send_order(7,0,0)
        return

    last_time_mesg = time.time()
    starting_flag = False
    arduino_message_last = -1
    arduino_comma_last = -1
    arduino_objective_last = -1
    arduino_read_flag = True
    xy_local_flag = False
    flag_x_wanted = False
    count_blind = int(0)
    time_starttime = time.time()
    last_time_recive_data_2 = time.time()
    xy_local_flag_search = False

    while True:
        if automatic_mode == True:  # if autonomus mode is on
                if arduino_ready == True and stage == "idling":  # do it once only
                    if data_list[33] == 1:
                        stage = "watching"  # if arduino is ready start movement
        time_running = time.time() - time_starttime
        if automatic_mode == False: #robot controled by WSAD keyboard
            keyboard_flags=manual_control(keyboard_speed,keyboard_stop_flag,keyboard_w_flag,keyboard_s_flag,keyboard_a_flag,keyboard_d_flag)
            keyboard_stop_flag = keyboard_flags[0]
            keyboard_w_flag = keyboard_flags[1]
            keyboard_s_flag = keyboard_flags[2]
            keyboard_a_flag = keyboard_flags[3]
            keyboard_d_flag = keyboard_flags[4]
        else: #automatic mode ON
            try:
                if arduino_dist_obs > 0 and arduino_dist_obs < 8:
                    order_stop()
            except:
                None
            #### AUTONOMOUS CONTROL SYSTEM ##########
            if stage == "after_stop":
                time.sleep(2)
                send_order(15,0,0)
                data_list[29] = 0
                data_list[32] = 0
                first_time_marker = 0
                time.sleep(2)
                next_stage = 'watching'
                X_global_wanted = data_list[19] + 15  # to move objective point 15cm from camera position to center of wheels to match arduino odometry
                Y_global_wanted = data_list[20]
                xy_local_flag = False
                stage = 'back_to_control'

            arduino_data = data_list[0]  # distance from obstacle from arudino sonar
            id_marker = data_list[3]  # marker id (-1 is null)
            role_angle = data_list[4]  # marker angle [role_marker] //looking from higher ground
            pitch_marker = data_list[5]  # marker angle [pitch marker] [main angle]
            yaw_angle = data_list[6]  # marker angle [yaw_marker] //is it rotated?
            center_of_marker = data_list[7]  # marker center on the frame
            width_of_marker = data_list[8]  # marker width on the frame
            x_marker = data_list[10]  # position x marker
            y_marker = data_list[11]  # position y marker
            z_marker = data_list[12]  # position z marker #visual distance
            visual_distance = data_list[12]  # visual distance
            camera_x = data_list[13]  # coordinate of camera X
            camera_y = data_list[14]  # coordinate of camera Y
            camera_z = data_list[15]  # coordinate of camera Z
            marker_x = data_list[16]  # coordinate of marker X
            marker_y = data_list[17]  # coordinate of marker Y
            vector = data_list[18]  # vector for heading towards marker ??
            X_global_wanted = data_list[19] + 15  # to move objective point 15cm from camera position to center of wheels to match arduino odometry
            Y_global_wanted = data_list[20]
            X_global_wanted_15 = X_global_wanted - 15
            marker_stop = data_list[28]
            marker_visible = data_list[30]
            # visual distance, angle, center of marker, marker_visibility
            real_distance = distance_correct(visual_distance, center_of_marker)  # correcting distance

            #######----------------------------------------/calculating medians
            if time.time() - last_time_data > 0.1:
                data_tables[0][index] = real_distance
                data_tables[1][index] = pitch_marker
                index += 1
                if index == serie_len:
                    index = 0
                medians[0] = np.median(data_tables[0][:])
                medians[1] = np.median(data_tables[1][:])
                last_time_data = time.time()
            #######----------------------------------------/calculating medians

            #######++++++++++++++++++++++++++++++++++++++++++++++++++++++
            visibility = marker_visibility(last_time_marker, id_marker, visible)
            visible = visibility[0]  # checking visibility of marker
            last_time_marker = visibility[1]
            check_1 = check_zeros(data_tables[0][:])
            check_2 = check_zeros(data_tables[:][0])
            if check_1 == True and check_2 == True:
                checked = True  # marker was seen and its position is known
            angle = medians[1]
            distance = medians[0]
            #######++++++++++++++++++++++++++++++++++++++++++++++++++++++
            if time.time() - last_time_recive_data > 0.1: #check for update from arduino every x miliseconds
                if time.time() - last_time_recive_data_2 > 1:
                    try:
                        print("arduino_data: ",arduino_data)
                    except:
                        None
                    last_time_recive_data_2 =time.time()
                try:
                    message_list = decode_message(arduino_data)
                    arduino_state = message_list[9] #message_list[0]
                    arduino_X_global_pos = message_list[1]
                    arduino_Y_global_pos = message_list[2]
                    arduino_theta_global = message_list[3]
                    arduino_X_local_pos = message_list[4]
                    arduino_Y_local_pos = message_list[5]
                    arduino_theta_local = message_list[6]
                    arduino_message = message_list[7] #+1 when message is read
                    arduino_objective = message_list[8] #+1 when objective is done
                    arduino_comma = message_list[0] #+1 when comma is read #arduino_comma = message_list[9]
                    arduino_dist_obs = message_list[10] #distance from arduino sonar
                    #arduino_xd_local = message_list[11] #current arduino goals XY local
                    #arduino_yd_local = message_list[12] #current arduino goals XY local
                    starting_flag = True
                except:
                    starting_flag = False
                    #print("Warning: cannot decode_message ")
                last_time_recive_data = time.time()

            if starting_flag == True:
                if arduino_message != arduino_message_last or arduino_objective != arduino_objective_last or arduino_comma != arduino_comma_last:
                    arduino_message_last = arduino_message
                    arduino_objective_last = arduino_objective
                    arduino_comma_last = arduino_comma
                    print("arduino_message: ", arduino_message)
                    print("arduino_objective: ", arduino_objective)
                    print("arduino_comma: ", arduino_comma)
                    time_spent = time.time() - last_time_mesg
                    print("time: ",time_spent)
                    last_time_mesg = time.time()



            if time.time() - last_time_recive_data_ard_state > 1:
   
                try:
                    if arduino_state == 1:
                        arduino_ready = True
                        if arduino_read_flag == True:
                            time.sleep(2)
                            arduino_read_flag = False
                    else:
                        arduino_ready = False
                except:
                    print("warning: arduino ready error")
                last_time_recive_data_ard_state = time.time()


#### AUTONOMOUS CONTROL SYSTEM ##########
            #if stage == "watching":
                #approaching_XY(angle,distance,center_of_marker,approaching_stop_flag,vector)
        visibility = marker_visibility(last_time_marker, id_marker, visible)
        visible = visibility[0]  # checking visibility of marker
        last_time_marker = visibility[1]

        if stage == "watching":
            time.sleep(0.5)
            last_stage = stage_hist(last_stage,stage)
            if data_list[32] == 1 and angle != 0.0: #and data_list[30]
                data_list[32] = 2
                angle_theta_correction = angle #angle  # correction for robot using global theta and first time seen vector marker
                first_seen_theta = arduino_theta_global #-
                print("angle_theta_correction: ",angle_theta_correction)
                print("first_seen_theta: ",arduino_theta_global)
                first_time_marker = 1

            if data_list[30] == 1: #visible = true
                #if data_list[19] > 2:  # to move objective point 15cm from camera position to center of wheels to match arduino odometry
                stage = "analyse_road"
            else:
                if first_time_marker == 1 and xy_local_flag_search == True:
                    stage = "counscious_search"
                else:
                    stage = "blind_search"

        if stage == "blind_search": #rotate in place by fixed angle
            time.sleep(0.5)
            last_stage = stage_hist(last_stage, stage)
            if data_list[31] == 0:
                fixed_tur_value = 25
            else:
                fixed_tur_value = 10
            if last_turning == "right":
                fixed_turn = int(fixed_tur_value)
            else:
                fixed_turn = int(-fixed_tur_value)
            order_turn(fixed_turn)
            order_sent()

        if stage == "counscious_search": #rotate towards marker considering last information about odometry and angle
            time.sleep(0.5)
            last_stage = stage_hist(last_stage, stage)
            # angle error that should be now in global theta -> fastest way to rotate to marker
            #odometry_correction_turn = arduino_theta_global - first_seen_theta + angle_theta_correction
            odometry_correction_turn = first_seen_theta - arduino_theta_global + angle_theta_correction
            print("arduino_theta_global: ",arduino_theta_global)
            print("first_seen_theta: ", first_seen_theta)
            print("angle_theta_correction: ",angle_theta_correction)
            print("ordering: ", odometry_correction_turn)
            time.sleep(0.5)
            if odometry_correction_turn <= 0:
                last_turning == "right"
            else:
                last_turning == "left"
            order_turn(odometry_correction_turn)
            data_list[29] = 0 #marker was found -> false
            data_list[32] = 0
            order_sent()
            first_time_marker = 0
            xy_local_flag_search = False

        if stage == "analyse_road":
            time.sleep(0.5) # ***
            visibility = marker_visibility(last_time_marker, id_marker, visible)
            visible = visibility[0]  # checking visibility of marker
            last_time_marker = visibility[1]
            if data_list[30] == 0:
                stage = "watching"
            last_stage = stage_hist(last_stage, stage)
            maximum_distance_to_point = int(13)
            to_point = to_point_distance(X_global_wanted, Y_global_wanted)
            if to_point > maximum_distance_to_point and xy_local_flag == False:
                X_global_wanted = data_list[19] + 15  # to move objective point 15cm from camera position to center of wheels to match arduino odometry
                Y_global_wanted = data_list[20]
                if X_global_wanted > 15:
                    stage = "approaching_XY"
                else:
                    stage = "watching"
            else:
                maximum_distance_to_line = int(10)
                to_line = to_line_distance(X_global_wanted, Y_global_wanted,X_global_wanted_15)
                to_line = abs(to_line)
                #if to_line < 0: ### ***
                if to_line > maximum_distance_to_line:
                    stage = "adjusting_to_line_rotating"
                else:
                    angle_alfa = angle
                    maximum_angle_to_marker_surface = int(100)
                    if abs(angle_alfa) > maximum_angle_to_marker_surface:
                        stage = "order_angle_alfa"
                    else:
                        center_error = center_of_marker - center_of_camera
                        maximum_center_error = int(25)
                        if abs(center_error) > maximum_center_error:
                            stage = "order_center_correction"
                        else:
                            maximum_visual_distance = int(15)
                            if visual_distance - maximum_visual_distance > 5:
                                stage = "forward_marker"
                            else:
                                stage = "stop"

        if stage == "approaching_XY":
            last_stage = stage_hist(last_stage, stage)
            order_local_xy(X_global_wanted, Y_global_wanted)
            order_sent()
            xy_local_flag = True
            xy_local_flag_search = True

        if stage == "adjusting_to_line_rotating":
            last_stage = stage_hist(last_stage, stage)
            #using angle alfa rotate robot to 90 degrees towards line from center of the marker
            #angle alfa -> angle between surface of camera and surface of the marker
            angle_alfa = angle
            #angle_90 -> angle that set robot parallel to the surface of the marker in direction to center
            # desired direction of turn
            if Y_global_wanted > 0:
                last_turning = "left"
                if angle_alfa > 0:
                    angle_90 = 90 - angle_alfa
                if angle_alfa <= 0:
                    angle_90 = 90 + angle_alfa
                print("angle_90: ",angle_90)
            else:
                last_turning = "right"
                if angle_alfa > 0:
                    angle_90 = -90 - angle_alfa
                if angle_alfa <= 0:
                    angle_90 = -90 + angle_alfa
            order_turn(angle_90)
            next_stage = "adjusting_to_line_forward"
            order_sent()

        if stage == "adjusting_to_line_forward":
            last_stage = stage_hist(last_stage, stage)
            #if to_line -
            if Y_global_wanted > 0:
                number = to_line - 2 #-2cm when approaching line in front of marker [from right side]
            else:
                number = to_line + 2
            if number > 0:
                order_local_xy(number, 0)
            else:
                order_local_xy(to_line,0) #was -5cm for tests
            next_stage = "adjusting_to_line_back_rotating"
            order_sent()

        if stage == "adjusting_to_line_back_rotating":
            last_stage = stage_hist(last_stage, stage)
            if last_turning == "left":
                order_turn(-90)
                #last_turning = "right"
            else:
                order_turn(90)
                #last_turning = "left"
            next_stage = "watching"
            order_sent()

        if stage == "order_angle_alfa":
            last_stage = stage_hist(last_stage, stage)
            order_turn(angle_alfa)
            order_sent()

        if stage == "order_center_correction":
            last_stage = stage_hist(last_stage, stage)
            order_center_correction_angle(center_of_marker)
            order_sent()

        if stage == "forward_marker":
            last_stage = stage_hist(last_stage, stage)
            remaining_distance = int(visual_distance - maximum_visual_distance)
            order_local_xy(remaining_distance,0)
            order_sent()

        if stage == "stop":
            send_order(2,18,0)
            time.sleep(7)
            last_stage = stage_hist(last_stage, stage)
            print("Objective completed")
            data_list[9] = "q"

#### END OF AUTONOMOUS CONTROL SYSTEM ####
#### COMMUNICATION SYSTEM ###########

        if stage == "sending_order":
            last_stage = stage_hist(last_stage, stage)
            listen_confirmation_order()
        if stage == "waiting_for_objective_confirmation":
            last_stage = stage_hist(last_stage, stage)
            listen_confirmation_objective()
        if stage == "sending_comma":
            last_stage = stage_hist(last_stage, stage)
            order_comma()
        if stage == "waiting_for_comma_confirmation":
            last_stage = stage_hist(last_stage, stage)
            listen_confirmation_comma()
        if stage == "back_to_control":
            last_stage = stage_hist(last_stage, stage)
            stage = next_stage

#### END OF COMMUNICATION SYSTEM ####


def main():
    stop_threads = False
    workers = []

    q = multiprocessing.Queue()

    thread = multiprocessing.Process(target=txt_comm_thr, args=(q,data_list))
    thread.start()
    workers.append(thread)

    thread = multiprocessing.Process(target=video_comm_thr, args=(q,data_list))
    thread.start()
    workers.append(thread)

    thread = multiprocessing.Process(target=control_thr, args=(q,data_list))
    thread.start()
    workers.append(thread)

    while True:
        if data_list[9]  == "q":
            cv2.destroyAllWindows()
            print('main: done sleeping; time to stop the threads.')
            stop_threads = True
            for worker in workers:
                worker.terminate()
                worker.join()
            break

    print('All finished.') #End of program.

if __name__ == '__main__':
    manager = multiprocessing.Manager() #launch manager to shared multiprocess DATA
    data_list = manager.list(range(34))  # create list with shared multiprocess DATA
    ### Declaring types of data:
    data_list[0] = ""    # distance from obstacle
    data_list[1] = int(0)     # desired X point for arduino
    data_list[2] = int(0)     # desired Y point for arduino
    data_list[3] = int(-1)    # marker id (-1 is null)
    data_list[4] = float(-1)  # marker angle [role_marker]
    data_list[5] = float(0)   # marker angle [pitch marker] [main angle]
    data_list[6] = float(-1)  # marker angle [yaw_marker]
    data_list[7] = float(-1)  # marker center on the frame
    data_list[8] = float(-1)  # marker width on the frame
    data_list[9] = "x"        # passing chars
    data_list[10] = float(-1) # position x marker
    data_list[11] = float(-1) # position y marker
    data_list[12] = float(0)  # position z marker
    data_list[13] = float(0)  # position of camera x
    data_list[14] = float(0)  # position of camera y
    data_list[15] = float(0)  # position of camera z
    data_list[16] = int(0)    # x postion of marker in cam world
    data_list[17] = int(0)    # y postion of marker in cam world
    data_list[18] = int(0)    # angle between vector in front of marker and robot for arduino
    data_list[19] = int(0)    # marker 1 x coordinates - main coordi
    data_list[20] = int(0)    # marker 1 y coordinates - main coordi
    data_list[21] = int(0)  # marker 2 x coordinates
    data_list[22] = int(0)  # marker 2 y coordinates
    data_list[23] = int(0)  # marker 3 x coordinates
    data_list[24] = int(0)  # marker 3 y coordinates
    data_list[25] = int(0)  # marker 4 x coordinates
    data_list[26] = int(0)  # marker 4 y coordinates
    data_list[27] = int(0)  # type of order
    data_list[28] = int(0) # check_for_marker for fast stop when marker is seen by another thread
    data_list[29] = int(0) # 0 deflaut, 1 - when in this loop marker is first time seen to remeber its position relative to robot
    data_list[30] = int(0) # is marker visible at the moment
    data_list[31] = int(0) #is first blind search
    data_list[32] = int(0) #is first time seeing marker
    data_list[33] = int(0) #video stream ready
    main() #start of main loop

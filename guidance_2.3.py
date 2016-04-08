#attempt at guidance No. 1

#=============================================================================================
# import modules
#=============================================================================================

import socket
import cv2
import numpy as np

#=============================================================================================
# webcam setup
#=============================================================================================

kernel = np.ones((5,5),np.uint8)

#input from webcam
cap = cv2.VideoCapture(0)

# set the screen size to 640x480
cap.set(3,640)
cap.set(4,480)

def nothing(x):
    pass

#=============================================================================================
# setting variables
#=============================================================================================

# connection info
s = socket.socket()
host = '10.0.0.104' #ip of raspberry pi
port = 9001                                           #      <===== port number here
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)     #allows port to be reused
s.bind((host, port))


# info from robot
info = {'name' : 0 , 'colour' : 0, 'midc' : 0 , 'oc' : 0}
color = {'hmn' : 0, 'hmx': 1, 'smn' : 0, 'smx': 0, 'vmn' : 0, 'vmx': 1}

# variables for finding the exact location of the robot
co1 = {'xco' : 1 , 'yco' :2, 'size' :3}
co2 = {'xco' : 1 , 'yco' :2, 'size' :3}
co3 = {'xco' : 1 , 'yco' :2, 'size' :3}
co4 = {'xco' : 1 , 'yco' :2, 'size' :3}
co5 = {'xco' : 1 , 'yco' :2, 'size' :3}
co6 = {'xco' : 1 , 'yco' :2, 'size' :3}
co7 = {'xco' : 1 , 'yco' :2, 'size' :3}
co8 = {'xco' : 1 , 'yco' :2, 'size' :3}
co9 = {'xco' : 1 , 'yco' :2, 'size' :3}
co10 = {'xco' : 1 , 'yco' :2, 'size' :3}

# exact location checker
check = 0
accuracy = 10

# robot's coordinates
robot = {'xco' : 1 , 'yco' : 2, 'size' :3}

# oriantation marker coordinates
orient = {'xco' : 1 , 'yco' : 2, 'size' :3}
#oriented = 0
#alligned = 0

# target coordinates
target = {'xco' : 0 , 'yco' : 0, 'size' :20}

# cirlce finding radius maker
radius = {'srad' : 2 , 'rad' : 3 , 'brad' : 4}
small = 0
big = 0



#=============================================================================================
# defining sub-routines
#=============================================================================================

# connecting to BrickPi
def connect(c):
    print "Got connection from",addr

    # gather info from rpi
    c.send('Thank you for connecting. What is your name robot?')    #1  name
    info['name'] = (c.recv(1024))
    c.send('midc?')                                                 #2  center circle size
    info['midc'] = int(c.recv(1024))
    c.send('oc?')                                                   #3  orientation circel size
    info['oc'] = int(c.recv(1024))
    c.send('colour?')                                               #4  colour
    info['colour'] = (c.recv(1024))
    
    #HSV values
    c.send('hmn?')                                                  #5  hmn
    color['hmn'] = int(c.recv(1024))
    c.send('hmx?')                                                  #6  hmx
    color['hmx'] = int(c.recv(1024))
    c.send('smn?')                                                  #7  smn
    color['smn'] = int(c.recv(1024))
    c.send('smx?')                                                  #8  smx
    color['smx'] = int(c.recv(1024))
    c.send('vmn?')                                                  #9  vmn
    color['vmn'] = int(c.recv(1024))
    c.send('vmx?')                                                  #10  vmx
    color['vmx'] = int(c.recv(1024))
    #making sure info and colour are right
    print 'Information revieved from ', info['name'], ':'
    print info['name'], info['colour'], info['midc'], info['oc']
    print color['hmx'], color['hmn'], color['smx'], color['smn'], color['vmx'], color['vmn'], '\n\n'


#remote control
def remote(c):
    print 'wasd controls'
    cap = cv2.VideoCapture(0)
    while(1):
        _, frame = cap.read()
        cv2.imshow('tracking',frame)
        k = cv2.waitKey(5) & 0xFF
        #print 'dope'            #this is a check point        
        if k == ord('w'):
            c.send('w')
        elif k == ord('a'):
            c.send('a')
        elif k == ord('s'):
            c.send('s')
        elif k == ord('d'):
            c.send('d')
        elif k == ord('e'):
            c.send('e')
        elif k == ord('p'):
            c.send('p')
        elif k == ord('i'):
            c.send('i')
        elif k == ord('o'):
            c.send('o')
        elif k == ord('l'):
            c.send('l')
        elif k == ord('j'):
            c.send('j')
        elif k == ord('k'):
            c.send('k')
        elif k == 27:
            break

    cap.release()

    cv2.destroyAllWindows()

#==================================== remote control end

def find_robot2(cap):
    accuracy = 5
    check = 0
    #cap = cv2.VideoCapture(0)
    radius['rad'] = info['midc']
    radius['srad'] = (float(round(info['midc'])) - 5)
    radius['brad'] = (float(round(info['midc'])) + 5)

    
    
    #center of robot
    while(1):

        _, frame = cap.read()

        #converting to HSV
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        hue,sat,val = cv2.split(hsv)

        # Apply thresholding
        hthresh = cv2.inRange(np.array(hue),np.array(color['hmn']),np.array(color['hmx']))
        sthresh = cv2.inRange(np.array(sat),np.array(color['smn']),np.array(color['smx']))
        vthresh = cv2.inRange(np.array(val),np.array(color['vmn']),np.array(color['vmx']))

        # AND h s and v
        tracking = cv2.bitwise_and(hthresh,cv2.bitwise_and(sthresh,vthresh))

        # Some morpholigical filtering
        dilation = cv2.dilate(tracking,kernel,iterations = 1)
        closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
        closing = cv2.GaussianBlur(closing,(5,5),0)

        # Detect circles using HoughCircles
        circles = cv2.HoughCircles(closing,cv2.HOUGH_GRADIENT,2,90,param1=120,param2=50,minRadius=10,maxRadius=0)
        # circles = np.uint16(np.around(circles))

        if circles is not None:        
                for i in circles[0,:]:
                    if int(round(i[2])) < radius['srad'] or int(round(i[2])) > radius['brad']:
                        pass
                    elif int(round(i[2])) > radius['srad']:
                        if check == 5:
                            co6['xco'] = co2['xco']
                            co6['yco'] = co2['yco']
                            co6['size'] = co2['size']
                            co2['xco'] = i[0]
                            co2['yco'] = i[1]
                            co2['size'] = i[2]
                            check += 1
                        elif check == 4:
                            co5['xco'] = co2['xco']
                            co5['yco'] = co2['yco']
                            co5['size'] = co2['size']
                            co2['xco'] = i[0]
                            co2['yco'] = i[1]
                            co2['size'] = i[2]
                            check += 1
                        elif check == 3:
                            co4['xco'] = co2['xco']
                            co4['yco'] = co2['yco']
                            co4['size'] = co2['size']
                            co2['xco'] = i[0]
                            co2['yco'] = i[1]
                            co2['size'] = i[2]
                            check += 1
                        elif check == 2:
                            co3['xco'] = co2['xco']
                            co3['yco'] = co2['yco']
                            co3['size'] = co2['size']
                            co2['xco'] = i[0]
                            co2['yco'] = i[1]
                            co2['size'] = i[2]
                            check += 1
                        elif check == 1:
                            co2['xco'] = i[0]
                            co2['yco'] = i[1]
                            co2['size'] = i[2]
                            check += 1
                        elif check == 0:
                            co1['xco'] = i[0]
                            co1['yco'] = i[1]
                            co1['size'] = i[2]
                            co2['xco'] = i[0]
                            co2['yco'] = i[1]
                            co2['size'] = i[2]
                            check += 1
                        robot['xco'] = int(round(co2['xco']))
                        robot['yco'] = int(round(co2['yco']))
        cv2.circle(frame,(int(round(robot['xco'])),int(round(robot['yco']))),int(round(robot['size'])),(0,0,255),5)
        cv2.circle(frame,(int(round(robot['xco'])),int(round(robot['yco']))),2,(0,0,255),10)           

        if int(round(co2['xco'])) < (int(round(co1['xco'])) - 10):
            check = 0
            #print 'change'
        elif int(round(co2['xco'])) > (int(round(co1['xco'])) + 10):
            check = 0
            #print 'change'
        elif int(round(co2['yco'])) < (int(round(co1['yco'])) - 10):
            check = 0
            #print 'change'
        elif int(round(co2['yco'])) > (int(round(co1['yco'])) + 10):
            check = 0
            #print 'change'

        cv2.imshow('tracking',frame)
        
        # number of checks
        if check == accuracy:
            xav = (co1['xco'] + co2['xco'] + co3['xco'] + co4['xco'])/4
            yav = (co1['yco'] + co2['yco'] + co3['yco'] + co4['yco'])/4
            robot['xco'] = xav
            robot['yco'] = yav
            
            break
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
           break

    
    #cap.release()
    
    check = 0



#==================================== find robot 2 end


#set the target
def set_target(cap):
    print 'click on your desired target please'
    #cap = cv2.VideoCapture(0)
    def draw_circle(event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDOWN:
            cv2.circle(frame,(x,y),25,(0,0,255),3)
            target['xco'] = x
            target['yco'] = y
          
            
    cv2.namedWindow('tracking')
    cv2.setMouseCallback('tracking',draw_circle)
    while (1):
        _, frame = cap.read()
        k = cv2.waitKey(5) & 0xFF
        cv2.imshow('tracking',frame)
        if k == 27:
            break

    
    print 'target: (',target['xco'],',',target['yco'],')'
    #cap.release()

    #cv2.destroyAllWindows()



#==================================== set target end


def find_orient2(cap):
    check = 0
    #cap = cv2.VideoCapture(0)
    accuracy = 4
    radius['rad'] = info['oc']
    radius['srad'] = (float(round(info['oc'])) - 5)
    radius['brad'] = (float(round(info['oc'])) + 5)
    
    # looking for orientation circle
    while(1):

        _, frame = cap.read()

        #converting to HSV
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        hue,sat,val = cv2.split(hsv)

        # Apply thresholding
        hthresh = cv2.inRange(np.array(hue),np.array(color['hmn']),np.array(color['hmx']))
        sthresh = cv2.inRange(np.array(sat),np.array(color['smn']),np.array(color['smx']))
        vthresh = cv2.inRange(np.array(val),np.array(color['vmn']),np.array(color['vmx']))

        # AND h s and v
        tracking = cv2.bitwise_and(hthresh,cv2.bitwise_and(sthresh,vthresh))

        # Some morpholigical filtering
        dilation = cv2.dilate(tracking,kernel,iterations = 1)
        closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
        closing = cv2.GaussianBlur(closing,(5,5),0)

        # Detect circles using HoughCircles
        circles = cv2.HoughCircles(closing,cv2.HOUGH_GRADIENT,2,90,param1=120,param2=50,minRadius=10,maxRadius=0)
        # circles = np.uint16(np.around(circles))

        if circles is not None:        
                for i in circles[0,:]:
                    if int(round(i[2])) < radius['srad'] or int(round(i[2])) > radius['brad']:
                        pass
                    elif int(round(i[2])) > radius['srad']:
                        if check == 3:
                            co4['xco'] = co2['xco']
                            co4['yco'] = co2['yco']
                            co4['size'] = co2['size']
                            co2['xco'] = i[0]
                            co2['yco'] = i[1]
                            co2['size'] = i[2]
                            check += 1
                        elif check == 2:
                            co3['xco'] = co2['xco']
                            co3['yco'] = co2['yco']
                            co3['size'] = co2['size']
                            co2['xco'] = i[0]
                            co2['yco'] = i[1]
                            co2['size'] = i[2]
                            check += 1
                        elif check == 1:
                            co2['xco'] = i[0]
                            co2['yco'] = i[1]
                            co2['size'] = i[2]
                            check += 1
                        elif check == 0:
                            co1['xco'] = i[0]
                            co1['yco'] = i[1]
                            co1['size'] = i[2]
                            co2['xco'] = i[0]
                            co2['yco'] = i[1]
                            co2['size'] = i[2]
                            check += 1
        cv2.circle(frame,(int(round(robot['xco'])),int(round(robot['yco']))),int(round(robot['size'])),(0,0,255),5)
        cv2.circle(frame,(int(round(co2['xco'])),int(round(co2['yco']))),int(round(orient['size'])),(0,0,255),5)
        cv2.line(frame,(int(round(robot['xco'])),int(round(robot['yco']))),(int(round(co2['xco'])),int(round(co2['yco']))),(255,0,0),5)
                   
        #def change():
        if int(round(co2['xco'])) < (int(round(co1['xco'])) - 10):
            check = 0
            #print 'change'
        elif int(round(co2['xco'])) > (int(round(co1['xco'])) + 10):
            check = 0
            #print 'change'
        elif int(round(co2['yco'])) < (int(round(co1['yco'])) - 10):
            check = 0
            #print 'change'
        elif int(round(co2['yco'])) > (int(round(co1['yco'])) + 10):
            check = 0
            #print 'change'

        cv2.imshow('tracking',frame)
        
        # number of checks
        if check >= accuracy:
            xav = (co1['xco'] + co2['xco'] + co3['xco'] + co4['xco'])/4
            yav = (co1['yco'] + co2['yco'] + co3['yco'] + co4['yco'])/4
            orient['xco'] = xav
            orient['yco'] = yav
            
            break
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
           break

    check = 0

    
#==================================== find orientation 2 end
    

def travel_to_target(cap, c):
        oriented = 0
        alligned = 0
        find_robot2(cap)

        if int(round(target['xco'])) == 0:
            print 'press esc once target is selected'
            set_target(cap)
    
        print 'orientating left or right'

        print 'target: (',target['xco'],',',target['yco'],')'
        print 'target: (',robot['xco'],',',robot['yco'],')'        
        #orientate left or right                                                                        left right?

        #if target is right of robot do this                                                            if right do this
        if int(round(target['xco'])) > int(round(robot['xco'])):
            print 'target is right of the robot'
            #if orient is above robot go clockwise                                                      orientate
            find_orient2(cap)
            
            if int(round(orient['yco'])) < int(round(robot['yco'])):
                print 'rotating clockwise'
                
                while True:
                    c.send('d')
                    find_orient2(cap)                  
                    if int(round(orient['yco'])) >= int(round(robot['yco'])):
                        c.send('e')
                        print 'orientated'
                        oriented = int(1)
                        break

                    

            #if orient is below robot go anticlockwise 
            elif int(round(orient['yco'])) > int(round(robot['yco'])) and oriented != 1:
                print 'rotating anticlockwise'
                while True:
                    c.send('a')
                    find_orient2(cap)
                    
                    if int(round(orient['yco'])) <= int(round(robot['yco'])):
                        c.send('e')
                        print 'orientated'
                        oriented = int(1)
                        break


        #if target is left of robot do this                                                             if left do this
        elif int(round(target['xco'])) < int(round(robot['xco'])) and oriented != 1:
            print 'target is left of the robot'
            #if orient is below robot go clockwise                                                     orientate
            find_orient2(cap)
            
            if int(round(orient['yco'])) > int(round(robot['yco'])):
                print 'rotating clockwise'
                                
                while True:
                    c.send('d')
                    find_orient2(cap)
                    if int(round(orient['yco'])) <= int(round(robot['yco'])):
                        c.send('e')
                        print 'orientated'
                        oriented = int(1)
                        break

                

            #if orient is above robot go anticlockwise
            elif int(round(orient['yco'])) < int(round(robot['yco'])) and oriented != 1:
                print 'rotating anticlockwise'
                
                while True:
                    c.send('a')
                    find_orient2(cap)
                    if int(round(orient['yco'])) >= int(round(robot['yco'])):
                        c.send('e')
                        print 'orientated'
                        oriented = int(1)
                        break

            oriented = int(0)

        elif int(round(target['xco'])) == int(round(robot['xco'])):
            alligned = 1


        #move left or right                                                                             move left or right?

        #if target is right of robot do this                                                            if right move right
        if int(round(target['xco'])) > int(round(robot['xco'])) and alligned != 1:
            print 'go forward'
            while True:
                c.send('w')
                find_robot2(cap)
                if int(round(target['xco'])) <= int(round(robot['xco'])):
                    c.send('e')
                    print 'vertically alligned'
                    alligned = int(1)
                    break

        
            
        #if target is left of robot do this                                                             if left move left
        elif int(round(target['xco'])) < int(round(robot['xco'])) and alligned != 1:
            print 'go forward'
            while True:
                c.send('w')
                find_robot2(cap)
                if int(round(target['xco'])) >= int(round(robot['xco'])):
                    c.send('e')
                    print 'vertically alligned'
                    alligned = int(1)
                    break                







        alligned = 0
               
        #orientate up or down                                                                           up or down?
                    
        #if target is up of robot do this                                                               if up do this
        if int(round(target['yco'])) < int(round(robot['yco'])):
            print 'target is up of the robot'
            find_orient2(cap)
            
            #if orient is left of robot go clockwise                                                    orientate
            if int(round(orient['xco'])) < int(round(robot['xco'])):
                print 'rotating clockwise'
                
                while True:
                    c.send('d')
                    find_orient2(cap)
                    if int(round(orient['xco'])) >= int(round(robot['xco'])):
                        c.send('e')
                        print 'orientated'
                        oriented = int(1)
                        break

                

            #if orient is right robot go anticlockwise
            elif int(round(orient['xco'])) > int(round(robot['xco'])) and oriented != 1:
                print 'rotating anticlockwise'                
                while True:
                    c.send('a')
                    find_orient(cap)
                    if int(round(orient['xco'])) <= int(round(robot['xco'])):
                        c.send('e')
                        print 'orientated'
                        oriented = int(1)
                        break

                        

        #if target is below robot do this                                                             if down do this
        elif int(round(target['yco'])) > int(round(robot['yco'])) and oriented != 1:
            print 'target is below the robot'
            find_orient2(cap)
            #if orient is right of robot go clockwise                                                      orientate
            if int(round(orient['xco'])) > int(round(robot['xco'])):
                print 'rotating clockwise'                
                while True:
                    c.send('d')
                    find_orient2(cap)
                    if int(round(orient['xco'])) <= int(round(robot['xco'])):
                        c.send('e')
                        print 'orientated'
                        oriented = int(1)
                        break

                

            #if orient is left of robot go anticlockwise
            elif int(round(orient['xco'])) > int(round(robot['xco'])) and oriented != 1:
                print 'rotating anticlockwise'
                c.send('a')
                while True:
                    c.send('a')
                    find_orient2(cap)
                    if int(round(orient['xco'])) >= int(round(robot['xco'])):
                        c.send('e')
                        print 'orientated'
                        oriented = int(1)
                        break

            oriented = 0
                



        elif int(round(target['yco'])) == int(round(robot['yco'])):
            alligned = 1


        #move up or down                                                                             move up or down?

        #if target is above robot do this                                                               if up move up
        if int(round(target['yco'])) < int(round(robot['yco'])) and alligned != 1:
            print 'go forward'            
            while True:
                c.send('w')
                find_robot2(cap)
                if int(round(target['yco'])) >= int(round(robot['yco'])):
                    c.send('e')
                    print 'horizontally alligned'
                    alligned = int(1)
                    break

            
        #if target is left of robot do this                                                             if down move down
        elif int(round(target['yco'])) > int(round(robot['yco'])) and alligned != 1:
            print 'go forward'            
            while True:
                c.send('w')
                find_robot2(cap)
                if int(round(target['yco'])) >= int(round(robot['yco'])):
                    c.send('e')
                    print 'horizontally alligned'
                    alligned = int(1)
                    break


        alligned = 0

        print '\n\nyou have arrived at your destination!'


#==================================== travel to target end


def pick_up(c):
    c.send('pick up')
    print 'picked up brick'

def drop(c):
    c.send('drop')
    print 'dropped brick'

def track_robot(cap):
    while True:
        print 'center'
        find_robot2(cap)
        print 'orientator'
        find_orient2(cap)            
        print 'robot (x,y):  (',robot['xco'], ',', robot['yco'], ')'
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

def track_orientation(cap):
    print 'finding center'
    find_robot2(cap)
    print 'robot (x,y):  ',robot['xco'], ',', robot['yco'], ')'
    print 'showing orientation'
    while True:
        find_orient2(cap)            
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    
    

#=============================================================================================
# main body of guidance script
#=============================================================================================

# main menu

while True:
    print '\n Options: connect, remote, find robot, quit, target, find orientation, travel to target, track robot, track orientation'
    inp = str(raw_input())
    if inp == 'connect':
        print 'connecting....'
        s.listen(5)
        c, addr = s.accept()
        connect(c,)
        robot['size'] = int(round(info['midc']))
        orient['size'] = int(round(info['oc']))
            
    elif inp == 'remote':
        remote(c,)

    elif inp == 'find robot':            
        find_robot2(cap)
        print 'robot (x,y):  ',robot['xco'], ',', robot['yco'], ')'
            
            
    elif inp == 'target':
        set_target(cap)

    elif inp == 'find orientation':
        find_orient2()

    elif inp == 'travel to target':
        travel_to_target(cap, c,)

    elif inp == 'track robot':
        track_robot(cap)

    elif inp == 'track orientation':
        track_orientation(cap)                  

    elif inp == 'pick up':
        pick_up(c,)

    elif inp == 'drop':
        drop(c,)
                    
    elif inp == 'quit':
        break


    else:
        print 'commmand not recognized'
        



print 'all done for now'

c.send('q')

c.close()

cap.release()

cv2.destroyAllWindows()

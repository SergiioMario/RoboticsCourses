#!/usr/bin/env python
import serial, time, sys, math
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from hardware_tools import roboclaw  
import tf

def printHelp():
    print "MOBILE BASE for JustinaWinter BY MARCOSOFT. Options:"
    print "\t --port \t Serial port name. If not provided, the default value is \"/dev/ttyACM0\""
    print "\t --simul\t Simulation mode."
    print " - Mobile base can be moved by publishing either mobile_base/cmd_vel or"
    print " - mobile_base/speeds. Speeds must be values in [-1, 1] where a value of 1 "
    print " - represents the maximum speed that each motor can generate."
    print "PLEASE DON'T TRY TO OPERATE JUSTINA IF YOU ARE NOT QUALIFIED ENOUGH."

def callbackStop(msg):
    leftSpeed = 0
    rightSpeed = 0
    newSpeedData = True

def callbackSpeeds(msg):
    global leftSpeed
    global rightSpeed
    global newSpeedData
    #Speeds are assumed to come in float in [-1,1] for each tire. The values need to be transformed to values in [0,127]
    #A float value of -1, indicates the maximum speed backwards
    #Similar for +1
    leftSpeed = msg.data[0]
    rightSpeed = msg.data[1]
    if leftSpeed > 1:
        leftSpeed = 1
    elif leftSpeed < -1:
        leftSpeed = -1
    if rightSpeed > 1:
        rightSpeed = 1
    elif rightSpeed < -1:
        rightSpeed = -1
    newSpeedData = True

def callbackCmdVel(msg):
    global leftSpeed
    global righSpeed
    global newSpeedData
    leftSpeed = msg.linear.x - msg.angular.z*0.44/2.0
    rightSpeed = msg.linear.x + msg.angular.z*0.44/2.0
    if leftSpeed > 1:
        leftSpeed = 1
    elif leftSpeed < -1:
        leftSpeed = -1
    if rightSpeed > 1:
        rightSpeed = 1
    elif rightSpeed < -1:
        rightSpeed = -1
    newSpeedData = True

def calculateOdometry(currentPos, leftEnc, rightEnc): #Encoder measurements are assumed to be in ticks
    leftEnc = leftEnc * 0.58/16000 #From ticks to meters
    rightEnc = rightEnc * 0.58/16000
    deltaTheta = (rightEnc - leftEnc)/0.44 #0.44 is the robot diameter
    if math.fabs(deltaTheta) >= 0.0001:
        rg = (leftEnc + rightEnc)/(2*deltaTheta)
        deltaX = rg*math.sin(deltaTheta)
        deltaY = rg*(1-math.cos(deltaTheta))
    else:
        deltaX = (leftEnc + rightEnc)/2
        deltaY = 0
    currentPos[0] += deltaX * math.cos(currentPos[2]) - deltaY * math.sin(currentPos[2])
    currentPos[1] += deltaX * math.sin(currentPos[2]) + deltaY * math.cos(currentPos[2])
    currentPos[2] += deltaTheta
    return currentPos
    

def main(portName, simulated):
    print "INITIALIZING MOBILE BASE BY MARCOSOFT..."
    ###Connection with ROS
    rospy.init_node("mobile_base")
    pubOdometry = rospy.Publisher("mobile_base/odometry", Odometry, queue_size = 1)
    pubBattery = rospy.Publisher("robot_state/base_battery", Float32, queue_size = 1)
    subStop = rospy.Subscriber("robot_state/stop", Empty, callbackStop)
    subSpeeds = rospy.Subscriber("mobile_base/speeds", Float32MultiArray, callbackSpeeds)
    #subCmdVel = rospy.Subscriber("mobile_base/cmd_vel", Twist, callbackCmdVel)

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20)
    Roboclaw = roboclaw.Roboclaw(portName, 115200) #nombre del puerto, baudrate del puerto
    ###Communication with the Roboclaw
    if not simulated:
        print "MobileBase.-> Trying to open serial port on \"" + portName + "\""
        Roboclaw.Open()
        address = 0x80
 
        print "MobileBase.-> Serial port openned on \"" + portName + "\" at 38400 bps (Y)"
        print "MobileBase.-> Clearing previous encoders readings"

        Roboclaw.ResetEncoders(address) #reseteando encoders
   
    ###Variables for setting tire speeds
    global leftSpeed
    global rightSpeed
    global newSpeedData
    leftSpeed = 0
    rightSpeed = 0
    newSpeedData = False
    speedCounter = 5
    ###Variables for odometry
    robotPos = [0, 0, 0]
    while not rospy.is_shutdown():
        if newSpeedData:
            newSpeedData = False
            speedCounter = 5
            if not simulated:
                leftSpeed = int(leftSpeed*127)
                rightSpeed = int(rightSpeed*127)
                try:
                    if leftSpeed >= 0:
                        Roboclaw.ForwardM2(address, leftSpeed)
                    else:
                        Roboclaw.BackwardM2(address, -leftSpeed)
                    if rightSpeed >= 0:
                        Roboclaw.ForwardM1(address, rightSpeed)
                    else:
                        Roboclaw.BackwardM1(address, -rightSpeed)
                except:
                    print "MOBILE BASE.->Error while sending speeds to Roboclaw"
        else:
            speedCounter -= 1
            if speedCounter == 0:
                if not simulated:
                    Roboclaw.ForwardM1(address, 0)
                    Roboclaw.ForwardM2(address, 0)
                else:
                    leftSpeed = 0
                    rightSpeed = 0
            if speedCounter < -1:
                speedCounter = -1
        if not simulated:
            try:
                c1, encoderLeft, c2 =  Roboclaw.ReadEncM2(address)
                d1, encoderRight, d2 = Roboclaw.ReadEncM1(address)
                Roboclaw.ResetEncoders(address)
            except:
                print "MOBILE BASE.->Error while reading encoders"
        else:
            encoderLeft = leftSpeed * 0.1 * 16000 / 0.58
            encoderRight = rightSpeed * 0.1 * 16000 / 0.58        ###Odometry calculation
        robotPos = calculateOdometry(robotPos, encoderLeft, encoderRight)
        #print "Encoders: " + str(encoderLeft) + "  " + str(encoderRight)
        ##Odometry and transformations
        ts = TransformStamped()
        ts.header.stamp = rospy.Time.now()
        ts.header.frame_id = "odom"
        ts.child_frame_id = "base_link"
        ts.transform.translation.x = robotPos[0]
        ts.transform.translation.y = robotPos[1]
        ts.transform.translation.z = 0
        ts.transform.rotation = tf.transformations.quaternion_from_euler(0, 0, robotPos[2])
        br.sendTransform((robotPos[0], robotPos[1], 0), ts.transform.rotation, rospy.Time.now(), ts.child_frame_id, ts.header.frame_id)
        msgOdom = Odometry()
        msgOdom.header.stamp = rospy.Time.now()
        msgOdom.pose.pose.position.x = robotPos[0]
        msgOdom.pose.pose.position.y = robotPos[1]
        msgOdom.pose.pose.position.z = 0
        msgOdom.pose.pose.orientation.x = 0
        msgOdom.pose.pose.orientation.y = 0
        msgOdom.pose.pose.orientation.z = math.sin(robotPos[2]/2)
        msgOdom.pose.pose.orientation.w = math.cos(robotPos[2]/2)
        pubOdometry.publish(msgOdom)
        ###Reads battery and publishes the corresponding topic
        motorBattery = 18.5
        if not simulated:
           motorBattery = Roboclaw.ReadMainBatteryVoltage(address) #Lectura de voltaje de bateria
        #msgBattery = Float32()
        #msgBattery.data = motorBattery
        #pubBattery.publish(msgBattery)
        rate.sleep()
    #End of while
    if not simulated:
        Roboclaw.ForwardM1(address, 0)
        Roboclaw.ForwardM2(address, 0)
       
#end of main()

if __name__ == '__main__':
    try:
        if "--help" in sys.argv:
            printHelp()
        elif "-h" in sys.argv:
            printHelp()
        else:
            portName = "/dev/ttyACM0"
            simulated = False
            if "--port" in sys.argv:
                portName = sys.argv[sys.argv.index("--port") + 1]
            if "--simul" in sys.argv:
                simulated = True
            main(portName, simulated)
    except rospy.ROSInterruptException:
        pass

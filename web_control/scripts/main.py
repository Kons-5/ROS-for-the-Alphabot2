#!/usr/bin/python3
# -*- coding:utf-8 -*-
import rospy
from geometry_msgs.msg import TwistStamped
from web_control.msg import Float64Stamped
from bottle import get,post,run,route,request,template,static_file
from AlphaBot import AlphaBot
from PCA9685 import PCA9685
import RPi.GPIO as GPIO
import threading
import socket #ip
import os
import math
import signal
import sys

# ROS initialization
rospy.init_node('robot_controller', anonymous=True)
rate = rospy.Rate(30)
cmd_vel_pub = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=30)
pan_pub = rospy.Publisher('/pan_angle', Float64Stamped, queue_size=30)
tilt_pub = rospy.Publisher('/tilt_angle', Float64Stamped, queue_size=30)

diameter = rospy.get_param('~base_link_diameter', 0.1)  # Default: 0.1 m
max_speed = rospy.get_param('~max_speed', 1.5)          # Default: 1.5 m/s

# Initialize the AlphaBot and PWM
Ab = AlphaBot()
pwm = PCA9685(0x40)
pwm.setPWMFreq(50)

#Set servo parameters
HPulse = 1500                       # Sets the initial Pulse
HStep = 0                           # Sets the initial step length
VPulse = 1500                       # Sets the initial Pulse
VStep = 0                           # Sets the initial step length
pwm.setServoPulse(1, VPulse)
pwm.setServoPulse(0, HPulse)
last_vel = 50                       # Default speed at 50% motor power
linear_x = 0.0                      # Will be overwritten
angular_z = 0.0                     # Will be overwritten

def publish_cmd_vel(event):
    twist = TwistStamped()
    twist.twist.linear.x = linear_x * max_speed
    twist.twist.angular.z = angular_z * max_speed
    twist.header.stamp = rospy.Time.now()
    twist.header.frame_id = "base_link"
    cmd_vel_pub.publish(twist)

def pulse_to_radians(pulse):
    # Convert pulse width (500-2500) to radians (-pi/2 to pi/2)
    return (pulse - 1500) / 1000.0 * math.pi / 2.0

def publish_pan_tilt(event):
    pan_msg = Float64Stamped()
    pan_msg.data = pulse_to_radians(HPulse)
    pan_msg.header.stamp = rospy.Time.now()
    pan_msg.header.frame_id = "base_link"
    tilt_msg = Float64Stamped()
    tilt_msg.data = pulse_to_radians(VPulse)
    tilt_msg.header.stamp = rospy.Time.now()
    tilt_msg.header.frame_id = "base_link"
    pan_pub.publish(pan_msg)
    tilt_pub.publish(tilt_msg)

# Determine the script's directory
script_dir = os.path.dirname(os.path.realpath(__file__))

@get("/")
def index():
    return static_file('index.html', root=script_dir)

@route('/<filename>')
def server_static(filename):
    return static_file(filename, root=script_dir)

@route('/fonts/<filename>')
def server_fonts(filename):
    return static_file(filename, root=os.path.join(script_dir, 'fonts'))
	
@post("/cmd")
def cmd():
    global HStep,VStep,last_vel,linear_x,angular_z
    code = request.body.read().decode()
    speed = request.POST.get('speed')
    print(code)
    if(speed != None):
        Ab.setPWMA(float(speed))
        Ab.setPWMB(float(speed))
        last_vel = float(speed)
    if code == "stop":
        linear_x = 0.0
        angular_z = 0.0
        HStep = 0
        VStep = 0
        Ab.stop()
        print("stop")
    elif code == "forward":
        Ab.forward()
        linear_x = last_vel / 100.0
        angular_z = 0.0
        print("forward")
    elif code == "backward":
        Ab.backward()
        linear_x = -last_vel / 100.0
        angular_z = 0.0
        print("backward")
    elif code == "turnleft":
        Ab.left()
        linear_x = 0.0
        angular_z = (10.0 / 100.0) / (diameter/2)
        print("turnleft")
    elif code == "turnright":
        Ab.right()
        linear_x = 0.0
        angular_z = -(10.0 / 100.0) / (diameter/2)
        print("turnright")
    elif code == "up":
        VStep = -5
        print("up")
    elif code == "down":
        VStep = 5
        print("down")
    elif code == "left":
        HStep = 5
        print("left")
    elif code == "right":
        HStep = -5
        print("right")
    return "OK"
	
def timerfunc():
	global HPulse,VPulse,HStep,VStep,pwm
	
	if(HStep != 0):
		HPulse += HStep
		if(HPulse >= 2500): 
			HPulse = 2500
		if(HPulse <= 500):
			HPulse = 500
		#set channel 2, the Horizontal servo
		pwm.setServoPulse(0, HPulse)    
		
	if(VStep != 0):
		VPulse += VStep
		if(VPulse >= 2500): 
			VPulse = 2500
		if(VPulse <= 500):
			VPulse = 500
		#set channel 3, the vertical servo
		pwm.setServoPulse(1, VPulse)   
	
	global t        #Notice: use global variable!
	t = threading.Timer(0.01, timerfunc)
	t.start()

def clean_shutdown(signum, frame):
    global t
    print("Shutting down...")
    Ab.stop()
    t.cancel()
    GPIO.cleanup()  # Clean up GPIO pins
    rospy.signal_shutdown("User requested shutdown")
    sys.exit(0)

def get_ip():
    methods = [
        ('10.42.0.1', 80),  # Intranet IP
        ('8.8.8.8', 80)     # Public IP (Google DNS)
    ]
    
    for ip, port in methods:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect((ip, port))
            local_ip = s.getsockname()[0]
            s.close()
            return local_ip
        except socket.error:
            continue
    
    # If all methods fail, return localhost
    return '127.0.0.1'

if __name__ == "__main__":
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, clean_shutdown)
    signal.signal(signal.SIGTERM, clean_shutdown)

    # Create ROS timers for publishing cmd_vel and pan_tilt at 10Hz
    rospy.Timer(rospy.Duration(0.1), publish_cmd_vel)
    rospy.Timer(rospy.Duration(0.1), publish_pan_tilt)

    # Threads
    t = threading.Timer(0.02, timerfunc)
    t.setDaemon(True)
    t.start()

    # Run server
    localhost = get_ip()
    run(host=localhost, port=8000)

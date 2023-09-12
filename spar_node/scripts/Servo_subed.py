#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool



EPIPEN_onboard = True
GPS_onboard = True

# Start PWM running on both servos, value of 0 (pulse off)
GPS_servo.start(0)
PEN_servo.start(0)

def engage():
    GPS_servo.ChangeDutyCycle(2)
    PEN_servo.ChangeDutyCycle(2)

def release(payload):
    if payload == 'GPS':
        GPS_servo.ChangeDutyCycle(12)
        GPS_onbord = False
        
    if payload == 'PEN':
        PEN_servo.ChangeDutyCycle(12)
        EPIPEN_onboard = False
        



        
        
def deploy(msg_in):

    if msg_in.data:
        rospy.loginfo("EpiPen Released!")
        release("PEN")

    else:
        rospy.loginfo("GPS Released!")
        release("GPS")
        


        

def shutdown():
	# Clean up our ROS subscriber if they were set, avoids error messages in logs
	if sub is not None:
		sub_a.unregister()

	# XXX: Could perform some failsafe actions here!

	# Close down our GPIO
	GPIO.cleanup()

if __name__ == '__main__':
	# Setup the ROS backend for this node
	rospy.init_node('actuator_controller', anonymous=True)

	# Setup the GPIO
        GPIO.setup(11,GPIO.OUT)
        GPS_servo = GPIO.PWM(11,50) # pin 11 for GPS servo
        GPIO.setup(12,GPIO.OUT)
        PEN_servo = GPIO.PWM(12,50) # pin 12 for EPIPEN servo 


	# Setup the publisher for a single actuator (use additional subscribers for extra actuators)
	sub_a = rospy.Subscriber('/actuator_control/actuator_a', Bool, deploy)

	# Make sure we clean up all our code before exiting
	rospy.on_shutdown(shutdown)

	# Loop forever
	rospy.spin()

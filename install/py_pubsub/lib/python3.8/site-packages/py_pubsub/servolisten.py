#Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# ITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or 

import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String






class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')

        self.ultrapub = self.create_publisher(String,'ultrasonic',500)
        
        self.cmdsub = self.create_subscription(
                String,
                'cmdpi',
                self.cmd_callback,
                5)
        	
    def cmd_callback(self, msg):
        self.newcmd = msg

    def refresh(self):
        rclpy.spin_once(self)

    def pub_ultra(self, cmdstr):
        msg = String()
        msg.data = cmdstr
        self.ultrapub.publish(msg)
        print("ultra sent")

    def get_msg(self):
         rclpy.spin_once(self)
         return self.newcmd
    
    def set_msg(self):
         self.newcmd = ""
            

    


def get_distance(auto_nav,TRIG_PIN,ECHO_PIN):
    try:
        GPIO.output(TRIG_PIN, True)
        time.sleep(0.001)
        GPIO.output(TRIG_PIN, False)

        start = time.time()
        end = time.time()

        while GPIO.input(ECHO_PIN)==0:
            pulse_start = time.time()
            if (pulse_start-start)>1:
                break
        
        end = time.time()
        while GPIO.input(ECHO_PIN)==1:
            pulse_end = time.time()
            if (pulse_end-end)>1:
                break

        pulse_duration = pulse_end - pulse_start

        distance = pulse_duration * 17150 #speed of sound is 34300 cm/s
        distance = round(distance, 2) #some correctionfactor, eneds to b etested
        return distance

    except:
        return get_distance(auto_nav,TRIG_PIN,ECHO_PIN)


def wait_for_can_mode(auto_nav,TRIG_PIN,ECHO_PIN):
    LOWER_DIST = 11 #cm
    UPPER_DIST = 13 #cm
    while(True):
        auto_nav.pub_ultra("still waiting")
        distance = get_distance(auto_nav,TRIG_PIN,ECHO_PIN)
        print(distance)
        if (distance < LOWER_DIST) or (distance > UPPER_DIST):
            auto_nav.pub_ultra("can in")
            #can is present 
            #return 1 #returns 1 if can is present
        else:
            auto_nav.pub_ultra("can out")
        time.sleep(0.5)            #return 0
    #return 0 #returns zero if no can

#send V to led
def turn_on_led(led_pin):
    GPIO.output(led_pin, GPIO.HIGH)


def turn_off_led(led_pin):
    GPIO.output(led_pin, GPIO.LOW)

 
def follow_line(auto_nav,led_pin):

    ldr_pin1 = 26
    ldr_pin2 = 22
    turn_on_led(led_pin)
    #stop before turning
    time.sleep(0.1)
    #turn 180 degrees such that the bot is facing away from the dispenser
    auto_nav.rotate180()
    
    while True:
        
        left_ldr = GPIO.input(ldr_pin1)
        right_ldr = GPIO.input(ldr_pin2)
        # on the line, continue straight 
        if left_ldr == 1 and right_ldr == 1:
            auto_nav.pub_ultra("back")
            
        # motor is deviating right, move left
        elif left_ldr == 1 and right_ldr == 0:
            auto_nav.pub_ultra("left")
            
        #motor is deviating left, move right
        elif left_ldr == 0 and right_ldr == 1:
            auto_nav.pub_ultra("right")


        elif left_ldr == 0 and right_ldr ==0 :
            auto_nav.pub_ultra("done")
            break
        
        else:
            pass

    turn_off_led(led_pin)
        
def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()

         
    #ignore warnings if other functions is running
    GPIO.setwarnings(False)

    led_pin = 34

    ldr_pin1 = 26
    ldr_pin2 = 22



    GPIO.setmode(GPIO.BCM)
    left_ldr = GPIO.setup(ldr_pin1, GPIO.IN) #to read sensors
    right_ldr = GPIO.setup(ldr_pin2, GPIO.IN)



    TRIG_PIN = 24   #24 for general, 24 for gpio.board
    ECHO_PIN = 8   #8 for general, 8 for gpio.board

    GPIO.setup(TRIG_PIN,GPIO.OUT)
    GPIO.setup(ECHO_PIN,GPIO.IN)
    GPIO.output(TRIG_PIN, False)
    time.sleep(1) #1 second delay

    auto_nav.set_msg()

    #while(True):
    #    auto_nav.pub_ultra("waiting")
    #    print("sent waiting message")
    wait_for_can_mode(auto_nav,TRIG_PIN,ECHO_PIN)
    #    auto_nav.pub_ultra("done")
    #while(True):
    #    cmd = auto_nav.get_msg()
    #    if cmd == "park":
    #        auto_nav.pub_ultra(str(left_ldr)+str(right_ldr))
    #        follow_line(auto_nav,led_pin)
    #    elif cmd == "wait":
    #        wait_for_can_mode(auto_nav,TRIG_PIN,ECHO_PIN)
    #        auto_nav.pub_ultra("done")


if __name__ == '__main__':
    main()

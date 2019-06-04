#!/usr/bin/env python
"""
   pid_velocity - takes messages on wheel_vtarget 
      target velocities for the wheels and monitors wheel for feedback
      
    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import rospy
import roslib

from std_msgs.msg import Int32 as Int16
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from numpy import array

    
######################################################
######################################################
class PidVelocity():
######################################################
######################################################


    #####################################################
    def __init__(self):
    #####################################################
        rospy.init_node("pid_velocity")
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)
        
        ### initialize variables
        self.target = 0
        self.motor = 0
        self.vel = 0
        self.integral = 0
        self.error = 0
        self.derivative = 0
        self.previous_error = 0
        self.wheel_prev = 0
        self.wheel_latest = 0
        self.then = rospy.Time.now()
        self.wheel_mult = 0
        self.prev_encoder = 0
        
        ### get parameters #### 
        self.Kp = rospy.get_param('~Kp',10)
        self.Ki = rospy.get_param('~Ki',10)
        self.Kd = rospy.get_param('~Kd',0.001)
        self.out_min = rospy.get_param('~out_min',-255)
        self.out_max = rospy.get_param('~out_max',255)
        self.rate = rospy.get_param('~rate',30)
        self.max_change = rospy.get_param('~max_change',0.666666)
        self.timeout_ticks = rospy.get_param('~timeout_ticks',4)
        self.ticks_per_meter = rospy.get_param('ticks_meter', 20)
        self.vel_threshold = rospy.get_param('~vel_threshold', 0.001)
        self.encoder_min = rospy.get_param('encoder_min', -32768)
        self.encoder_max = rospy.get_param('encoder_max', 32768)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
        self.wheel_latest = 0.0
        self.prev_pid_time = rospy.Time.now()
        rospy.logdebug("%s got Kp:%0.3f Ki:%0.3f Kd:%0.3f tpm:%0.3f" % (self.nodename, self.Kp, self.Ki, self.Kd, self.ticks_per_meter))
        
        #### subscribers/publishers 
        rospy.Subscriber("wheel", Int16, self.wheelCallback) 
        rospy.Subscriber("wheel_vtarget", Float32, self.targetCallback) 
        self.pub_motor = rospy.Publisher('motor_cmd',Int8, queue_size=10) 
        self.pub_vel = rospy.Publisher('wheel_vel', Float32, queue_size=10)
   
        
    #####################################################
    def spin(self):
    #####################################################
        self.r = rospy.Rate(self.rate) 
        self.then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
        self.wheel_prev = self.wheel_latest
        self.then = rospy.Time.now()
        while not rospy.is_shutdown():
            self.spinOnce()
            self.r.sleep()
            
    #####################################################
    def spinOnce(self):
    #####################################################
        self.previous_error = 0.0
        self.integral = 0.0
        self.error = 0.0
        self.derivative = 0.0 
        self.vel = 0.0
 	self.motor = 0       
        # only do the loop if we've recently recieved a target velocity message
        while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
            self.calcVelocity()
            self.doPid()
            self.pub_motor.publish(self.motor)
            self.r.sleep()
            self.ticks_since_target += 1
            if self.ticks_since_target == self.timeout_ticks:
                self.pub_motor.publish(0)
		rospy.loginfo("timeout")
            
    #####################################################
    def calcVelocity(self):
    #####################################################
        self.dt_duration = rospy.Time.now() - self.then
        self.dt = self.dt_duration.to_sec()
        rospy.logdebug("-D- %s caclVelocity dt=%0.3f wheel_latest=%0.3f wheel_prev=%0.3f" % (self.nodename, self.dt, self.wheel_latest, self.wheel_prev))
    
        # we received a new wheel value
	if(self.dt != 0):
	        cur_vel = (self.wheel_latest - self.wheel_prev) / self.dt
		self.updateVel(cur_vel)
	        rospy.logdebug("-D- %s **** wheel updated vel=%0.3f **** " % (self.nodename, self.vel))
	        self.wheel_prev = self.wheel_latest
	        self.then = rospy.Time.now()
		self.pub_vel.publish(self.vel)
        else:
		rospy.loginfo("dt was zero")
    #####################################################
    def updateVel(self, val):
    #####################################################
        self.vel = self.vel * 0.8 + 0.2 * val
    def updateDerv(self,val):
        self.derivative = val * 0.8 + self.derivative * 0.2
    #####################################################
    def doPid(self):
    #####################################################
        pid_dt = self.dt
	if(self.dt!=0):
        	self.prev_pid_time = rospy.Time.now()
        
        	self.error = self.target - self.vel
        	self.integral = self.integral + (self.error * pid_dt)
        # rospy.loginfo("i = i + (e * dt):  %0.3f = %0.3f + (%0.3f * %0.3f)" % (self.integral, self.integral, self.error, pid_dt))
        	self.updateDerv((self.error - self.previous_error) / pid_dt)
        	self.previous_error = self.error
    		#Make PID loop a passthrough
		#self.motor = self.target 
        	self.motor += max(min((self.Kp * self.error) + (self.Ki * self.integral) + (self.Kd * self.derivative),self.max_change),-self.max_change)
        	self.motor = min(max(self.motor,self.out_min),self.out_max)
      
        #if (self.target == 0):
        #    self.motor = 0
    
        	rospy.logdebug("vel:%0.2f tar:%0.2f err:%0.2f int:%0.2f der:%0.2f ## motor:%d " % 
                      (self.vel, self.target, self.error, self.integral, self.derivative, self.motor))
    	else:
		rospy.loginfo("dt was zero")  	

    #####################################################
    def wheelCallback(self, msg):
    ######################################################
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_encoder > self.encoder_high_wrap) :
            self.wheel_mult = self.wheel_mult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_encoder < self.encoder_low_wrap) :
            self.wheel_mult = self.wheel_mult - 1
           
         
        self.wheel_latest = 1.0 * (enc + self.wheel_mult * (self.encoder_max - self.encoder_min)) / self.ticks_per_meter 
        self.prev_encoder = enc
        
        
#        rospy.logdebug("-D- %s wheelCallback msg.data= %0.3f wheel_latest = %0.3f mult=%0.3f" % (self.nodename, enc, self.wheel_latest, self.wheel_mult))
    
    ######################################################
    def targetCallback(self, msg):
    ######################################################
        self.target = msg.data
        self.ticks_since_target = 0
        # rospy.logdebug("-D- %s targetCallback " % (self.nodename))
    
    
if __name__ == '__main__':
    """ main """
    try:
        pidVelocity = PidVelocity()
        pidVelocity.spin()
    except rospy.ROSInterruptException:
        pass

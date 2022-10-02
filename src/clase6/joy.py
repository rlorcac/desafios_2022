#!/usr/bin/env python

import rospy #importar ros para python
from sensor_msgs.msg import Joy  # joystick
from duckietown_msgs.msg import Twist2DStamped #ruedas
from std_msgs.msg import Bool
import numpy as np

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.subJoy = rospy.Subscriber("/duckiebot/joy", Joy, self.callback)
		self.subFreno = rospy.Subscriber("duckiebot/c6/freno", Bool, self.frenar)
		self.pub = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size=10)
		self.freno = False
                self.vel = 0
                self.turn = 0

	def publicar(self, B, controls):
		self.msg = Twist2DStamped()
                self.vel = -controls[1]
		self.turn = controls[2]
		if self.freno or B:
			self.vel = max(0, vel)
			self.turn = 0	
		msg.v = self.vel # rango maximo [-1,1]
		msg.omega = np.interp(self.turn, [-1,1], [-15, 15]) # rango maximo [-20,20]
		self.pub.publish(msg)
		self.freno = False

	def callback(self,msg):
		#print(msg.axes)
		#print(msg.buttons)
		self.B = self.msg.buttons[1]
		self.right_hor, self.right_ver = self.msg.axes[3:5]
		self.left_hor, self.left_ver = self.msg.axes[0:2]
		self.controls = (self.left_hor, self.left_ver, self.right_hor, self.right_ver)
		self.publicar(self.B, self.controls)
		
	def frenar(self, msg):
	        self.freno = msg.data

def main():
	rospy.init_node('clase6_joy') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()

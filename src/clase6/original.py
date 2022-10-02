#!/usr/bin/env python

import rospy #importar ros para python
from sensor_msgs.msg import Joy  # joystick
from duckietown_msgs.msg import Twist2DStamped #ruedas
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

def _linmap(argrange, tgtrange, arg):
	n = tgtrange[0]
	m = float(tgtrange[1]-tgtrange[0])/float(argrange[1]-argrange[0])
	return n + m * (arg-argrange[0])

def linmap(argrange, tgtrange, iterable):
	ret = [_linmap(argrange, tgtrange, arg) for arg in iterable]
	return ret

def freno(B, controls):
	if B:
		controls = [0 for elem in controls]
	return (B, controls)

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.sub = rospy.Subscriber("/duckiebot/joy", Joy, self.callback)
		self.subPatos = rospy.Subscriber("duckiebot/posicionPato", Point, self.frenar)
		self.pub = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size=10)
		self.freno = False
		self.pubFreno = rospy.Publisher("duckiebot/freno", Bool, queue_size = 10)

	def publicar(self, B, controls):
		msg = Twist2DStamped()
		(B, controls) = freno(B, controls)
		vel = -controls[1]
		turn = controls[2]
		if self.freno:
			vel = max(0, vel)
			turn = 0	
		msg.v = vel # [-1,1]
		msg.omega = _linmap([-1,1], [-15, 15], turn) # [-20,20]
		self.pub.publish(msg)
		self.freno = False

	def callback(self,msg):
		#print(msg.axes)
		#print(msg.buttons)
		B = msg.buttons[1]
		right_hor, right_ver = msg.axes[3:5]
		left_hor, left_ver = msg.axes[0:2]
		controls = (left_hor, left_ver, right_hor, right_ver)
		self.publicar(B, controls)
		
	def frenar(self, msg):
		if msg.x < 25:
			self.freno = True
		self.pubFreno.publish(Bool(self.freno))

def main():
	rospy.init_node('clase6') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()

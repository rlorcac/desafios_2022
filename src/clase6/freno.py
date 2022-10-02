#!/usr/bin/env python

import rospy #importar ros para python
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.sub = rospy.Subscriber("duckiebot/posicionPato", Point, self.callback)
		self.pub = rospy.Publisher("duckiebot/c6/freno", Bool, queue_size = 10)
        
        def callback(self, msg):
                self.frenar = msg.x < 25
                self.pubFreno.publish(Bool(self.frenar))

def main():
	rospy.init_node('clase6_freno') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()

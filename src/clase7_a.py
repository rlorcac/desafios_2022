#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy


class Template(object):
	def __init__(self, path):
		super(Template, self).__init__()
		self.path = "/home/duckiebot/duckietown/catkin_ws/src/desafios_2022/src/cascade3_LBP_.xml"
		self.sub = rospy.Subscriber("/duckiebot/camera_node/image/rect", Image, self.procesar_img)
		self.pub = rospy.Publisher("/duckiebot/camera_node/image/patos_cv", Image, queue_size = 10)

	def publicar(self):
		pass

	def callback(self,msg):
		pass

	def procesar_img(self, img):
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(img, "bgr8")
		# Pasar imagen a escala de grises
		img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		# Definir detecciones
		dets = self.detector.detectMultiScale(img_gray, 1.3, 10)
		# Dibujar rectangulos de cada blob
		for det in dets:
			x,y,w,h = det	
			cv2.rectangle(image, (x,y), (x+w,y+h), (255,255,255), 2)
		# Publicar imagen final
		image_out = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
		msg = bridge.cv2_to_imgmsg(image_out, "bgr8")
		self.pub.publish(msg)

def main():
	rospy.init_node('clase7') #creacion y registro del nodo!
	
	detector = cv2.CascadeClassifier("/home/duckiebot/duckietown/catkin_ws/src/desafios_2022/src/cascade3_LBP_.xml")
	obj = Template('/home/duckiebot/duckietown/catkin_ws/src/desafios_2022/src/cascade3_LBP_.xml') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()

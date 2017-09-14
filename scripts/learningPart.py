#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'
import rospy
import cv2
import tensorflow as tf
import random
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from sherlotics.msg import LEARNINGtoCENTER
from cv_bridge import CvBridge, CvBridgeError

#----------------------------------------------------------

rospy.init_node('learningPart_node')

IMAGE_SIZE = 50
IMAGE_COLOR = 3
TOTAL_IMAGE_SIZE = IMAGE_SIZE * IMAGE_SIZE * IMAGE_COLOR
VARIETY_SIZE = 6
COUNT = 0
#W1 b1 W2 b2 W3 b3 W4 b4 W5 b5

graph1 = tf.Graph()

with graph1.as_default():
    X = tf.placeholder(tf.float32, [None, TOTAL_IMAGE_SIZE]) # 50*50*3

    W1 = tf.get_variable("W1", shape=[TOTAL_IMAGE_SIZE, 512],
                         initializer=tf.contrib.layers.xavier_initializer())
    b1 = tf.Variable(tf.random_normal([512]))
    L1 = tf.nn.relu(tf.matmul(X, W1) + b1)

    W2 = tf.get_variable("W2", shape=[512, 512],
                         initializer=tf.contrib.layers.xavier_initializer())
    b2 = tf.Variable(tf.random_normal([512]))
    L2 = tf.nn.relu(tf.matmul(L1, W2) + b2)

    W3 = tf.get_variable("W3", shape=[512, 512],
                         initializer=tf.contrib.layers.xavier_initializer())
    b3 = tf.Variable(tf.random_normal([512]))
    L3 = tf.nn.relu(tf.matmul(L2, W3) + b3)

    W4 = tf.get_variable("W4", shape=[512, 512],
                         initializer=tf.contrib.layers.xavier_initializer())
    b4 = tf.Variable(tf.random_normal([512]))
    L4 = tf.nn.relu(tf.matmul(L3, W4) + b4)

    W5 = tf.get_variable("W5", shape=[512, VARIETY_SIZE],
                         initializer=tf.contrib.layers.xavier_initializer())
    b5 = tf.Variable(tf.random_normal([VARIETY_SIZE]))
    hypothesis = tf.matmul(L1, W5) + b5

    softmax = tf.nn.softmax(hypothesis)
    prediction = tf.argmax(hypothesis, 1)

#----------------------------------------------------------------

def callback(msg):
  try:
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
  except CvBridgeError as e:
    print(e)

  cv_image = cv_image / 255.0

  with tf.Session(graph=graph1) as sess:
    robotMode = 0
    sess.run(tf.global_variables_initializer())
    saver = tf.train.Saver()
    saver.restore(sess, "/home/sher/catkin_ws/src/sherlotics/data/Turtlebot_Learning_Data.ckpt")

    x_data = sess.run(tf.reshape(cv_image, shape=[1,TOTAL_IMAGE_SIZE]))
    traffic_number = sess.run(prediction, feed_dict={X: x_data})
    traffic_softmax = sess.run(softmax, feed_dict={X: x_data})
    #print traffic_number
    final_number=-1
    if traffic_softmax[0,traffic_number] > 0.99:
        final_number = traffic_number

    if final_number == 0:
        robotMode = 1
    elif final_number == 1:
        robotMode = 2
  try:
    if robotMode != 0:
      pub.publish(robotMode)
  except CvBridgeError as e:
    print(e)

pub = rospy.Publisher('LEARNINGtoCENTER_msg',LEARNINGtoCENTER, queue_size = 1)

bridge = CvBridge()
image_sub = rospy.Subscriber("TRAFFICtoLEARNING_msg",Image, callback, queue_size = 1)

rospy.spin()

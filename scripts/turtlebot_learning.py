#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'
import numpy as np
import cv2
import tensorflow as tf
import random
#---------------Neural Network part---------------------

# parameters
learning_rate = 0.001
IMAGE_SIZE = 50
IMAGE_COLOR = 3
TOTAL_IMAGE_SIZE = IMAGE_SIZE * IMAGE_SIZE * IMAGE_COLOR
DATA_SIZE = 60
VARIETY_SIZE = 6

# input place holders
X = tf.placeholder(tf.float32, [None, TOTAL_IMAGE_SIZE]) # 50*50*3
Y = tf.placeholder(tf.int32, [None, 1])

Y_one_hot = tf.one_hot(Y, VARIETY_SIZE)
Y_one_hot = tf.reshape(Y_one_hot, [-1,VARIETY_SIZE])

# dropout (keep_prob) rate  0.7 on training, but should be 1 for testing
keep_prob = tf.placeholder(tf.float32)

# weights & bias for nn layers
W1 = tf.get_variable("W1", shape=[TOTAL_IMAGE_SIZE, 512],
                     initializer=tf.contrib.layers.xavier_initializer())
b1 = tf.Variable(tf.random_normal([512]))
L1 = tf.nn.relu(tf.matmul(X, W1) + b1)
L1 = tf.nn.dropout(L1, keep_prob=keep_prob)

W2 = tf.get_variable("W2", shape=[512, 512],
                     initializer=tf.contrib.layers.xavier_initializer())
b2 = tf.Variable(tf.random_normal([512]))
L2 = tf.nn.relu(tf.matmul(L1, W2) + b2)
L2 = tf.nn.dropout(L2, keep_prob=keep_prob)

W3 = tf.get_variable("W3", shape=[512, 512],
                     initializer=tf.contrib.layers.xavier_initializer())
b3 = tf.Variable(tf.random_normal([512]))
L3 = tf.nn.relu(tf.matmul(L2, W3) + b3)
L3 = tf.nn.dropout(L3, keep_prob=keep_prob)

W4 = tf.get_variable("W4", shape=[512, 512],
                     initializer=tf.contrib.layers.xavier_initializer())
b4 = tf.Variable(tf.random_normal([512]))
L4 = tf.nn.relu(tf.matmul(L3, W4) + b4)
L4 = tf.nn.dropout(L4, keep_prob=keep_prob)

W5 = tf.get_variable("W5", shape=[512, VARIETY_SIZE],
                     initializer=tf.contrib.layers.xavier_initializer())
b5 = tf.Variable(tf.random_normal([VARIETY_SIZE]))
hypothesis = tf.matmul(L1, W5) + b5

# define cost/loss & optimizer
cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(
    logits=hypothesis, labels=Y_one_hot))
optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(cost)

softmax = tf.nn.softmax(hypothesis)
prediction = tf.argmax(hypothesis, 1)
correct_prediction = tf.equal(prediction, tf.argmax(Y_one_hot, 1))
accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))

sess = tf.Session()
sess.run(tf.global_variables_initializer())

#-----------------vision part--------------------------

cap = cv2.VideoCapture(0)

lower_blue = np.array([100,100,0])
upper_blue = np.array([130,255,255])
lower_red1 = np.array([0,100,0])
upper_red1 = np.array([10,255,255])
lower_red2 = np.array([170,100,0])
upper_red2 = np.array([179,255,255])

trafficImage=[0,0,0,0,0,0,0]

input_data = np.array([])
for i in range(0,VARIETY_SIZE,1):
    for j in range(0,DATA_SIZE/VARIETY_SIZE,1):
        #print i
        print '/home/sher/catkin_ws/src/sherlotics/scripts/img_data/'+str(i+1)+'_'+str(j+1)+'.jpg'
        img = cv2.imread('/home/sher/catkin_ws/src/sherlotics/scripts/img_data/'+str(i+1)+'_'+str(j+1)+'.jpg', cv2.IMREAD_COLOR)
        resized_img = cv2.resize(img, (IMAGE_SIZE, IMAGE_SIZE), interpolation=cv2.INTER_CUBIC)
        resized_img = resized_img / 255.0
        stack = np.array(resized_img)
        input_data=np.append(input_data,stack)
x_data = sess.run(tf.reshape(input_data, shape=[DATA_SIZE,TOTAL_IMAGE_SIZE]))
print x_data

output_data = []
for i in range(0,VARIETY_SIZE):
    for _ in range(DATA_SIZE/VARIETY_SIZE):
        output_data.append(i)
y_data = sess.run(tf.reshape(output_data, shape=[DATA_SIZE,1]))
print y_data

saver = tf.train.Saver()
#saver.restore(sess, "/home/m/catkin_ws/src/sherlotics/data/Turtlebot_Learning_Data.ckpt")

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_blue = cv2.inRange(img_hsv, lower_blue, upper_blue)
    mask_red1 = cv2.inRange(img_hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(img_hsv, lower_red2, upper_red2)

    mask_red = mask_red1 | mask_red2
    mask = mask_blue | mask_red

    numOfLabels, img_label, stats, centroids = cv2.connectedComponentsWithStats(mask)

    # Bitwise-AND mask and original image
    #img_result = cv2.bitwise_and(frame,frame, mask= mask)

    max_area=0

    for idx, centroid in enumerate(centroids):
        if stats[idx][0] == 0 and stats[idx][1] == 0:
            continue
        if np.any(np.isnan(centroid)):
            continue
        x, y, width, height, area = stats[idx]
        centerX, centerY = int(centroid[0]), int(centroid[1])

        if max_area < area :
            max_area=area
            final_x = x
            final_y = y
            final_width = width
            final_height = height

    cv2.rectangle(frame, (final_x, final_y), (final_x+final_width, final_y+final_height), (0, 0, 255),1)
    roi_frame = frame[final_y:final_y+final_height,final_x:final_x+final_width]
    roi_frame = cv2.resize(roi_frame, (IMAGE_SIZE, IMAGE_SIZE), interpolation=cv2.INTER_CUBIC)

    cv2.imshow( 'roi_frame', roi_frame )
    cv2.imshow( 'frame', frame )

    presentKey = cv2.waitKey(1) & 0xFF

    if presentKey == 32: # spacebar
        learning_frame = roi_frame/255.0
        while(True):
            optionKey = cv2.waitKey(1) & 0xFF
            if optionKey == 49: # 1
                print "1번 표지판 저장"
                trafficImage[1]=learning_frame
                break
            elif optionKey == 50: # 2
                print "2번 표지판 저장"
                trafficImage[2]=learning_frame
                break
            elif optionKey == 51: # 3
                print "3번 표지판 저장"
                trafficImage[3]=learning_frame
                break
            elif optionKey == 52: # 4
                print "4번 표지판 저장"
                trafficImage[4]=learning_frame
                break

            #---------------------------------------------------------
            if optionKey == 92: # \ 표지판 test
                print "표지판 test"
                x_data = sess.run(tf.reshape(learning_frame, shape=[1,TOTAL_IMAGE_SIZE]))
                print sess.run(softmax, feed_dict={X: x_data, keep_prob: 1})
                break

            elif optionKey == 13: # 엔터 learning
                print "learning"
                # image_data = np.array([trafficImage[1],trafficImage[2],trafficImage[3],trafficImage[4]])
                # x_data = sess.run(tf.reshape(image_data, shape=[DATA_SIZE,TOTAL_IMAGE_SIZE]))
                # output_data = [0,1,2,3]
                # y_data = sess.run(tf.reshape(output_data, shape=[DATA_SIZE,1]))

                for step in range(100):
                    sess.run(optimizer, feed_dict={X: x_data, Y: y_data, keep_prob: 0.7})
                    if step % 10 == 0:
                        loss, acc = sess.run([cost, accuracy], feed_dict={
                                             X: x_data, Y: y_data, keep_prob: 1})
                        print("Step: {:5}\tLoss: {:.3f}\tAcc: {:.2%}".format(
                            step, loss, acc))
                print "learning complete"
                break
            elif optionKey == 8: # 백스페이스 weight 저장
                print "weight 저장"
                saver = tf.train.Saver()
                save_path = saver.save(sess, "/home/sher/catkin_ws/src/sherlotics/data/Turtlebot_Learning_Data.ckpt")
                break
            elif optionKey == 27: #esc 나가기
                print "취소"
                break
            cv2.imshow( 'learning_frame', learning_frame )
    if presentKey == 27:
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

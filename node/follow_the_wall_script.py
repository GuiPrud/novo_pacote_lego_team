#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import math
import os
import numpy as np

tragetoria = np.zeros((115, 2))
tragetoria[1,0]=2.944
tragetoria[2,0]=6.250
tragetoria[3,0]=8.772
tragetoria[4,0]=10.833
tragetoria[5,0]=13.781
tragetoria[6,0]=16.139
tragetoria[7,0]=18.970
tragetoria[8,0]=24.272
tragetoria[9,0]=27.057
tragetoria[10,0]=30.123
tragetoria[11,0]=32.715
tragetoria[12,0]=35.275
tragetoria[13,0]=37.926
tragetoria[14,0]=40.552
tragetoria[15,0]=43.164
tragetoria[16,0]=46.103
tragetoria[17,0]=49.693
tragetoria[18,0]=53.120
tragetoria[19,0]=54.164
tragetoria[20,0]=55.045
tragetoria[21,0]=55.417
tragetoria[22,0]=55.417
tragetoria[23,0]=55.185
tragetoria[24,0]=53.863
tragetoria[25,0]=52.053
tragetoria[26,0]=50.198
tragetoria[27,0]=47.461
tragetoria[28,0]=45.373
tragetoria[29,0]=42.934
tragetoria[30,0]=41.773
tragetoria[31,0]=40.683
tragetoria[32,0]=40.497
tragetoria[33,0]=40.915
tragetoria[34,0]=42.541
tragetoria[35,0]=44.235
tragetoria[36,0]=45.846
tragetoria[37,0]=45.301
tragetoria[38,0]=44.599
tragetoria[39,0]=45.639
tragetoria[40,0]=46.236
tragetoria[41,0]=45.119
tragetoria[42,0]=44.313
tragetoria[43,0]=45.768
tragetoria[44,0]=48.029
tragetoria[45,0]=53.574
tragetoria[46,0]=56.850
tragetoria[47,0]=59.082
tragetoria[48,0]=61.433
tragetoria[49,0]=66.317
tragetoria[50,0]=71.767
tragetoria[51,0]=74.615
tragetoria[52,0]=77.386
tragetoria[53,0]=79.861
tragetoria[54,0]=79.937
tragetoria[55,0]=80.707
tragetoria[56,0]=81.031
tragetoria[57,0]=80.820
tragetoria[58,0]=80.637
tragetoria[59,0]=80.787
tragetoria[60,0]=79.409
tragetoria[61,0]=76.011
tragetoria[62,0]=74.015
tragetoria[63,0]=70.393
tragetoria[64,0]=65.361
tragetoria[65,0]=63.420
tragetoria[66,0]=61.359
tragetoria[67,0]=57.720
tragetoria[68,0]=54.194
tragetoria[69,0]=50.353
tragetoria[70,0]=44.821
tragetoria[71,0]=40.608
tragetoria[72,0]=37.567
tragetoria[73,0]=32.914
tragetoria[74,0]=29.590
tragetoria[75,0]=25.514
tragetoria[76,0]=21.745
tragetoria[77,0]=18.063
tragetoria[78,0]=15.840
tragetoria[79,0]=14.742
tragetoria[80,0]=14.485
tragetoria[81,0]=16.243
tragetoria[82,0]=20.150
tragetoria[83,0]=28.721
tragetoria[84,0]=30.911
tragetoria[85,0]=31.307
tragetoria[86,0]=30.725
tragetoria[87,0]=28.093
tragetoria[88,0]=24.459
tragetoria[89,0]=19.939
tragetoria[90,0]=15.723
tragetoria[91,0]=13.580
tragetoria[92,0]=14.232
tragetoria[93,0]=17.959
tragetoria[94,0]=22.921
tragetoria[95,0]=27.254
tragetoria[96,0]=30.096
tragetoria[97,0]=29.956
tragetoria[98,0]=28.861
tragetoria[99,0]=26.858
tragetoria[100,0]=22.898
tragetoria[101,0]=14.698
tragetoria[102,0]=11.344
tragetoria[103,0]=8.595
tragetoria[104,0]=7.360
tragetoria[105,0]=3.307
tragetoria[106,0]=-0.374
tragetoria[107,0]=-3.449
tragetoria[108,0]=-6.943
tragetoria[109,0]=-9.948
tragetoria[110,0]=-12.044
tragetoria[111,0]=-11.043
tragetoria[112,0]=-5.731
tragetoria[113,0]=-1.119
tragetoria[114,0]=4.169


tragetoria[1,1]= -0.723
tragetoria[2,1]= -0.44
tragetoria[3,1]= -0.663
tragetoria[4,1]= -0.565
tragetoria[5,1]= -0.291
tragetoria[6,1]= -0.426
tragetoria[7,1]= -0.355
tragetoria[8,1]= -0.186
tragetoria[9,1]= 0.043
tragetoria[10,1]= 0.143
tragetoria[11,1]= 0.061
tragetoria[12,1]= -0.029
tragetoria[13,1]= 0.301
tragetoria[14,1]= 0.145
tragetoria[15,1]= 0.365
tragetoria[16,1]= 0.185
tragetoria[17,1]= 0.279
tragetoria[18,1]= -0.212
tragetoria[19,1]= 0.394
tragetoria[20,1]=1.246
tragetoria[21,1]=2.553
tragetoria[22,1]=3.810
tragetoria[23,1]=5.521
tragetoria[24,1]=7.983
tragetoria[25,1]=9.440
tragetoria[26,1]=10.508
tragetoria[27,1]=10.999
tragetoria[28,1]=11.484
tragetoria[29,1]=12.198
tragetoria[30,1]=13.049
tragetoria[31,1]=15.218
tragetoria[32,1]=18.195
tragetoria[33,1]=21.584
tragetoria[34,1]=25.240
tragetoria[35,1]=30.778
tragetoria[36,1]=37.223
tragetoria[37,1]=41.874
tragetoria[38,1]=45.824
tragetoria[39,1]=49.865
tragetoria[40,1]=55.075
tragetoria[41,1]=59.480
tragetoria[42,1]=65.535
tragetoria[43,1]=69.875
tragetoria[44,1]=72.421
tragetoria[45,1]=74.560
tragetoria[46,1]=76.807
tragetoria[47,1]=80.219
tragetoria[48,1]=84.631
tragetoria[49,1]=89.380
tragetoria[50,1]=95.154
tragetoria[51,1]=99.133
tragetoria[52,1]=102.796
tragetoria[53,1]=110.063
tragetoria[54,1]=115.397
tragetoria[55,1]=119.263
tragetoria[56,1]=126.411
tragetoria[57,1]=129.872
tragetoria[58,1]=134.778
tragetoria[59,1]=137.543
tragetoria[60,1]=137.598
tragetoria[61,1]=137.565
tragetoria[62,1]=136.210
tragetoria[63,1]=130.549
tragetoria[64,1]=123.333
tragetoria[65,1]=120.231
tragetoria[66,1]=117.101
tragetoria[67,1]=112.251
tragetoria[68,1]=107.140
tragetoria[69,1]=101.582
tragetoria[70,1]=95.353
tragetoria[71,1]=89.921
tragetoria[72,1]=86.100
tragetoria[73,1]=80.801
tragetoria[74,1]=76.906
tragetoria[75,1]=73.228
tragetoria[76,1]=69.337
tragetoria[77,1]=65.655
tragetoria[78,1]=63.224
tragetoria[79,1]=60.175
tragetoria[80,1]=57.384
tragetoria[81,1]=55.035
tragetoria[82,1]=53.361
tragetoria[83,1]=53.096
tragetoria[84,1]=52.118
tragetoria[85,1]=49.276
tragetoria[86,1]=46.713
tragetoria[87,1]=44.850
tragetoria[88,1]=46.573
tragetoria[89,1]=50.347
tragetoria[90,1]=50.301
tragetoria[91,1]=46.760
tragetoria[92,1]=44.197
tragetoria[93,1]=41.914
tragetoria[94,1]=40.610
tragetoria[95,1]=39.305
tragetoria[96,1]=35.858
tragetoria[97,1]=31.385
tragetoria[98,1]=27.192
tragetoria[99,1]=23.092
tragetoria[100,1]=21.089
tragetoria[101,1]=21.555
tragetoria[102,1]=23.605
tragetoria[103,1]=24.350
tragetoria[104,1]=24.723
tragetoria[105,1]=25.096
tragetoria[106,1]=23.791
tragetoria[107,1]=19.878
tragetoria[108,1]=13.448
tragetoria[109,1]=7.811
tragetoria[110,1]=3.991
tragetoria[111,1]=2.080
tragetoria[112,1]= -0.063
tragetoria[113,1]= -0.156
tragetoria[114,1]= 0.124
orientacao_objetivo = Quaternion(x=0,y=0,z=0,w=0)
orientacao_atual = Quaternion(x=0,y=0,z=0,w=0)
posicao_objetivo = Point(x = 5, y = 5)
posicao_atual = Point(0,0,0);

posicao = 1

os.chdir(r'/root/catkin_ws/src/novo_pacote_teste_lego_team/node')
def callback(msg):
	global orientacao_atual
	global posicao_atual
	orientacao_atual = msg.pose.pose.orientation
	posicao_atual = msg.pose.pose.position


class FollowTheGap(object):    

    def __init__(self):	
	
	#rospy.loginfo(str(objetivo.x))
	rate = rospy.Rate(40)
	direc_msg = AckermannDriveStamped()
        while not rospy.is_shutdown():

		global orientacao_objetivo
		global orientacao_atual
		global posicao_objetivo

		angulo_atual = 2*math.asin(orientacao_atual.z)		
		
		#rospy.loginfo(str(os.getcwd()))	
		
		# Essas condições servem para que os angulos fiquem entre 0<theta<180 e 0>theta>-180

		if(posicao_atual.x < posicao_objetivo.x):
			angulo_objetivo = math.atan((posicao_objetivo.y - posicao_atual.y)/(posicao_objetivo.x - posicao_atual.x))				
		else:	
			if (posicao_atual.y < posicao_objetivo.y):
				angulo_objetivo = math.pi+math.atan((posicao_objetivo.y - posicao_atual.y)/(posicao_objetivo.x - posicao_atual.x))	
			else:
				angulo_objetivo = -math.pi+math.atan((posicao_objetivo.y - posicao_atual.y)/(posicao_objetivo.x - posicao_atual.x))	
		
		#rospy.loginfo(str(angulo_objetivo*180/math.pi))
		erro = angulo_objetivo -angulo_atual

		Kerro = -1	
		K = 1

		distancia_objetivo = math.sqrt(math.pow((posicao_objetivo.y - posicao_atual.y),2) + math.pow((posicao_objetivo.x - posicao_atual.x),2))

		#velocidade = Kerro*abs(erro)+K*distancia_objetivo/27
		global posicao
		velocidade = 4
		
		if(distancia_objetivo < 0.5):
			if(posicao == 114):
				posicao = 1
			else:			
				posicao = posicao + 1

		novo_objetivo = tragetoria[posicao,:]
		posicao_objetivo = Point(x = novo_objetivo[0], y = novo_objetivo[1])
						
		#if(distancia_objetivo < 0.5):
		#	if(posicao == 1):
		#		posicao_objetivo = Point(x = 15, y = 9)
		#		posicao = 2
		#	elif(posicao == 2):
		#		posicao_objetivo = Point(x = 18, y = 5)
		#		posicao = 3
		#	elif(posicao == 3):
		#		posicao_objetivo = Point(x = 13, y = 0)
		#		posicao = 1

		direc_msg.drive.speed = velocidade
		direc_msg.drive.steering_angle = erro

		gap_pub.publish(direc_msg)
		rate.sleep()


def main():
	rospy.init_node('new_lego_team_node', anonymous = False)

	#self.sub = rospy.Subscriber('/scan', LaserScan, callback)
	sub = rospy.Subscriber('/new_lego_team_id/odom', Odometry, callback)

	fg = FollowTheGap()
	rospy.spin()


gap_pub = rospy.Publisher('/new_lego_team_id/drive', AckermannDriveStamped, queue_size=10)

main()

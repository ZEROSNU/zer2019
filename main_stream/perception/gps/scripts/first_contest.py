#!/usr/bin/env python
import rospy
import numpy
import math
from beginner_tutorials.msg import location
from core_msgs.msg import MissionState
from core_msgs.msg import ActiveNode

nodename = 'gps'
active = True
def signalResponse(data) :
    if 'zero_monitor' in data.active_nodes :
        if nodename in data.active_nodes :
            active = True
        else :
            active = False
    else :
        rospy.signal_shutdown('no monitor')
rospy.Subscriber('/active_nodes', ActiveNode, signalResponse)

pub = rospy.Publisher('/mission_state', MissionState, queue_size = 10)
rospy.init_node(nodename, anonymous=True)
mission = MissionState()
count_time = 0


Mission_number = 26

limit = 0.0001

def slope(X1, X2): # X1과 X2를 지나는 직선의 기울기 
    return (X2[1] - X1[1]) / (X2[0] - X1[0])

def y_int(X1, X2): # X1과 X2를 지나는 직선의 y 절편 
    return X1[1] - X1[0] * slope(X1, X2)

def cross_point (X1, X2, X3, X4): # X1, X2을 지나는 직선과 X3, X4를 지나는 직선의 교점 
    X = [0,0]
    X[0] = (y_int(X3, X4) - y_int(X1, X2)) / (slope(X3, X4) - slope(X1, X2))
    X[1] = (slope(X1, X2) * y_int(X3, X4) - slope(X3, X4) * y_int(X1, X2)) / (slope(X1, X2) - slope(X3, X4))
    return X

def area_3 (X1, X2, X3): # 삼각형 X1X2X3 의 넓이 
    return abs(X1[0]*X2[1] + X2[0]*X3[1] + X3[0]*X1[1] - X2[0]*X1[1] - X3[0]*X2[1] - X1[0]*X3[1]) / 2.0

def area_4 (X1, X2, X3, X4): # 사각형 X1X2X3X4 의 넓이 
    return abs(X1[0]*X2[1] + X2[0]*X3[1] + X3[0]*X4[1] + X4[0]*X1[1] - X2[0]*X1[1] - X3[0]*X2[1] - X4[0]*X3[1] - X1[0]*X4[1]) / 2.0

def in_out(location, n): # 임의의 점이 사각형 내부에 있는지 판단. 만약 점 O가 사각형 ABCD 내부에 있으면 사각형 ABCD의 넓이 = 삼각형 ABO, BCO, CDO, DAO의 넓이 합임을 이용 . 안에 있으면 1, 밖에 있으면 0을 반환
    
    S = area_4 (data[n][0], data[n][1], data[n][2], data[n][3]) 
    S1 = area_3 (location, data[n][0], data[n][1])
    S2 = area_3 (location, data[n][1], data[n][2])
    S3 = area_3 (location, data[n][2], data[n][3])
    S4 = area_3 (location, data[n][3], data[n][0])
    tmp = S / (S1 + S2 + S3 + S4)

    #print ( tmp ) 
    if abs (tmp - 1 ) < limit: # 이상적인 경우 tmp = 1이지만 실제로는 오차가 존재하므로 그 범위를 limit로 제한함. limit 를 조절하여 사각형 내부의 유무에 대한 엄밀한 정도를 조절함. 
        return 1

    if abs (tmp - 1 ) > limit :
        return 0

raw_data = [ # 지도상에서 검은 점. 실제로 측정한 데이터, 처음을 제외하고 순서대로 지도상에서 1 ~ 75를 가리킴  
    [0,0], # 0
    [37.2390632629,126.772987366], # 1
    [37.2390403748,126.773086548], # 2
    [37.2396125793,126.773452759], # 3
    [37.2396316528,126.773361206], # 4
    [37.2397346497,126.773399353], # 5 
    [37.2397956848,126.7734375], # 6
    [37.2398643494,126.773498535], # 7
    [37.2398109436,126.773681641], # 8
    [37.2397346497,126.773628235], # 9
    [37.2396888733,126.773590088],# 10
    [37.2398757935,126.773155212], # 11
    [37.2399253845,126.773010254], # 12
    [37.2400970459,126.773040771], # 13
    [37.2400550842,126.773109436], # 14
    [37.2399215698,126.773162842], # 15
    [37.2402534485,126.773200989], # 16
    [37.2401809692,126.773170471], # 17
    [37.2405281067,126.773345947], # 18
    [37.2404670715,126.773330688], # 19
    [37.2402381897,126.773384094], # 20
    [37.2402229309,126.773361206], # 21
    [37.2401351929,126.773521423], # 22
    [37.2402000427,126.7735672], # 23
    [37.2403144836,126.773803711], # 24
    [37.2402877808,126.773910522], # 25
    [37.2401351929,126.773979187], # 26
    [37.2400627136,126.773933411], # 27
    [37.2399940491,126.773681641], # 28
    [37.2400550842,126.773612976], # 29
    [37.2400360107,126.774108887], # 30
    [37.239982605, 126.774055481], # 31
    [37.2398223877,126.774124146], # 32
    [37.2397956848,126.77419281], # 33
    [37.2400588989,126.774360657], # 34
    [37.2400817871,126.774299622], # 35
    [37.2401351929,126.774429321], #36
    [37.2401771545,126.774375916], # 37       
    [37.2403182983,126.774482727], # 38
    [37.240398407, 126.774360657], # 39
    [37.2404289246,126.774429321], # 40
    [37.2404785156,126.77419281],  # 41
    [37.2404899597,126.774162292], # 42
    [37.2404251099,126.773986816], # 43
    [37.240447998,126.773918152], # 44
    [37.2405815125,126.773788452], # 45
    [37.2406311035,126.773841858], # 46
    [37.2411689758,126.774459839], # 47
    [37.2412109375,126.774337769], # 48
    [37.2415390015,126.774375916], # 49
    [37.2415351868,126.774536133], # 50
    [37.2416534424,126.774658203], # 51
    [37.2417259216,126.774665833], # 52
    [37.2418441772,126.774635315], # 53
    [37.2418251038,126.774391174], # 54
    [37.2417068481,126.774261475], # 55
    [37.2416534424,126.774246216], # 56
    [37.2416114807,126.774017334], # 57
    [37.2416725159,126.774002075], # 58
    [37.2418212891,126.773826599], # 59
    [37.2418327332,126.773750305], # 60
    [37.2417106628,126.77381134], # 61
    [37.2426109314,126.773658752], # 62
    [37.2426185608,126.773780823], # 63
    [37.2426834106,126.773895264], # 64
    [37.2427940369,126.773918152], # 65
    [37.2429084778,126.773757935], # 66
    [37.2429199219,126.773643494], # 67
    [37.2427215576,126.774154663], # 68
    [37.242816925,126.774208069], # 69
    [37.2430496216,126.774307251], # 70
    [37.2430801392,126.774383545], # 71
    [37.2428817749,126.774688721], # 72
    [37.2427940369,126.774673462], # 73
    [37.2425498962,126.774597168], # 74
    [37.242515564,126.774330139] # 75
]

cross_data = numpy.zeros(shape = (29,2)).tolist() # 위의 데이터를 바탕으로 교차점을 구하여 얻은 빨간 점. 실제로 들어갈 수 없는 지점이 존재하기 때문. 역시 마찬가지로 위부터 순서대로 1 ~ 28 점을 의미함 

cross_data[1] = cross_point (raw_data[4], raw_data[3], raw_data[10], raw_data[9])
cross_data[2] = cross_point (raw_data[7], raw_data[8], raw_data[10], raw_data[9])
cross_data[3] = cross_point (raw_data[5], raw_data[6], raw_data[7], raw_data[9])
cross_data[4] = cross_point (raw_data[6], raw_data[5], raw_data[4], raw_data[3])

cross_data[5] = cross_point (raw_data[13], raw_data[14], raw_data[11], raw_data[15])

cross_data[6] = cross_point (raw_data[16], raw_data[17], raw_data[20], raw_data[21])
cross_data[7] = cross_point (raw_data[18], raw_data[19], raw_data[20], raw_data[21])

cross_data[8] = cross_point (raw_data[22], raw_data[23], raw_data[28], raw_data[29])
cross_data[9] = cross_point (raw_data[22], raw_data[23], raw_data[24], raw_data[25])
cross_data[10] = cross_point (raw_data[24], raw_data[25], raw_data[26], raw_data[27])
cross_data[11] = cross_point (raw_data[29], raw_data[28], raw_data[26], raw_data[27])

cross_data[12] = cross_point (raw_data[30], raw_data[31], raw_data[32], raw_data[33])
cross_data[13] = cross_point (raw_data[30], raw_data[31], raw_data[35], raw_data[34])

cross_data[14] = cross_point (raw_data[39], raw_data[40], raw_data[36], raw_data[37])

cross_data[15] = cross_point (raw_data[41], raw_data[42], raw_data[43], raw_data[44])
cross_data[16] = cross_point (raw_data[45], raw_data[46], raw_data[43], raw_data[44])

cross_data[17] = cross_point (raw_data[41], raw_data[47], raw_data[49], raw_data[50])

cross_data[18] = cross_point (raw_data[51], raw_data[52], raw_data[50], raw_data[49])
cross_data[19] = cross_point (raw_data[55], raw_data[56], raw_data[50], raw_data[49])
cross_data[20] = cross_point (raw_data[55], raw_data[56], raw_data[53], raw_data[54])
cross_data[21] = cross_point (raw_data[51], raw_data[52], raw_data[53], raw_data[54])

cross_data[22] = cross_point (raw_data[72], raw_data[73], raw_data[74], raw_data[75])
cross_data[23] = cross_point (raw_data[72], raw_data[73], raw_data[71], raw_data[70])
cross_data[24] = cross_point (raw_data[71], raw_data[70], raw_data[69], raw_data[68])
cross_data[25] = cross_point (raw_data[69], raw_data[68], raw_data[74], raw_data[75])

cross_data[26] = cross_point (raw_data[66], raw_data[67], raw_data[65], raw_data[64])
cross_data[27] = cross_point (raw_data[65], raw_data[64], raw_data[62], raw_data[63])

cross_data[28] = cross_point (raw_data[59], raw_data[60], raw_data[58], raw_data[57])

data = [ # 미션 구역을 사각형으로 나눌때 각 미션 구역마다의 꼭지점, 마찬가지로 처음을 제외하고 순서대로 Area 1 ~ 26을 가리킴 
    [[0,0], [0,0], [0,0], [0,0]],
    [raw_data[1],raw_data[2],raw_data[3],raw_data[4]],
    [cross_data[1],cross_data[2],cross_data[3],cross_data[4]],
    [raw_data[5],raw_data[6],raw_data[15],raw_data[11]],
    [raw_data[11],raw_data[12],raw_data[13],cross_data[5]],
    [raw_data[13],raw_data[14],raw_data[17],raw_data[16]],
    [raw_data[16],cross_data[6],cross_data[7],raw_data[18]],
    [raw_data[21],raw_data[22],raw_data[23],raw_data[20]],
    [cross_data[8],cross_data[9],cross_data[10],cross_data[11]],
    [raw_data[27],raw_data[31],raw_data[30],raw_data[26]],
    [raw_data[33],raw_data[34],cross_data[13],cross_data[12]],
    [raw_data[34],raw_data[36],raw_data[37],raw_data[35]],
    [raw_data[36],raw_data[38],raw_data[40],cross_data[14]],
    [raw_data[40],raw_data[41],raw_data[42],raw_data[39]],
    [raw_data[41],raw_data[46],cross_data[16],cross_data[15]],
    [raw_data[43],raw_data[44],raw_data[24],raw_data[25]],
    [raw_data[28],raw_data[29],raw_data[7],raw_data[8]],

    [raw_data[41],raw_data[47],raw_data[48],raw_data[46]],
    [raw_data[47],cross_data[17],raw_data[49],raw_data[48]],
    [cross_data[18],cross_data[21],cross_data[20],cross_data[19]],
    [raw_data[53],raw_data[74],raw_data[75],raw_data[54]],
    [cross_data[22],cross_data[23],cross_data[24],cross_data[25]],
    [raw_data[68],raw_data[69],raw_data[65],raw_data[64]],
    [cross_data[27],cross_data[26],raw_data[67],raw_data[62]],
    [raw_data[59],raw_data[63],raw_data[62],raw_data[60]],
    [raw_data[60],raw_data[61],raw_data[57],cross_data[28]],
    [raw_data[58],raw_data[57],raw_data[56],raw_data[55]],
]



count_2 = 0 
    
def callback(msg):

    location = [msg.Lat, msg.Long]

    global count_time
    global count_2
    area_inout = numpy.zeros(shape = (Mission_number + 1,1)) # area_inout 에서는 매 순간마다 각 area 구역 안에 현재 위치가 포함되어 있는지 저장한다. 

    for i in range(1,Mission_number+1):
        area_inout[i] = in_out(location, i)

    #print (area_inout)

    for i in range(1,Mission_number + 1):

        if area_inout[i] == 1:
            if i == 1 or i == 5 or i == 7 or i == 8 or i == 13 or i == 15 or i == 16:
                mission.header.stamp = rospy.Time.now()
                mission.header.seq = count_time
                count_time = count_time +1
                mission.header.frame_id = 'gps'
                mission.mission_state = 'FORWARD_MOTION'

                if active :
                    pub.publish(mission)

            if  i == 3:
                mission.header.stamp = rospy.Time.now()
                mission.header.seq = count_time
                count_time = count_time +1
                mission.header.frame_id = 'gps'
                mission.mission_state = 'FORWARD_MOTION'
                count_2 = count_2 + 1

                if active :
                    pub.publish(mission)

            if i == 2:

                if count_2 == 0: # Mission area 2의 경우 처음 지나칠 때는 좌회전을 하지만 그 뒤로는 직진을 해야 한다. 그래서 처음 count_2 = 0으로 하고 count_2 가 0이면 좌회전을 하고, 이후 area 3을 지나면 이를 증가시켜 다음번에 area 2를 지나칠때 count_2 >0이면 직진을 하도록 한다.  

                    mission.header.stamp = rospy.Time.now()
                    mission.header.seq = count_time
                    count_time = count_time + 1
                    mission.header.frame_id = 'gps'
                    mission.mission_state = 'LEFT_MOTION'
                    
                    if active :
                        pub.publish(mission)

                if count_2 > 0:

                    mission.header.stamp = rospy.Time.now()
                    mission.header.seq = count_time
                    count_time = count_time + 1
                    mission.header.frame_id = 'gps'
                    mission.mission_state = 'FORWARD_MOTION'
                    
                    if active :
                        pub.publish(mission)

            if i == 4 or i == 6 :

                mission.header.stamp = rospy.Time.now()
                mission.header.seq = count_time
                count_time = count_time +1
                mission.header.frame_id = 'gps'
                mission.mission_state = 'RIGHT_MOTION'

                if active :
                    pub.publish(mission)
            
            if i == 9 or i == 11:

                mission.header.stamp = rospy.Time.now()
                mission.header.seq = count_time
                count_time = count_time +1
                mission.header.frame_id = 'gps'
                mission.mission_state = 'FORWARD_MOTION_SLOW'

                if active :
                    pub.publish(mission)

            if i == 10:

                mission.header.stamp = rospy.Time.now()
                mission.header.seq = count_time
                count_time = count_time +1
                mission.header.frame_id = 'gps'
                mission.mission_state = 'LEFT_MOTION_SLOW'

                if active :
                    pub.publish(mission)

            if i == 12 or i == 14:

                mission.header.stamp = rospy.Time.now()
                mission.header.seq = count_time
                count_time = count_time +1
                mission.header.frame_id = 'gps'
                mission.mission_state = 'LEFT_MOTION'

                if active :
                    pub.publish(mission)
            

def mainloop():

    rospy.init_node('gps', anonymous=True)
    
    rospy.Subscriber('location_msg', location, callback)

    rospy.spin()

if __name__ == '__main__':

    try:
        mainloop()
        
    except rospy.ROSInterruptException:
        pass

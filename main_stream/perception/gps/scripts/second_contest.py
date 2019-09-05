#!/usr/bin/env python
import rospy
import numpy
import math
from core_msgs.msg import Location
from core_msgs.msg import MissionState
from core_msgs.msg import ActiveNode

nodename = 'gps'
active = True

def signalResponse(data) :
    global active
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


Mission_number = 33

limit = 0.0001

def slope(X1, X2): 
    return (X2[1] - X1[1]) / (X2[0] - X1[0])

def y_int(X1, X2): 
    return X1[1] - X1[0] * slope(X1, X2)

def cross_point (X1, X2, X3, X4): 
    X = [0,0]
    X[0] = (y_int(X3, X4) - y_int(X1, X2)) / (slope(X3, X4) - slope(X1, X2))
    X[1] = (slope(X1, X2) * y_int(X3, X4) - slope(X3, X4) * y_int(X1, X2)) / (slope(X1, X2) - slope(X3, X4))
    return X

def area_3 (X1, X2, X3): 
    return abs(X1[0]*X2[1] + X2[0]*X3[1] + X3[0]*X1[1] - X2[0]*X1[1] - X3[0]*X2[1] - X1[0]*X3[1]) / 2.0

def area_4 (X1, X2, X3, X4): 
    return abs(X1[0]*X2[1] + X2[0]*X3[1] + X3[0]*X4[1] + X4[0]*X1[1] - X2[0]*X1[1] - X3[0]*X2[1] - X4[0]*X3[1] - X1[0]*X4[1]) / 2.0

def in_out(location, n): 
    
    S = area_4 (data[n][0], data[n][1], data[n][2], data[n][3])
    S1 = area_3 (location, data[n][0], data[n][1])
    S2 = area_3 (location, data[n][1], data[n][2])
    S3 = area_3 (location, data[n][2], data[n][3])
    S4 = area_3 (location, data[n][3], data[n][0])
    tmp = S / (S1 + S2 + S3 + S4)

    #print ( tmp )
    if abs (tmp - 1 ) < limit:
        return 1

    if abs (tmp - 1 ) > limit :
        return 0

p = [
    [37.239034830165046 , 126.77317240732009], #0
    [37.23901774732779 , 126.77305170791442], #1
    [37.239045506936364 , 126.77293637292678], #2
    [37.23941492229267 , 126.77341380613143], #3
    [37.23947576047644 , 126.77399113191007], #4
    [37.23949607221611 , 126.77377025802127], #5
    [37.23958362116058 , 126.77353154141895], #6
    [37.239673305339714 , 126.77332232911579], #7
    [37.23980996674085 , 126.77304874379627], #8
    [37.239920489680244 , 126.77284092175114], #9
    [37.23958925588845 , 126.77416095810554], #10
    [37.23968748140508 , 126.77390346604011], #11
    [37.239775030127205 , 126.77363524513862], #12
    [37.23984589889854 , 126.7734228446651], #13
    [37.239976678008844 , 126.77317339922672], #14
    [37.24011547418378 , 126.77290786053425], #15
    [37.23977615336765 , 126.77428112401196], #16
    [37.23985943132482 , 126.7740719117088], #17
    [37.239949115175776 , 126.77381441964337], #18
    [37.240064422827366 , 126.77351937665173], #19
    [37.24022243673009 , 126.7732864631547], #20
    [37.24031587061285 , 126.77305287792262], #21
    [37.239965171132866 , 126.77440598369162], #22
    [37.24004417822965 , 126.7741860425524], #23
    [37.24017016234784 , 126.77395805478614], #24
    [37.240280502927966 , 126.77366385663117], #25
    [37.24038086286011 , 126.77340100014771], #26
    [37.24050684641556 , 126.77317301238145], #27
    [37.24014797434832 , 126.7745151883421], #28
    [37.24025474041676 , 126.77427647173977], #29
    [37.24038713013159 , 126.7740672594366], #30
    [37.24050884305136 , 126.7738151317892], #31
    [37.240588141758245 , 126.77355495751476], #32
    [37.24069501442636 , 126.77332712787666], #33
    [37.24036446482566 , 126.77462746694346], #34
    [37.240496854347775 , 126.77442898347635], #35
    [37.24056518433273 , 126.7741661269929], #36
    [37.24064726831952 , 126.77392622685943], #37
    [37.24100543265035 , 126.77449301852414], #38
    [37.24113579929796 , 126.77416925460989], #39
    [37.2415436395197 , 126.77466546327764], #40
    [37.24154150423632 , 126.77427117855245], #41
    [37.24157536304818 , 126.77389030487234], #42
    [37.241615933403665 , 126.7736301305979], #43
    [37.24180969415683 , 126.77469917034932], #44
    [37.2417968825009 , 126.7742271015627], #45
    [37.24185453493538 , 126.77396156287023], #46
    [37.24187283810664 , 126.77354524622899], #47
    [37.242528733138386 , 126.77466419060704], #48
    [37.242558626708515 , 126.7742618592548], #49
    [37.24264538946419 , 126.77385356989657], #50
    [37.242662471479285 , 126.77358266678607], #51
    [37.24294720275367 , 126.77462978111419], #52
    [37.24296121698161 , 126.77423817859801], #53
    [37.24296030094653 , 126.773876080381], #54
    [37.242975247647514 , 126.77355421529921], #55
    [37.23927805930314, 126.77317880044757], #56
    [37.23925884117297,126.77322708020984], #57
    [37.239461698966586,126.77335582624255], #58
    [37.2394681049933,126.77329413543521] #59
] 



data = [
    [p[1],p[2],p[7],p[6]], # 0
    [p[0],p[1],p[6],p[3]], # 1
    [p[4],p[5],p[11],p[10]], # 2
    [p[5],p[6],p[12],p[11]], # 3
    [p[6],p[7],p[13],p[12]], # 4
    [p[7],p[8],p[14],p[13]], # 5
    [p[8],p[9],p[15],p[14]], # 6
    [p[10],p[2],p[7],p[6]], # 7
    [p[12],p[13],p[19],p[18]], # 8
    [p[14],p[15],p[21],p[20]], # 9
    [p[16],p[17],p[23],p[22]], # 10
    [p[17],p[18],p[24],p[23]], # 11
    [p[18],p[19],p[25],p[24]], # 12
    [p[19],p[20],p[26],p[25]], # 13
    [p[20],p[21],p[27],p[26]], # 14
    [p[22],p[23],p[29],p[28]], # 15
    [p[24],p[25],p[31],p[30]], # 16
    [p[26],p[27],p[33],p[32]], # 17
    [p[28],p[29],p[35],p[34]], # 18
    [p[29],p[30],p[36],p[35]], # 19
    [p[30],p[31],p[37],p[36]], # 20
    [p[37],p[32],p[38],p[37]], # 21
    [p[32],p[33],p[39],p[38]], # 22
    [p[36],p[37],p[39],p[38]], # 23
    [p[38],p[39],p[47],p[40]], # 24
    [p[40],p[47],p[45],p[44]], # 25
    [p[47],p[42],p[46],p[45]], # 26
    [p[42],p[43],p[47],p[46]], # 27
    [p[44],p[45],p[49],p[48]], # 28
    [p[46],p[47],p[51],p[50]], # 29
    [p[48],p[49],p[53],p[52]], # 30
    [p[49],p[50],p[54],p[53]], # 31
    [p[50],p[51],p[55],p[54]], # 32
    [p[56],p[57],p[58],p[59]] # 33
]

count_4 = 0
count_12 = 0
count_25 = 0

def callback(msg):
    global active
    location = [msg.Lat, msg.Long]

    global count_time
    global count_4
    area_inout = numpy.zeros(shape = (Mission_number,1))
    tmp = 0
    t = 0

    for i in range(0,Mission_number):
        area_inout[i] = in_out(location, i)

    for i in range(1,Mission_number):

        if area_inout[i] == 1:
            tmp = i
            t = t + 1
    print t
    print tmp
    print '=========='
    if t == 1 or tmp == 33:

        if tmp == 33:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'PARKING_READY'

            if active :
                pub.publish(mission)

        if tmp == 0:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'READY_TO_PARK'

            if active :
                pub.publish(mission)

        if tmp == 1:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'PARKING'

            if active :
                pub.publish(mission)

        if tmp == 4:
            if count_4 == 0:
                mission.header.stamp = rospy.Time.now()
                mission.header.seq = count_time
                count_time = count_time +1
                mission.header.frame_id = 'gps'
                mission.mission_state = 'INTERSECTION_LEFT'

                if active :
                    pub.publish(mission)

            if count_4 > 0:
                mission.header.stamp = rospy.Time.now()
                mission.header.seq = count_time
                count_time = count_time +1
                mission.header.frame_id = 'gps'
                mission.mission_state = 'INTERSECTION_STRAIGHT'

                if active :
                    pub.publish(mission)
                

        if tmp == 5:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'DRIVING_SECTION'
            count_4 = count_4 + 1

            if active :
                pub.publish(mission)

        if tmp == 6:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'INTERSECTION_RIGHT'
            count_4 = count_4 + 1

            if active :
                pub.publish(mission)

        if tmp == 9:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'DRIVING_SECTION'

            if active :
                pub.publish(mission)

        if tmp == 14:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'INTERSECTION_RIGHT'

            if active :
                pub.publish(mission)

        if tmp == 13:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'DRIVING_SECTION'

            if active :
                pub.publish(mission)

        if tmp == 12:

            if count_12 == 0:
                mission.header.stamp = rospy.Time.now()
                mission.header.seq = count_time
                count_time = count_time +1
                mission.header.frame_id = 'gps'
                mission.mission_state = 'INTERSECTION_LEFT'

                if active :
                    pub.publish(mission)

            if count_12 > 0:
                mission.header.stamp = rospy.Time.now()
                mission.header.seq = count_time
                count_time = count_time +1
                mission.header.frame_id = 'gps'
                mission.mission_state = 'INTERSECTION_STRAIGHT'

                if active :
                    pub.publish(mission)

        if tmp == 16:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'DRIVING_SECTION'

            if active :
                pub.publish(mission)

        if tmp == 20:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'INTERSECTION_STRAIGHT'

            if active :
                pub.publish(mission)

        if tmp == 23:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'OBSTACLE_STATIC'

            if active :
                pub.publish(mission)


        if tmp == 24:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'OBSTACLE_STATIC'

            if active :
                pub.publish(mission)

        if tmp == 25:
            if count_25 == 0:
                mission.header.stamp = rospy.Time.now()
                mission.header.seq = count_time
                count_time = count_time +1
                mission.header.frame_id = 'gps'
                mission.mission_state = 'INTERSECTION_STRAIGHT'

                if active :
                    pub.publish(mission)

            if count_25 > 0:
                mission.header.stamp = rospy.Time.now()
                mission.header.seq = count_time
                count_time = count_time +1
                mission.header.frame_id = 'gps'
                mission.mission_state = 'INTERSECTION_RIGHT'

                if active :
                    pub.publish(mission)

        if tmp == 28:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'DRIVING_SECTION'

            if active :
                pub.publish(mission)

        if tmp == 30:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'INTERSECTION_LEFT'

            if active :
                pub.publish(mission)

        if tmp == 31:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'DRIVING_SECTION'

            if active :
                pub.publish(mission)

        if tmp == 32:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'INTERSECTION_LEFT'

            if active :
                pub.publish(mission)

        if tmp == 29:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'DRIVING_SECTION'

            if active :
                pub.publish(mission)

        if tmp == 27:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'INTERSECTION_LEFT'

            if active :
                pub.publish(mission)

        if tmp == 26:
            mission.header.stamp = rospy.Time.now()
            mission.header.seq = count_time
            count_time = count_time +1
            mission.header.frame_id = 'gps'
            mission.mission_state = 'DRIVING_SECTION'

            if active :
                pub.publish(mission)


def mainloop():

    rospy.init_node('gps', anonymous=True)
    
    rospy.Subscriber('/Location_msg', Location, callback)

    rospy.spin()

if __name__ == '__main__':

    try:
        mainloop()
        
    except rospy.ROSInterruptException:
        pass

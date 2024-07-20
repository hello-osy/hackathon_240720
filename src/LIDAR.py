#https://github.com/slamtec/rplidar_ros 여기서 찾으면 됨.
#어제는 roslaunch rplidar_ros view_rplidar_c1.launch 했더니 라이다 창 뜬 것이었음.

#github에 올라온 것 중에서 src/node.cpp 를 파이썬으로 변환한 것임.

import rplidar #RPLIDAR 장치를 제어할 때 사용하는 라이브러리
import rospy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty, EmptyResponse

# LIDAR 드라이버 초기화
lidar = rplidar.RPLidar('/dev/ttyUSB0') #RPLIDAR 장치를 초기화하고 지정한 시리얼 포트에 연결

def publish_scan(pub, scan_data, frame_id): #레이저 스캔 데이터를 받아 ROS 메시지 타입인 'LaserScan'으로 변환해서 발행한다.
    scan_msg = LaserScan()
    scan_msg.header.stamp = rospy.Time.now()
    scan_msg.header.frame_id = frame_id

    scan_msg.angle_min = 0.0
    scan_msg.angle_max = 2 * 3.14159
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / len(scan_data)
    scan_msg.time_increment = 0.0
    scan_msg.scan_time = 0.1
    scan_msg.range_min = 0.15
    scan_msg.range_max = 10.0

    scan_msg.ranges = []
    scan_msg.intensities = []
    
    for (_, angle, distance, quality) in scan_data:
        if distance == 0:
            scan_msg.ranges.append(float('Inf'))
        else:
            scan_msg.ranges.append(distance / 1000.0)
        scan_msg.intensities.append(quality)

    pub.publish(scan_msg)

def stop_motor(req): #LIDAR 모터를 중지하는 서비스 콜백 함수
    lidar.stop()
    lidar.set_motor_pwm(0)
    return EmptyResponse()

def start_motor(req): #LIDAR 모터를 시작하는 서비스 콜백 함수입니다
    lidar.set_motor_pwm(660)
    lidar.start_scan()
    return EmptyResponse()

def main():
    rospy.init_node('rplidar_node')
    
    frame_id = rospy.get_param('~frame_id', 'laser_frame')
    scan_pub = rospy.Publisher('/lidar', LaserScan, queue_size=1000)

    rospy.Service('stop_motor', Empty, stop_motor) #stop_motor 서비스를 정의합니다.
    rospy.Service('start_motor', Empty, start_motor) #start_motor 서비스를 정의합니다.

    lidar.set_motor_pwm(660) #LIDAR 모터를 시작합니다.
    lidar.start_scan() #LIDAR 스캔을 시작합니다.

    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        scan_data = [] #스캔 데이터를 초기화합니다.
        for scan in lidar.iter_scans(): #하나씩 담아서 저장함.
            scan_data = scan
            break

        publish_scan(scan_pub, scan_data, frame_id)
        rate.sleep()

    lidar.stop() #LIDAR 스캔을 중지합니다.
    lidar.set_motor_pwm(0) #모터를 중지합니다.
    lidar.disconnect() #LIDAR 연결을 해제합니다.

if __name__ == '__main__':
    main()

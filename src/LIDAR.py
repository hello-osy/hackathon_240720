#https://github.com/slamtec/rplidar_ros 여기서 찾으면 됨.
#어제는 roslaunch rplidar_ros view_rplidar_c1.launch 했더니 라이다 창 뜬 것이었음.

#github에 올라온 것 중에서 src/node.cpp 를 파이썬으로 변환한 것임.

import rplidar
import rospy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty, EmptyResponse

# LIDAR 드라이버 초기화
lidar = rplidar.RPLidar('/dev/ttyUSB0')

def publish_scan(pub, scan_data, frame_id):
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

def stop_motor(req):
    lidar.stop()
    lidar.set_motor_pwm(0)
    return EmptyResponse()

def start_motor(req):
    lidar.set_motor_pwm(660)
    lidar.start_scan()
    return EmptyResponse()

def main():
    rospy.init_node('rplidar_node')
    
    frame_id = rospy.get_param('~frame_id', 'laser_frame')
    scan_pub = rospy.Publisher('/lidar', LaserScan, queue_size=1000)

    rospy.Service('stop_motor', Empty, stop_motor)
    rospy.Service('start_motor', Empty, start_motor)

    lidar.set_motor_pwm(660)
    lidar.start_scan()

    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        scan_data = []
        for scan in lidar.iter_scans():
            scan_data = scan
            break

        publish_scan(scan_pub, scan_data, frame_id)
        rate.sleep()

    lidar.stop()
    lidar.set_motor_pwm(0)
    lidar.disconnect()

if __name__ == '__main__':
    main()

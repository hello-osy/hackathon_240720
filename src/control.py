#https://github.com/hello-osy/project2/tree/dev_osy/src/control/src/control.py 에서 수정함.
#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Float32MultiArray
from jetracer.nvidia_racecar import NvidiaRacecar
import control_library
import math
import signal

car = NvidiaRacecar()
#car.steering = ?? / car.throttle = ?? 이렇게 하면 조향이랑 속도 바꿀 수 있음. 굳이 publish 안 해도 됨.

theta = 0
WIDTH = 800 #카메라 화소에 맞게 수정해야 함.
LEGTH = 600

def imu_callback():
    pass

def lidar_callback():
    pass

def distance_callback():
    pass

#########################################


def matching(x, input_min, input_max, output_min, output_max):
    return (x - input_min) * (output_max - output_min) / (input_max - input_min) + output_min # map() 함수 정의.

def steering_theta(self, w1, w2): # 차선 기울기 기준 조향
    if np.abs(w1) > np.abs(w2):  # 우회전
        if w1 * w2 < 0:  # 정방향 or 약간 틀어진 방향
            w1 = -w1
            angle = np.arctan(np.abs(math.tan(w1) - math.tan(w2)) / (1 + math.tan(w1) * math.tan(w2)))
            theta = self.matching(angle, 0, np.pi / 2, 0, 10)
        elif w1 * w2 > 0:  # 극한으로 틀어진 방향
            if w1 > w2:
                theta = 0
            else:
                theta = 0
        else:
            theta = 0
    elif np.abs(w1) < np.abs(w2):  # 좌회전
        if w1 * w2 < 0:  # 정방향 or 약간 틀어진 방향
            w1 = -w1
            angle = np.arctan(np.abs(math.tan(w1) - math.tan(w2)) / (1 + math.tan(w1) * math.tan(w2)))
            theta = self.matching(angle, 0, np.pi / 2, 0, -10)
        elif w1 * w2 > 0:  # 극한으로 틀어진 방향
            if w1 > w2:
                theta = 0
            else:
                theta = 0
        else:
            theta = 0
    else:
        theta = 0

    return theta

def steering_vanishing_point(x):
    standard_x = int(WIDTH / 2)
    diff = standard_x - x 
    if diff > 0:   # 좌회전
        theta = matching(diff, 0, -WIDTH / 2, 0, -10)
    elif diff < 0:
        theta = matching(diff, 0, WIDTH / 2, 0, 10)
    else:
        theta = 0

    return theta

def steering_calcu(input_data): # 조향값 도출
    x1, y1, x2, y2, x3, y3, x4, y4 = input_data

    # 분모가 0인 경우를 예외 처리
    if x1 == x2 and x3 == x4:
        return None  # 두 점이 같은 x 좌표를 가지면 기울기가 무한대가 되므로 None을 반환
    else:
        # 기울기 계산
        left_calculated_weight = (y1 - y2) / (x2 - x1)
        right_calculated_weight = (y3 - y4) / (x4 - x3)

        # 절편 계산
        left_calculated_bias = y1 - left_calculated_weight * x1
        right_calculated_bias = y3 - right_calculated_weight * x3

        cross_x = (left_calculated_bias - right_calculated_bias) / (right_calculated_weight - left_calculated_weight)
        cross_y = left_calculated_weight * ((left_calculated_bias - right_calculated_bias) / (right_calculated_weight - left_calculated_weight)) + right_calculated_bias
        
        if not np.isnan(cross_x) and not np.isnan(cross_y):
            if -5 < steering_theta(left_calculated_weight, right_calculated_weight) < 5:
                print('소실점 조향 서보모터 각도: ', steering_vanishing_point(cross_x))
                steering_angle = steering_vanishing_point(cross_x)
            else:
                print("기울기 조향 서보모터 각도: ", steering_theta(left_calculated_weight, right_calculated_weight))
                steering_angle = steering_theta(left_calculated_weight, right_calculated_weight)
        
    return steering_angle

def lane_callback(msg): #경로 생성, 경로 추종

    #msg.data는 [right_x1, y1, right_x2, y2, left_x1, y1, left_x2, y2] 

    #car.steering = ??
    #car.throttle = ??
    dt = 0.001

    theta = steering_calcu(msg.data)

    car.steering = theta #단위 check
    car.throttle = 1
    

def main():
    rospy.init_node('control', anonymous=True)

    #토픽 이름, 메시지 타입, 콜백 함수
    #콜백함수는 다시 설정할 것
    rospy.Subscriber('/imu', String, imu_callback)
    rospy.Subscriber('/lidar', LaserScan, lidar_callback)
    rospy.Subscriber('/distance', Int32, distance_callback)
    rospy.Subscriber('/lane_detector', Float32MultiArray, lane_callback) #차선 정보를 점 4개로 표현한 정보를 받음.

    # Ctrl+C 누르면 종료할 수 있게 만듦.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.spin()  # 콜백 함수를 호출하면서 계속 실행

def signal_handler(sig, frame):
    rospy.signal_shutdown('Shutting down')

if __name__ == '__main__': #스크립트가 직접 실행될 때 main() 함수를 호출하여 프로그램을 시작합니다.
    main()
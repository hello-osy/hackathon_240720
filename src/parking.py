import rospy
import signal
from std_msgs.msg import Int32MultiArray, String
from jetracer.nvidia_racecar import NvidiaRacecar

nvidiaracecar = NvidiaRacecar()

parking_detected = False
parking_mode = False

def distance_callback(msg):
    ultra_1_distance, ultra_2_distance, ultra_3_distance = msg.data #초음파 센서 3개 값. 앞 왼 뒤 순서인 듯? 아니면 바꿀 것.
    if parking_detected and ultra_2_distance < 50:
        parking_mode = True
    
    if parking_mode:
        if ultra_2_distance>=50:
            nvidiaracecar.throttle=0
            nvidiaracecar.steering=0
            rospy.sleep(1)
            nvidiaracecar.throttle=-0.3
            nvidiaracecar.steering=0.5
            rospy.sleep(1)
            nvidiaracecar.throttle=-0.3
            nvidiaracecar.steering=-0.3
        
    
'''
nc: 8
names: ['car', 'children', 'children_cross', 'green', 'parking', 'red', 'slow', 'stop_line']
'''
        

def detection_callback(msg):
    msg_length = len(msg.data)
    for i in msg_length/3 - 1 :
        cls_id, width, height = msg.data[3*i], msg.data[3*i+1], msg.data[3*i+2]
        if cls_id == 4: #주차 표지판 인식했을 때
            parking_detected = True
            break


def main():
    rospy.init_node('parking')
    rospy.Subscriber('/distance', Int32MultiArray, distance_callback)
    rospy.Subscriber('/detected_objects', Int32MultiArray, detection_callback)

    # Ctrl+C 누르면 종료할 수 있게 만듦.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.spin()  # 콜백 함수를 호출하면서 계속 실행

def signal_handler(sig, frame):
    rospy.signal_shutdown('Shutting down')

if __name__ == '__main__': #스크립트가 직접 실행될 때 main() 함수를 호출하여 프로그램을 시작합니다.
    main()
#작동전압 5V

'''
#여기는 아두이노 코드임. 파이썬 파일이 아님. 분리해야 함.

int TrigPin1 = 2;
int EchoPin1 = 3;
int TrigPin2 = 4;
int EchoPin2 = 5;
int TrigPin3 = 6;
int EchoPin3 = 7;

void setup()
{
    Serial.begin(9600);    // 통신 속도
    pinMode(TrigPin1, OUTPUT);  // TrigPin 출력
    pinMode(EchoPin1, INPUT);   // EchoPin 입력
    pinMode(TrigPin2, OUTPUT);  // TrigPin 출력
    pinMode(EchoPin2, INPUT);   // EchoPin 입력
    pinMode(TrigPin3, OUTPUT);  // TrigPin 출력
    pinMode(EchoPin3, INPUT);   // EchoPin 입력
}

void loop()
{
    digitalWrite(TrigPin1, HIGH);     // TrigPin HIGH
    delayMicroseconds(10);           // 10us 지연
    digitalWrite(TrigPin1, LOW);      // TrigPin LOW
    int distance1 = pulseIn(EchoPin1, HIGH) * 0.017;  // cm로 변환

    digitalWrite(TrigPin2, HIGH);     // TrigPin HIGH
    delayMicroseconds(10);           // 10us 지연
    digitalWrite(TrigPin2, LOW);      // TrigPin LOW
    int distance2 = pulseIn(EchoPin2, HIGH) * 0.017;  // cm로 변환

    digitalWrite(TrigPin3, HIGH);     // TrigPin HIGH
    delayMicroseconds(10);           // 10us 지연
    digitalWrite(TrigPin3, LOW);      // TrigPin LOW
    int distance3 = pulseIn(EchoPin3, HIGH) * 0.017;  // cm로 변환
    
    Serial.print(distance1);  // distance1를 시리얼에 출력
    Serial.print(" ");
    Serial.print(distance2);  // distance2를 시리얼에 출력
    Serial.print(" ");
    Serial.print(distance3);  // distance3를 시리얼에 출력

    delay(1000);  // 1초 지연
}

'''

#sudo apt-get install ros-melodic-rosserial-python 리눅스 터미널

#여기부터는 아두이노가 보낸 메시지 받는 파이썬 코드임

#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray
import serial

def talker():
    # Initialize serial connection to Arduino
    ser = serial.Serial('/dev/ttyUSB0', 9600)  # 포트를 수정해야할 수도 있음. COM3이나 /dev/ttyUSB0 혹은 다른 것으로
    pub = rospy.Publisher('/distance', Int32MultiArray, queue_size=10)  # /distance 토픽 발행 퍼블리셔 정의
    rospy.init_node('arduino_listener', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        try:
            if ser.in_waiting:
                distance_str = ser.readline().decode('utf-8').strip()
                distances = [int(x) for x in distance_str.split(' ')]
                if len(distances) == 3:
                    msg = Int32MultiArray(data=distances)
                    rospy.loginfo(f"Distances: {distances}")
                    pub.publish(msg)
        except ValueError:
            rospy.logwarn("Failed to convert serial data to integer array.")
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

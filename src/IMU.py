import sys
import serial
import rospy
from std_msgs.msg import String


def main():
    # Initialize the ROS node
    rospy.init_node('imu_publisher', anonymous=True)

    # Create a publisher for the /imu topic
    imu_pub = rospy.Publisher('/imu', String, queue_size=10)

    # Get serial port information from the user
    comport_num = input("EBIMU Port (e.g., COM3 or /dev/ttyUSB0): ")
    comport_baudrate = int(input("Baudrate (e.g., 115200): "))

    # Set up serial connection
    ser = serial.Serial(port=comport_num, baudrate=comport_baudrate)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            line = ser.readline()
            line = line.decode('utf-8').strip()
            words = line.split(",")  # Fields split

            if '*' in words[0]:
                words[0] = words[0].replace('*', '')

            imu_data = ' '.join(words)
            
            # Publish the IMU data
            imu_pub.publish(imu_data)

            rospy.loginfo(f"Published IMU data: {imu_data}")

        rate.sleep()

    ser.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

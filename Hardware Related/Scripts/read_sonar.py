import rospy
from sensor_msgs.msg import PointCloud
import math

SONAR_NUM = 8
offset = [160, 220, 240, 240, 240, 240, 220, 160]
distToObstacle = [0] * SONAR_NUM

def get_sonarData_Callback(sonarScanedData):
    seq = sonarScanedData.header.seq
    print("seq of sonar beam and distance measured -->")
    print("Frame[{}]: ".format(seq))
    for i in range(SONAR_NUM):
        print("{}\t".format(i), end='')
    print()

    for i in range(SONAR_NUM):
        tmpX = sonarScanedData.points[i].x
        tmpY = sonarScanedData.points[i].y
        distToObstacle[i] = int(math.sqrt(tmpX*tmpX + tmpY*tmpY) * 1000 - offset[i])
        print("{}\t".format(distToObstacle[i]), end='')
    print("\n")

def main():
    rospy.init_node("p3_sonar")
    rospy.Subscriber("/RosAria/sonar", PointCloud, get_sonarData_Callback)
    print("\n********** Sonar Readings: **********\n")

    while not rospy.is_shutdown():
        rospy.Duration(0.2)
        rospy.spin()

if __name__ == "__main__":
    main()
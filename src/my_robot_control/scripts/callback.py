#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range

class SonarScanner:
    def __init__(self):
        rospy.init_node('sonar_scanner')
        self.regions = {
            'right': 0,
            'fright': 0,
            'front_right': 0,
            'front': 0,
            'front_left': 0,
            'fleft': 0,
            'left': 0,
            'fleft_2': 0
        }
        self.scan_subs = [
            rospy.Subscriber('/sonar_scan', Range, self.scan_callback),
            rospy.Subscriber('/sonar_scan_1', Range, self.scan_callback_1),
            rospy.Subscriber('/sonar_scan_2', Range, self.scan_callback_2),
            rospy.Subscriber('/sonar_scan_3', Range, self.scan_callback_3),
            rospy.Subscriber('/sonar_scan_4', Range, self.scan_callback_4),
            rospy.Subscriber('/sonar_scan_5', Range, self.scan_callback_5),
            rospy.Subscriber('/sonar_scan_6', Range, self.scan_callback_6),
            rospy.Subscriber('/sonar_scan_7', Range, self.scan_callback_7)
        ]

    def update_region(self, region, value):
        self.regions[region] = min(value, 10)

    def scan_callback(self, data):
        self.update_region('right', data.range)
        self.print_regions()

    def scan_callback_1(self, data):
        self.update_region('fright', data.range)
        self.print_regions()

    def scan_callback_2(self, data):
        self.update_region('front_right', data.range)
        self.print_regions()

    def scan_callback_3(self, data):
        self.update_region('front', data.range)
        self.print_regions()

    def scan_callback_4(self, data):
        self.update_region('front_left', data.range)
        self.print_regions()

    def scan_callback_5(self, data):
        self.update_region('fleft', data.range)
        self.print_regions()

    def scan_callback_6(self, data):
        self.update_region('left', data.range)
        self.print_regions()

    def scan_callback_7(self, data):
        self.update_region('fleft_2', data.range)
        self.print_regions()

    def print_regions(self):
        rospy.loginfo("Sonar Regions: {}".format(self.regions))
        for region, value in self.regions.items():
            rospy.loginfo("Region: {}, Value: {}".format(region, value))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        scanner = SonarScanner()
        scanner.run()
    except rospy.ROSInterruptException:
        pass

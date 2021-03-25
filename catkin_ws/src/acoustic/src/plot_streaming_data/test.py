#!/usr/bin/env python3
import rospy
def main():
	rospy.init_node("test_parameter", anonymous=True)
	fps = rospy.get_param("~FPS_")
	print(fps)
if __name__ == "__main__":
	main()

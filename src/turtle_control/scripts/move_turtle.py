#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def mover_tortuga():
    rospy.init_node('mover_tortuga', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('/t2/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    cmd1 = Twist()
    cmd2 = Twist()

    while not rospy.is_shutdown():
        # ==== 1. Movimiento de turtle1: avanzar
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 5 and not rospy.is_shutdown():
            cmd1.linear.x = 2.0
            cmd1.angular.z = 0.0
            cmd2.linear.x = 0.0
            cmd2.angular.z = 2.0  # turtle2 gira primero

            pub.publish(cmd1)
            pub2.publish(cmd2)
            rate.sleep()

        # ==== 2. Movimiento de turtle1: girar / turtle2: avanzar
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 5 and not rospy.is_shutdown():
            cmd1.linear.x = 0.0
            cmd1.angular.z = 2.0
            cmd2.linear.x = 2.0
            cmd2.angular.z = 0.0

            pub.publish(cmd1)
            pub2.publish(cmd2)
            rate.sleep()

        # ==== 3. Detener ambas durante 1 segundo
        cmd1 = Twist()
        cmd2 = Twist()
        pub.publish(cmd1)
        pub2.publish(cmd2)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        mover_tortuga()
    except rospy.ROSInterruptException:
        pass

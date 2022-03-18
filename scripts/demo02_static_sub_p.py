#! /usr/bin/env/python

import rospy
import tf2_ros
from tf2_geometry_msgs import PointStamped

"""
    订阅方：订阅坐标变换消息， 传入被转换的坐标点， 调用转换算法
    流程：
        1.导包
        2.初始化
        3.创建订阅对象
        4.组织被转化的坐标点
        5.转换逻辑实现，调用tf封装的算法
        6.输出结果
        7.spin() | spinOnce()
"""

if __name__ == "__main__":    
    # 2.初始化
    rospy.init_node("static_sub_p")
    # 3.创建订阅对象
    # 3-1.创建缓存对象
    buffer = tf2_ros.Buffer()
    # 3-2.创建订阅对象（将缓存对象传入）
    sub = tf2_ros.TransformListener(buffer)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    # 4.组织被转化的坐标点
        ps = PointStamped()
        ps.header.frame_id = "laser"
        ps.header.stamp = rospy.Time.now()     
        ps.point.x = 2.0
        ps.point.y = 3.0
        ps.point.z = 5.0
        try:
            # 5.转换逻辑实现，调用tf封装的算法
            # 转换实现
            ps_out = buffer.transform(ps,"base_link")
            # 6.输出结果
            rospy.loginfo("转换后的坐标：(%.2f,%.2f,%.2f),参考的坐标系：%s",
                        ps_out.point.x,
                        ps_out.point.y,
                        ps_out.point.z,
                        ps_out.header.frame_id)
        except Exception as e:
            rospy.logwarn("错误提示：%s",e)
        
    # 7.spin() | spinOnce()
        rate.sleep()
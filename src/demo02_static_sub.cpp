#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

/*
    订阅方：订阅发布的坐标系相对关系，传入一个坐标点，调用 tf 实现转换

    流程 ：
        1.包含头文件
        2.编码， 初始化， Nodeandle（必须的）
        3.创建订阅对象； ---> 订阅坐标系相对关系
        4.组织一个坐标点数据
        5.转换算法，需要调用TF内置实现
        6.最后输出
*/

int main(int argc, char *argv[])
{
    // 2.编码， 初始化， Nodeandle（必须的）
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "static_sub");
    ros::NodeHandle nh;
    // 3.创建订阅对象； ---> 订阅坐标系相对关系
    // 3-1.创建订阅对象； ---> 订阅坐标系相对关系
    tf2_ros::Buffer buffer;
    // 3-2.创建监听对象（监听对象可以将订阅的数据存入buffer）
    tf2_ros::TransformListener listener(buffer);
    // 4.组织一个坐标点数据
    geometry_msgs::PointStamped ps;
    ps.header.frame_id = "laser";
    ps.header.stamp = ros::Time::now();
    ps.point.x = 2.0;
    ps.point.y = 3.0;
    ps.point.z = 5.0;
    //添加休眠
    ros::Duration(2).sleep();
    // 5.转换算法，需要调用TF内置实现
    ros::Rate rate(10);
    while (ros::ok())
    {
        //核心代码 ---- 将 ps 转换成相对于 base_link 的坐标点
        geometry_msgs::PointStamped ps_out;
        ps_out = buffer.transform(ps,"base_link");
        // 6.最后输出
        ROS_INFO("转换后的坐标值：(%.2f,%.2f,%.2f),参考的坐标系:%s",
                    ps_out.point.x,
                    ps_out.point.y,
                    ps_out.point.z,
                    ps_out.header.frame_id.c_str()
                    );
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

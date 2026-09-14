#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS Noetic turtlesim HUTB drawing demo.

功能：
1. 订阅 /turtle1/pose 获取海龟实时位置；
2. 发布 /turtle1/cmd_vel 控制运动；
3. 使用 /turtle1/set_pen 控制画笔；
4. 使用 /turtle1/teleport_absolute 在不同笔画之间移动；
5. 自动绘制 H、U、T、B 四个字母。
"""

import math

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, TeleportAbsolute
from std_srvs.srv import Empty


# =========================
# 全局变量
# =========================

current_pose = None

cmd_pub = None
set_pen_client = None
teleport_client = None
clear_client = None


# =========================
# 基础工具函数
# =========================

def pose_callback(msg):
    """
    接收 turtlesim 发布的实时位姿。
    """
    global current_pose
    current_pose = msg


def normalize_angle(angle):
    """
    把角度限制到 [-pi, pi]。
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def stop_turtle():
    """
    发布全零 Twist，使海龟停止。
    """
    if cmd_pub is not None:
        cmd_pub.publish(Twist())


def shutdown_handler():
    """
    ROS 节点退出时保证海龟停止。
    """
    stop_turtle()


def set_pen(off, r=20, g=80, b=220, width=4):
    """
    设置画笔。

    off=True:
        关闭画笔，移动时不留下轨迹。

    off=False:
        打开画笔。

    r/g/b:
        轨迹颜色。

    width:
        轨迹宽度。
    """
    try:
        set_pen_client(
            r,
            g,
            b,
            width,
            1 if off else 0
        )
        rospy.sleep(0.05)

    except rospy.ServiceException as error:
        rospy.logerr("设置画笔失败: %s", error)


def teleport_to(x, y, theta=0.0):
    """
    把海龟瞬移到指定位置。
    瞬移前应该关闭画笔。
    """
    try:
        teleport_client(x, y, theta)
        rospy.sleep(0.15)

    except rospy.ServiceException as error:
        rospy.logerr("瞬移失败: %s", error)


# =========================
# 闭环运动控制
# =========================

def move_to_point(target_x, target_y, tolerance=0.035):
    """
    使用 /turtle1/pose 的反馈控制海龟移动到目标点。

    不是单纯根据时间控制，而是实时计算：

    目标方向 = atan2(dy, dx)
    角度误差 = 目标方向 - 当前朝向
    距离误差 = sqrt(dx^2 + dy^2)

    当方向误差比较大时先转向；
    方向基本正确后再向前移动。
    """

    rate = rospy.Rate(40)

    start_time = rospy.Time.now()

    while not rospy.is_shutdown():

        if current_pose is None:
            rate.sleep()
            continue

        dx = target_x - current_pose.x
        dy = target_y - current_pose.y

        distance = math.hypot(dx, dy)

        # 已经到目标点
        if distance < tolerance:
            break

        target_angle = math.atan2(dy, dx)

        angle_error = normalize_angle(
            target_angle - current_pose.theta
        )

        cmd = Twist()

        # -------------------------
        # 朝向偏差较大：先旋转
        # -------------------------
        if abs(angle_error) > 0.22:

            cmd.linear.x = 0.0

            cmd.angular.z = 4.0 * angle_error

            # 限制最大角速度
            cmd.angular.z = max(
                -2.6,
                min(2.6, cmd.angular.z)
            )

        # -------------------------
        # 朝向基本正确：向目标移动
        # -------------------------
        else:

            cmd.linear.x = 1.8 * distance

            # 限制最大线速度
            cmd.linear.x = max(
                0.15,
                min(1.8, cmd.linear.x)
            )

            cmd.angular.z = 4.0 * angle_error

            cmd.angular.z = max(
                -2.0,
                min(2.0, cmd.angular.z)
            )

        cmd_pub.publish(cmd)

        # 防止异常情况下无限循环
        elapsed = (
            rospy.Time.now() - start_time
        ).to_sec()

        if elapsed > 15.0:
            rospy.logwarn(
                "移动到 (%.2f, %.2f) 超时",
                target_x,
                target_y
            )
            break

        rate.sleep()

    stop_turtle()
    rospy.sleep(0.08)


# =========================
# 绘图函数
# =========================

def draw_polyline(points, color=(20, 80, 220), width=4):
    """
    绘制一组连续折线。

    points 例如：

    [
        (1.0, 2.0),
        (1.0, 8.0),
        (2.0, 8.0)
    ]

    会按照这些点依次连线。
    """

    if len(points) < 2:
        return

    # 获取起点
    start_x, start_y = points[0]

    # 计算初始方向
    next_x, next_y = points[1]

    heading = math.atan2(
        next_y - start_y,
        next_x - start_x
    )

    # -------------------------
    # 先关笔
    # -------------------------

    set_pen(True)

    # -------------------------
    # 瞬移到这一笔的起点
    # -------------------------

    teleport_to(
        start_x,
        start_y,
        heading
    )

    # -------------------------
    # 开笔
    # -------------------------

    r, g, b = color

    set_pen(
        False,
        r=r,
        g=g,
        b=b,
        width=width
    )

    # -------------------------
    # 依次绘制
    # -------------------------

    for x, y in points[1:]:

        if rospy.is_shutdown():
            return

        move_to_point(x, y)

    stop_turtle()

    # 这一笔画完之后关闭画笔
    set_pen(True)

    rospy.sleep(0.12)


# =========================
# H
# =========================

def draw_h():
    """
    绘制字母 H。
    """

    rospy.loginfo("正在绘制 H")

    color = (30, 90, 230)

    # 左竖线
    draw_polyline(
        [
            (0.8, 2.5),
            (0.8, 8.5)
        ],
        color
    )

    # 右竖线
    draw_polyline(
        [
            (2.1, 2.5),
            (2.1, 8.5)
        ],
        color
    )

    # 中间横线
    draw_polyline(
        [
            (0.8, 5.5),
            (2.1, 5.5)
        ],
        color
    )


# =========================
# U
# =========================

def draw_u():
    """
    绘制字母 U。
    """

    rospy.loginfo("正在绘制 U")

    color = (20, 170, 80)

    draw_polyline(
        [
            (2.9, 8.5),

            (2.9, 4.0),
            (2.95, 3.5),
            (3.1, 3.1),
            (3.35, 2.8),
            (3.65, 2.6),

            (3.95, 2.6),
            (4.25, 2.8),
            (4.50, 3.1),
            (4.65, 3.5),
            (4.70, 4.0),

            (4.70, 8.5)
        ],
        color
    )


# =========================
# T
# =========================

def draw_t():
    """
    绘制字母 T。
    """

    rospy.loginfo("正在绘制 T")

    color = (230, 130, 20)

    # 顶部横线
    draw_polyline(
        [
            (5.2, 8.5),
            (7.0, 8.5)
        ],
        color
    )

    # 中间竖线
    draw_polyline(
        [
            (6.1, 8.5),
            (6.1, 2.5)
        ],
        color
    )


# =========================
# B
# =========================

def draw_b():
    """
    绘制字母 B。

    turtlesim 本身走的是连续运动，
    这里使用多个小线段近似 B 的弧线。
    """

    rospy.loginfo("正在绘制 B")

    color = (210, 40, 60)

    # B 左侧竖线
    draw_polyline(
        [
            (7.5, 2.5),
            (7.5, 8.5)
        ],
        color
    )

    # B 上半圆
    draw_polyline(
        [
            (7.5, 8.5),

            (8.4, 8.5),
            (8.8, 8.45),
            (9.15, 8.25),
            (9.40, 7.95),
            (9.55, 7.60),

            (9.58, 7.20),

            (9.45, 6.80),
            (9.20, 6.45),
            (8.85, 6.20),
            (8.40, 6.10),

            (7.5, 6.10)
        ],
        color
    )

    # B 下半圆
    draw_polyline(
        [
            (7.5, 6.10),

            (8.45, 6.10),
            (8.90, 6.00),
            (9.25, 5.75),
            (9.50, 5.40),

            (9.65, 4.95),
            (9.65, 4.45),

            (9.50, 3.95),
            (9.25, 3.55),
            (8.90, 3.20),
            (8.45, 2.95),

            (7.5, 2.95)
        ],
        color
    )


# =========================
# 主函数
# =========================

def main():

    global cmd_pub
    global set_pen_client
    global teleport_client
    global clear_client

    # -------------------------
    # 初始化 ROS 节点
    # -------------------------

    rospy.init_node('draw_hutb_node')

    rospy.on_shutdown(shutdown_handler)

    # -------------------------
    # 发布速度
    # -------------------------

    cmd_pub = rospy.Publisher(
        '/turtle1/cmd_vel',
        Twist,
        queue_size=10
    )

    # -------------------------
    # 订阅实时位姿
    # -------------------------

    rospy.Subscriber(
        '/turtle1/pose',
        Pose,
        pose_callback
    )

    # -------------------------
    # 等待 turtlesim 服务
    # -------------------------

    rospy.loginfo("等待 turtlesim 服务...")

    rospy.wait_for_service('/turtle1/set_pen')

    rospy.wait_for_service(
        '/turtle1/teleport_absolute'
    )

    rospy.wait_for_service('/clear')

    # 创建服务客户端
    set_pen_client = rospy.ServiceProxy(
        '/turtle1/set_pen',
        SetPen
    )

    teleport_client = rospy.ServiceProxy(
        '/turtle1/teleport_absolute',
        TeleportAbsolute
    )

    clear_client = rospy.ServiceProxy(
        '/clear',
        Empty
    )

    # -------------------------
    # 等待第一次 pose
    # -------------------------

    rospy.loginfo("等待 /turtle1/pose ...")

    while (
        not rospy.is_shutdown()
        and current_pose is None
    ):
        rospy.sleep(0.1)

    rospy.sleep(0.5)

    # -------------------------
    # 清空画布
    # -------------------------

    try:
        clear_client()
    except rospy.ServiceException as error:
        rospy.logerr(
            "清空画布失败: %s",
            error
        )

    # -------------------------
    # 绘制 HUTB
    # -------------------------

    rospy.loginfo("==========================")
    rospy.loginfo("开始绘制 HUTB")
    rospy.loginfo("==========================")

    draw_h()

    rospy.loginfo("H 完成")

    draw_u()

    rospy.loginfo("U 完成")

    draw_t()

    rospy.loginfo("T 完成")

    draw_b()

    rospy.loginfo("B 完成")

    # -------------------------
    # 最终停止
    # -------------------------

    set_pen(True)

    stop_turtle()

    # 把海龟移动到右下角，
    # 避免挡住最终绘图
    teleport_to(
        10.3,
        1.0,
        math.pi
    )

    rospy.loginfo("==========================")
    rospy.loginfo("HUTB 绘制完成！")
    rospy.loginfo("==========================")

    rospy.sleep(1.0)


if __name__ == '__main__':

    try:
        main()

    except rospy.ROSInterruptException:
        pass

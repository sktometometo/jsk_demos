#!/usr/bin/env python

import math

import cv2
import numpy as np
import rospy
import scipy
import sensor_msgs.point_cloud2 as pc2
import tf2_geometry_msgs
import tf2_ros
import tf2_sensor_msgs
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, PolygonStamped
from scipy.spatial import ConvexHull
from sensor_msgs.msg import Image, PointCloud2


class PointCloudToImage:

    def __init__(self):
        rospy.init_node("pointcloud_to_image", anonymous=True)

        self.bridge = CvBridge()

        self.center_frame_id: str = "body"
        self.fixed_frame_id: str = "odom"
        self.dimension_x: float = 20.0  # meter
        self.dimension_y: float = 20.0  # meter
        self.resolusion: float = 0.01  # meter/pixel

        self.minimum_length: float = 3.0  # meter

        self.image_width = int(self.dimension_x / self.resolusion)
        self.image_height = int(self.dimension_y / self.resolusion)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pc_sub = rospy.Subscriber(
            "/spot/lidar/points/throttled",
            PointCloud2,
            self.callback,
        )
        self.image_pub = rospy.Publisher(
            "/room_polygon",
            PolygonStamped,
            queue_size=1,
        )

    def callback(self, data: PointCloud2):
        try:
            # base_linkフレームに変換
            transform_pc_to_center = self.tf_buffer.lookup_transform(
                self.center_frame_id,
                data.header.frame_id,
                data.header.stamp,
                rospy.Duration(1.0),
            )
            transform_center_to_odom = self.tf_buffer.lookup_transform(
                self.fixed_frame_id,
                self.center_frame_id,
                data.header.stamp,
                rospy.Duration(1.0),
            )
            transformed_cloud = tf2_sensor_msgs.do_transform_cloud(
                data,
                transform_pc_to_center,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logerr("TF2 transform error: %s", e)
            return

        pc = pc2.read_points(
            transformed_cloud,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )

        points = np.array(list(pc))

        # 点群のxとy座標を使用して画像を生成
        img = np.zeros((self.image_height, self.image_width, 3), np.uint8)

        # 白色で点を描画
        for point in points:
            x = int((point[0] + self.dimension_x / 2) / self.resolusion)
            y = int((point[1] + self.dimension_y / 2) / self.resolusion)
            if 0 <= x < self.image_width and 0 <= y < self.image_height:
                img[y, x] = (255, 255, 255)

        # Hough直線検出
        # 画像をグレースケールに変換
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # 2値化
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)
        # 輪郭検出 (緩め)
        lines = cv2.HoughLinesP(
            binary,
            rho=1,
            theta=np.pi / 180,
            threshold=10,
            minLineLength=self.minimum_length / self.resolusion,
            maxLineGap=1.0 / self.resolusion,
        )

        if lines is None:
            return

        # すべての直線を含有し頂点を外接する凸包を計算
        points = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            points.append([x1, y1])
            points.append([x2, y2])
        try:
            hull = ConvexHull(points)
        except scipy.spatial.qhull.QhullError:
            return

        # 凸包を描画
        hull_lines = []
        for simplex in hull.simplices:
            hull_lines.append(
                [
                    (points[simplex[0]][0], points[simplex[0]][1]),
                    (points[simplex[1]][0], points[simplex[1]][1]),
                ]
            )

        # 画像を表示
        # cv2.line(
        #         img,
        #         (points[simplex[0]][0], points[simplex[0]][1]),
        #         (points[simplex[1]][0], points[simplex[1]][1]),
        #         (0, 255, 0),
        #         2,
        #     )
        # 1270x720 以下にresize
        # scale = min(1270 / img.shape[1], 720 / img.shape[0])
        # img = cv2.resize(img, None, fx=scale, fy=scale)
        # cv2.imshow("pointcloud_image", img)
        # cv2.waitKey(1)

        # Polygon の publish
        polygon = PolygonStamped()
        polygon.header.frame_id = self.fixed_frame_id
        for line in hull_lines:
            point1 = line[0]
            point2 = line[1]
            msg_point1_transformed = tf2_geometry_msgs.do_transform_point(
                PointStamped(
                    point=Point(
                        x=point1[0] * self.resolusion - self.dimension_x / 2,
                        y=point1[1] * self.resolusion - self.dimension_y / 2,
                    )
                ),
                transform_center_to_odom,
            )
            msg_point2_transformed = tf2_geometry_msgs.do_transform_point(
                PointStamped(
                    point=Point(
                        x=point2[0] * self.resolusion - self.dimension_x / 2,
                        y=point2[1] * self.resolusion - self.dimension_y / 2,
                    )
                ),
                transform_center_to_odom,
            )
            polygon.polygon.points.append(msg_point1_transformed.point)
            polygon.polygon.points.append(msg_point2_transformed.point)
        self.image_pub.publish(polygon)


if __name__ == "__main__":
    try:
        pc2img = PointCloudToImage()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        pass

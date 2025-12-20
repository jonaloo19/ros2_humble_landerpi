#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/03/11
# @author:aiden
# lane detection for autonomous driving
import os
import cv2
import math
import queue
import threading
import numpy as np
import sdk.common as common
from cv_bridge import CvBridge

bridge = CvBridge()

lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")

class LaneDetector(object):
    def __init__(self, color):
        # lane color
        self.target_color = color
        # ROI for lane detection
        if os.environ['DEPTH_CAMERA_TYPE'] == 'ascamera':
            self.rois = ((338, 360, 0, 320, 0.7), (292, 315, 0, 320, 0.2), (248, 270, 0, 320, 0.1))
        elif os.environ['DEPTH_CAMERA_TYPE'] == 'aurora':
            self.rois = ((348, 370, 0, 320, 0.7), (302, 325, 0, 320, 0.2), (258, 280, 0, 320, 0.1))
        else:
            self.rois = ((450, 480, 0, 320, 0.7), (390, 480, 0, 320, 0.2), (330, 480, 0, 320, 0.1))
        self.weight_sum = 1.0

    def set_roi(self, roi):
        self.rois = roi

    @staticmethod
    def get_area_max_contour(contours, threshold=100):
        '''
        obtain the contour corresponding to the maximum area
        :param contours:
        :param threshold:
        :return:
        '''
        contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
        contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
        if len(contour_area) > 0:
            max_c_a = max(contour_area, key=lambda c_a: c_a[1])
            return max_c_a
        return None
    
    def add_horizontal_line(self, image):
        #   |____  --->   |————   ---> ——
        h, w = image.shape[:2]
        roi_w_min = int(w/2)
        roi_w_max = w
        roi_h_min = 0
        roi_h_max = h
        roi = image[roi_h_min:roi_h_max, roi_w_min:roi_w_max]  # crop the right half
        flip_binary = cv2.flip(roi, 0)  # flip upside down
        max_y = cv2.minMaxLoc(flip_binary)[-1][1]  # extract the coordinates of the top-left point with a value of 255

        return h - max_y

    def add_vertical_line_far(self, image):
        h, w = image.shape[:2]
        roi_w_min = int(w/7)
        roi_w_max = int(w/2)
        roi_h_min = 0
        roi_h_max = h
        roi = image[roi_h_min:roi_h_max, roi_w_min:roi_w_max]
        
        # 创建调试图像
        debug_image = cv2.cvtColor(image.copy(), cv2.COLOR_GRAY2BGR)
        
        # 标记ROI区域
        cv2.rectangle(debug_image, (roi_w_min, roi_h_min), (roi_w_max, roi_h_max), (0, 255, 0), 2)
        cv2.putText(debug_image, "远距离ROI", (roi_w_min+5, roi_h_min+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        flip_binary = cv2.flip(roi, -1)  # flip the image horizontally and vertically
        
        # 创建翻转后的ROI调试图像
        roi_debug = cv2.cvtColor(flip_binary.copy(), cv2.COLOR_GRAY2BGR)
        
        (x_0, y_0) = cv2.minMaxLoc(flip_binary)[-1]  # extract the coordinates of the top-left point with a value of 255
        # 在翻转图像中标记第一个点
        cv2.circle(roi_debug, (x_0, y_0), 5, (0, 0, 255), -1)
        cv2.putText(roi_debug, f"点1: ({x_0},{y_0})", (x_0+5, y_0-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        y_center = y_0 + 55
        # 在翻转图像中标记y_center+55位置
        cv2.line(roi_debug, (0, y_center), (roi_w_max-roi_w_min, y_center), (0, 255, 255), 2)
        cv2.putText(roi_debug, f"y_center+55={y_center}", (10, y_center-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        roi_section = flip_binary[y_center:, :]
        (x_1, y_1) = cv2.minMaxLoc(roi_section)[-1]
        # 在翻转图像中标记第二个点
        cv2.circle(roi_debug, (x_1, y_1+y_center), 5, (255, 0, 0), -1)
        cv2.putText(roi_debug, f"点2: ({x_1},{y_1+y_center})", (x_1+5, y_1+y_center-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        
        down_p = (roi_w_max - x_1, roi_h_max - (y_1 + y_center))
        
        y_center = y_0 + 65
        # 在翻转图像中标记y_center+65位置
        cv2.line(roi_debug, (0, y_center), (roi_w_max-roi_w_min, y_center), (255, 255, 0), 2)
        cv2.putText(roi_debug, f"y_center+65={y_center}", (10, y_center-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        roi_section = flip_binary[y_center:, :]
        (x_2, y_2) = cv2.minMaxLoc(roi_section)[-1]
        # 在翻转图像中标记第三个点
        cv2.circle(roi_debug, (x_2, y_2+y_center), 5, (255, 0, 255), -1)
        cv2.putText(roi_debug, f"点3: ({x_2},{y_2+y_center})", (x_2+5, y_2+y_center-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        
        up_p = (roi_w_max - x_2, roi_h_max - (y_2 + y_center))
        
        # 在原图中标记计算出的点
        cv2.circle(debug_image, down_p, 5, (0, 0, 255), -1)
        cv2.putText(debug_image, f"down_p", (down_p[0]+5, down_p[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        cv2.circle(debug_image, up_p, 5, (255, 0, 0), -1)
        cv2.putText(debug_image, f"up_p", (up_p[0]+5, up_p[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        up_point = (0, 0)
        down_point = (0, 0)
        if up_p[1] - down_p[1] != 0 and up_p[0] - down_p[0] != 0:
            up_point = (int(-down_p[1]/((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) + down_p[0]), 0)
            down_point = (int((h - down_p[1])/((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) + down_p[0]), h)
            
            # 绘制计算出的车道线
            cv2.line(debug_image, up_point, down_point, (255, 255, 0), 2)
            cv2.putText(debug_image, f"up_point", (up_point[0]+5, up_point[1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(debug_image, f"down_point", (down_point[0]+5, down_point[1]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # 显示调试图像
        # cv2.imshow('远距离补线可视化', debug_image)
        # cv2.imshow('翻转远距离ROI', roi_debug)

        return up_point, down_point

    def add_vertical_line_near(self, image):
        # ——|         |——        |
        #   |   --->  |     --->
        h, w = image.shape[:2]
        roi_w_min = 0
        roi_w_max = int(w/2)
        roi_h_min = int(h/3)
        roi_h_max = h
        roi = image[roi_h_min:roi_h_max, roi_w_min:roi_w_max]
        
        # 创建一个彩色图像用于可视化
        debug_image = cv2.cvtColor(image.copy(), cv2.COLOR_GRAY2BGR)
        
        # 标记ROI区域
        cv2.rectangle(debug_image, (roi_w_min, roi_h_min), (roi_w_max, roi_h_max), (0, 255, 0), 2)
        
        flip_binary = cv2.flip(roi, -1)  # flip the image horizontally and vertically
        
        (x_0, y_0) = cv2.minMaxLoc(flip_binary)[-1]  # extract the coordinates of the top-left point with a value of 255
        down_p = (roi_w_max - x_0, roi_h_max - y_0)
        
        # 在原图中标记down_p点
        cv2.circle(debug_image, down_p, 5, (0, 0, 255), -1)
        cv2.putText(debug_image, f"down_p", (down_p[0]+5, down_p[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        (x_1, y_1) = cv2.minMaxLoc(roi)[-1]
        y_center = int((roi_h_max - roi_h_min - y_1 + y_0)/2)
        
        # 在翻转后的ROI图像中标记y_center位置
        # roi_debug = cv2.cvtColor(flip_binary.copy(), cv2.COLOR_GRAY2BGR)
        # cv2.line(roi_debug, (0, y_center), (roi_w_max-roi_w_min, y_center), (0, 255, 255), 2)
        # cv2.putText(roi_debug, f"y_center={y_center}", (10, y_center-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # 在原图中标记y_center对应的水平线
        actual_y_center = roi_h_max - y_center
        cv2.line(debug_image, (roi_w_min, actual_y_center), (roi_w_max, actual_y_center), (0, 0, 255), 2)
        cv2.putText(debug_image, f"y_center={y_center}", (roi_w_min+5, actual_y_center-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        roi = flip_binary[y_center:, :] 
        (x, y) = cv2.minMaxLoc(roi)[-1]
        up_p = (roi_w_max - x, roi_h_max - (y + y_center))
        
        # 在原图中标记up_p点
        cv2.circle(debug_image, up_p, 5, (255, 0, 0), -1)
        cv2.putText(debug_image, f"up_p", (up_p[0]+5, up_p[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        up_point = (0, 0)
        down_point = (0, 0)
        if up_p[1] - down_p[1] != 0 and up_p[0] - down_p[0] != 0:
            up_point = (int(-down_p[1]/((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) + down_p[0]), 0)
            down_point = down_p
            
            # 绘制计算出的车道线
            cv2.line(debug_image, up_point, down_point, (255, 255, 0), 2)

        # 显示调试图像
        # cv2.imshow('y_center可视化', debug_image)
        # cv2.imshow('翻转ROI中的y_center', roi_debug)

        return up_point, down_point, y_center

    def get_binary(self, image):
        # 通过LAB空间识别颜色
        img_lab = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)  # 将RGB转换为LAB
        img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # 高斯模糊去噪
        mask = cv2.inRange(img_blur, tuple(lab_data['lab']['Stereo'][self.target_color]['min']), tuple(lab_data['lab']['Stereo'][self.target_color]['max']))  # 二值化
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀

        return dilated

    def __call__(self, image, result_image):
        # extract the center point based on the proportion
        centroid_sum = 0
        h, w = image.shape[:2]
        max_center_x = -1
        center_x = []
        for roi in self.rois:
            blob = image[roi[0]:roi[1], roi[2]:roi[3]]  # crop ROI
            contours = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # find contours
            max_contour_area = self.get_area_max_contour(contours, 30)  # obtain the contour with the largest area
            if max_contour_area is not None:
                rect = cv2.minAreaRect(max_contour_area[0])  # the minimum bounding rectangle
                box = np.intp(cv2.boxPoints(rect))  # four corners
                for j in range(4):
                    box[j, 1] = box[j, 1] + roi[0]
                cv2.drawContours(result_image, [box], -1, (255, 255, 0), 2)  # draw the rectangle composed of the four points

                # obtain the diagonal points of the rectangle
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                # the center point of the line
                line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

                cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)  # draw the center point
                center_x.append(line_center_x)
            else:
                center_x.append(-1)
        for i in range(len(center_x)):
            if center_x[i] != -1:
                if center_x[i] > max_center_x:
                    max_center_x = center_x[i]
                centroid_sum += center_x[i] * self.rois[i][-1]
        if centroid_sum == 0:
            return result_image, None, max_center_x
        center_pos = centroid_sum / self.weight_sum  # calculate the center point based on the proportion
        angle = math.degrees(-math.atan((center_pos - (w / 2.0)) / (h / 2.0)))
        
        return result_image, angle, max_center_x

image_queue = queue.Queue(2)
def image_callback(ros_image):
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    bgr_image = np.array(cv_image, dtype=np.uint8)
    if image_queue.full():
        # if the queue is full, remove the oldest image
        image_queue.get()
        # put the image into the queue
    image_queue.put(bgr_image)

def main():
    running = True
    # self.get_logger().info('\033[1;32m%s\033[0m' % (*tuple(lab_data['lab']['Stereo'][self.target_color]['min']), tuple(lab_data['lab']['Stereo'][self.target_color]['max'])))

    while running:
        try:
            image = image_queue.get(block=True, timeout=1)
        except queue.Empty:
            if not running:
                break
            else:
                continue
        binary_image = lane_detect.get_binary(image)
        cv2.imshow('binary', binary_image)
        img = image.copy()
        y = lane_detect.add_horizontal_line(binary_image)
        roi = [(0, y), (640, y), (640, 0), (0, 0)]
        cv2.fillPoly(binary_image, [np.array(roi)], [0, 0, 0])  # fill the top with black to avoid interference
        min_x = cv2.minMaxLoc(binary_image)[-1][0]
        cv2.line(img, (min_x, y), (640, y), (255, 255, 255), 50)  # draw a virtual line to guide the turning
        result_image, angle, x = lane_detect(binary_image, image.copy()) 
        '''
        up, down = lane_detect.add_vertical_line_far(binary_image)
        #up, down, center = lane_detect.add_vertical_line_near(binary_image)
        cv2.line(img, up, down, (255, 255, 255), 10)
        '''
        cv2.imshow('image', img)
        key = cv2.waitKey(1)
        if key == ord('q') or key == 27:  # press Q or Esc to quit
            break

    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    import rclpy
    from sensor_msgs.msg import Image
    rclpy.init()
    node = rclpy.create_node('lane_detect')
    lane_detect = LaneDetector('yellow')
    node.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image', image_callback, 1)
    threading.Thread(target=main, daemon=True).start()
    rclpy.spin(node)





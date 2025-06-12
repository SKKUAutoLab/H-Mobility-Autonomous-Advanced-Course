import cv2
import numpy as np
import sys
import os
from typing import Tuple

message = """
 ____    ___   ____    ____  
|  _ \  / _ \ / ___|  |___ \ 
| |_) || | | |\___ \    __) |
|  _ < | |_| | ___) |  / __/ 
|_| \_\ \___/ |____/  |_____|
                             
    _            _                 
   / \    _   _ | |_   ___         
  / _ \  | | | || __| / _ \  _____ 
 / ___ \ | |_| || |_ | (_) ||_____|
/_/   \_\ \__,_| \__| \___/        
                                   
__     __       _      _        _       
\ \   / /  ___ | |__  (_)  ___ | |  ___ 
 \ \ / /  / _ \| '_ \ | | / __|| | / _ \
  \ V /  |  __/| | | || || (__ | ||  __/
   \_/    \___||_| |_||_| \___||_| \___|  

"""
print(message)

print("ROS2 기반 자율주행 설계 및 구현")
print("Sungkyunkwan University Automation Lab.")

print("------------------Authors------------------")
print("Hyeong-Keun Hong <whaihong@g.skku.edu>")
print("Jinsun Lee <with23skku@g.skku.edu>")
print("Siwoo Lee <edenlee@g.skku.edu>")
print("Jae-Wook Jeon <jwjeon@skku.edu>")
print("------------------------------------------")

def dominant_gradient(image, theta_limit):
	right_limit_radian = np.deg2rad(90+(90-theta_limit))
	left_limit_radian = np.deg2rad(90-(90-theta_limit))
	
	(height, width) = (image.shape[0], image.shape[1])
	
	if image.dtype != np.uint8:
		image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')

	image_original = image.copy()
	
	try:
		lines = cv2.HoughLines(image, 1, np.pi/180, int(width*(25/640)))
		angles = []
		
		if lines is not None:
			for line in lines:
				for rho, theta in line:
					a = np.cos(theta)
					b = np.sin(theta)
					x0 = a*rho
					y0 = b*rho
					x1 = int(x0 + 1000*(-b))
					y1 = int(y0+1000*(a))
					x2 = int(x0 - 1000*(-b))
					y2 = int(y0 -1000*(a))

					if theta < right_limit_radian and theta > left_limit_radian:
						continue
					else:
						angle = np.arctan((x2-x1)/(y1-y2))*180/np.pi
						angles.append(angle)
					
						cv2.line(image_original, (x1,y1), (x2,y2), (255,255,255))
		
		if len(angles) == 0:
			result = 0.
		else:
			result = np.median(angles)
		
		return result          
	except Exception as e:
		_, _, tb = sys.exc_info()
		print(f"gradient detection error = {e}, error line = {tb.tb_lineno}")
		exception_image_path = "./exception_image/"
		try:
			if not os.path.exists(exception_image_path):
				os.mkdir(exception_image_path)    
		except OSError:
			print('Error: Creating directory. ' + exception_image_path)
		return 0, None

def warpping(image, srcmat, dstmat):
	(h, w) = (image.shape[0], image.shape[1])
	transform_matrix = cv2.getPerspectiveTransform(srcmat, dstmat)
	minv = cv2.getPerspectiveTransform(dstmat, srcmat)
	_image = cv2.warpPerspective(image, transform_matrix, (w,h))
	return _image, minv

def bird_convert(img, srcmat, dstmat):
	srcmat = np.float32(srcmat)
	dstmat = np.float32(dstmat)
	img_warpped, minverse = warpping(img, srcmat, dstmat)
	return img_warpped

def roi_rectangle_below(img, cutting_idx):
	img = img[cutting_idx:]
	return img

def draw_edge(cv_image: np.array, detection, color: Tuple[int]) -> np.array:
	mask_msg = detection.mask
	mask_array = np.array([[int(ele.x), int(ele.y)] for ele in mask_msg.data])
	
	if mask_msg.data:
		cv_image = cv2.polylines(cv_image, [mask_array], isClosed=True, color=color, thickness=1, lineType=cv2.LINE_AA)
	return cv_image

def draw_edges(detection_msg, cls_name: str, color: Tuple[int]):
	cv_image = np.zeros((detection_msg.detections[0].mask.height, detection_msg.detections[0].mask.width))
	for detection in detection_msg.detections:
		if detection.class_name == cls_name:
			cv_image = draw_edge(cv_image, detection, color=255)
	return cv_image

def edge_image_postproc(cv_image: np.array, show_image=True):
	(h, w) = (cv_image.shape[0], cv_image.shape[1])
	dst_mat = [[round(w * 0.3), round(h * 0.0)], [round(w * 0.7), round(h * 0.0)], [round(w * 0.7), h], [round(w * 0.3), h]]
	src_mat = [[238, 316],[402, 313], [501, 476], [155, 476]]

	lane2_bird_img = bird_convert(cv_image, srcmat=src_mat, dstmat=dst_mat)
	roi_img = roi_rectangle_below(lane2_bird_img, 300)

	if show_image:
		cv_image_names = ['lane2_edge_img', 'lane2_bird_img', 'roi_img']
		cv_image_list = [cv_image, lane2_bird_img, roi_img]
		for name, image in zip(cv_image_names, cv_image_list):
			cv2.imshow(name, image)
		cv2.waitKey(1)

	return roi_img

def get_lane_center(cv_image: np.array, detection_height: int, detection_thickness: int, road_gradient: float, lane_width: int) -> int:
	detection_area_upper_bound = detection_height - int(detection_thickness/2)
	detection_area_lower_bound = detection_height + int(detection_thickness/2)

	detected_x_coords = np.sort(np.where(cv_image[detection_area_upper_bound:detection_area_lower_bound,:]!=0)[1])
	
	if (detected_x_coords.shape[0] < 5):
		line_x_axis_pixel = None
		center_pixel = None
		return lane_width/2
	
	cut_outliers_array = detected_x_coords[1:-1]
	difference_array = cut_outliers_array[1:] - cut_outliers_array[:-1]
	
	max_diff_idx_left = np.argmax(difference_array)
	max_diff_idx_right = np.argmax(difference_array)+1
	left_val = cut_outliers_array[max_diff_idx_left]
	right_val = cut_outliers_array[max_diff_idx_right]
	
	if abs(left_val - right_val) < (lane_width/3):
		line_x_axis_pixel = cut_outliers_array[round((cut_outliers_array.shape[0])/2)]
		center_pixel = None
	else:
		line_x_axis_pixel = None
		center_pixel = (left_val + right_val)/2
	
	if (center_pixel == None) & (line_x_axis_pixel == None):
		road_target_point_x = (lane_width/2)
	else:
		road_target_point_x = center_pixel
		if (road_target_point_x == None) & (line_x_axis_pixel != None):
			if road_gradient > 0:
				road_target_point_x = line_x_axis_pixel + (lane_width/2)
			else:
				road_target_point_x = line_x_axis_pixel - (lane_width/2)
			if road_target_point_x > (lane_width-1):
				road_target_point_x = (lane_width-1)
			elif road_target_point_x < 0:
				road_target_point_x = 0
	
	return road_target_point_x


def get_traffic_light_color(cv_image: np.array, bbox, hsv_ranges: dict) -> str:
	x_min, x_max = int(bbox.center.position.x - bbox.size.x / 2), int(bbox.center.position.x + bbox.size.x / 2)
	y_min, y_max = int(bbox.center.position.y - bbox.size.y / 2), int(bbox.center.position.y + bbox.size.y / 2)
	roi = cv_image[y_min:y_max, x_min:x_max]
	
	hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

	red_lower1, red_upper1 = hsv_ranges['red1']
	red_lower2, red_upper2 = hsv_ranges['red2']
	yellow_lower, yellow_upper = hsv_ranges['yellow']
	green_lower, green_upper = hsv_ranges['green']
	
	red_mask1 = cv2.inRange(hsv_roi, red_lower1, red_upper1)
	red_mask2 = cv2.inRange(hsv_roi, red_lower2, red_upper2)
	red_mask = red_mask1 + red_mask2
	yellow_mask = cv2.inRange(hsv_roi, yellow_lower, yellow_upper)
	green_mask = cv2.inRange(hsv_roi, green_lower, green_upper)
	
	red_ratio = cv2.countNonZero(red_mask) / (roi.size / 3)
	yellow_ratio = cv2.countNonZero(yellow_mask) / (roi.size / 3)
	green_ratio = cv2.countNonZero(green_mask) / (roi.size / 3)
	
	max_ratio = max(red_ratio, yellow_ratio, green_ratio)
	if max_ratio == red_ratio:
		return "Red"
	elif max_ratio == yellow_ratio:
		return "Yellow"
	elif max_ratio == green_ratio:
		return "Green"
	else:
		return "Unknown"
		
		


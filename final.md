```import cv2
import math
from pyzbar import pyzbar
import rospy
from cv_bridge import CvBridge
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image'''

rospy.init_node('flight')
bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# Topics

oil_detection = rospy.Publisher('/oil_detect', Image, queue_size=1)
defect_detection = rospy.Publisher('/defect_detect', Image, queue_size=1)
# Colors

blue = ((90, 60, 70), (116, 255, 255))
yellow = ((20, 60, 60), (40, 255, 255))
orange = ((0, 27, 70), (20, 255, 255))

# Line color

our_line_color = yellow

# Bools

can_line_fly = False

qrcode = None

def image_callback(data):

    global our_line_color, orange, can_line_fly, qrcode

    image = bridge.imgmsg_to_cv2(data, 'bgr8')
    oil_image = image.copy()
    defect_image = image.copy()

    barcodes = pyzbar.decode(image)
    for barcode in barcodes:
        b_data = barcode.data.decode("utf-8")
        b_type = barcode.type
        (x, y, w, h) = barcode.rect
        xc = x + w/2
        yc = y + h/2
        qrcode = b_data

    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img_line_th = cv2.inRange(img_hsv, our_line_color[0], our_line_color[1])
    img_oil_th = cv2.inRange(img_hsv, orange[0], orange[1])

    H, W, _ = image.shape

    img_line_cnts, _ = cv2.findContours(img_line_th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    img_line_cnts.sort(key = cv2.contourArea)

    img_oil_cnts, _ = cv2.findContours(img_oil_th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    img_oil_cnts.sort(key = cv2.contourArea)

    if len(img_oil_cnts) > 0 and can_line_fly:

        # Распознавание контура 

        oil_contour = img_oil_cnts[-1]
        S = cv2.contourArea(oil_contour)
        if S > 600:
            cv2.drawContours(oil_image, [oil_contour], 0, (255, 0, 0), 5)
            print("oil area", S)

    if len(img_line_cnts) > 0 and can_line_fly:
        line_contour = img_line_cnts[-1]

        if cv2.contourArea(line_contour) > 500:

            # Распознавание линии

            m = cv2.moments(line_contour)

            line_1_cnts, _ = cv2.findContours(img_line_th[0:H//3, 0:W], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE, offset=(0, 0))
            line_1_cnts.sort(key = cv2.contourArea)
            line_1_f = line_1_cnts[-1] if len(line_1_cnts) > 0 else 0

            line_2_cnts, _ = cv2.findContours(img_line_th[H//3:2*H//3, 0:W], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE, offset=(0, H//3))
            line_2_cnts.sort(key = cv2.contourArea)
            line_2_f = line_2_cnts[-1] if len(line_2_cnts) > 0 else 0

            line_3_cnts, _ = cv2.findContours(img_line_th[2*H//3:H, 0:W], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE, offset=(0, 2 * H//3))
            line_3_cnts.sort(key = cv2.contourArea)
            line_3_f = line_3_cnts[-1] if len(line_3_cnts) > 0 else 0

            # Определение участков контура

            if len(line_1_cnts) > 0: cv2.drawContours(image, [line_1_f], 0, (0, 255, 0), 5)
            if len(line_2_cnts) > 0: cv2.drawContours(image, [line_2_f], 0, (0, 255, 0), 5)
            if len(line_3_cnts) > 0: cv2.drawContours(image, [line_3_f], 0, (0, 255, 0), 5)

            m = cv2.moments(line_1_f)
            cx1 = int(m['m10'] / m['m00'])
            cy1 = int(m['m01'] / m['m00'])

            m = cv2.moments(line_2_f)
            cx2 = int(m['m10'] / m['m00'])
            cy2 = int(m['m01'] / m['m00'])

            #error = cx - W // 2

            #cv2.drawContours(image, [line_contour], 0, (0, 0, 255), 5)

            alpha = math.atan((cx2 - cx1) / (cy2 - cy1)) / math.pi * 180

            cv2.line(image, (cx1, cy1), (cx2, cy2), (255, 255, 255), 5)

            #print ( round(alpha, 2), 'grad' )

            set_velocity(vx = 0.1, vy = 0, vz = 0, yaw=float('nan'), yaw_rate=alpha*0.05, frame_id='body')

    if len(img_line_cnts) > 1 and can_line_fly:
        pyt_contour = img_line_cnts[-2]

        m = cv2.moments(line_contour)
        cx = int(m['m10'] / m['m00'])
        cy = int(m['m01'] / m['m00'])

        if cv2.contourArea(pyt_contour) > 200 and (H//2-H//3 <= cy <= H//2+H//3) and (W//2-W//3 <= cx <= W//2+W//3):
            cv2.drawContours(defect_image, [pyt_contour], 0, (180, 105, 255), 5)
            print("defect", round(get_telemetry('aruco_map').x, 2), round(get_telemetry('aruco_map').y, 2))

    #img = img_line_th[H//2-H//3:H//2+H//3, W//2-W//3:W//2+W//3]

    #print(img.shape)

    #cv2.rectangle(image, (W//2-W//3, H//2-H//3), (W//2+W//3, H//2+H//3), (255, 0, 255), 5)

    oil_detection.publish(bridge.cv2_to_imgmsg(oil_image, 'bgr8'))
    defect_detection.publish(bridge.cv2_to_imgmsg(defect_image, 'bgr8'))
    #wrong_detection.publish(bridge.cv2_to_imgmsg(img_line_th[H//2-H//6:H//2+H//6, W//2-W//6:W//2+W//6], 'mono8'))

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
high = 0.87

navigate(x=0, y=0, z=0.68, speed=0.3, auto_arm=True, frame_id='body')
rospy.sleep(5)

startT = get_telemetry(frame_id='aruco_map')

navigate(x=startT.x, y=startT.y, z=0.6, yaw=-math.pi / 2, frame_id='aruco_map', speed=0.3)
rospy.sleep(5)

while (qrcode == None):
    rospy.sleep(1)

qrd = qrcode.split('\n')

print("Navigation area x=" + qrd[0].split()[0], "y=" + qrd[1].split()[1])
print("Lake center x=" + qrd[1].split()[0], "y=" + qrd[1].split()[1])

x, y = map(float, qrd[0].split())
lake_x, lake_y = map(float, qrd[1].split())

rospy.sleep(1)

# print('Navigate to lake')

navigate(x=lake_x, y=lake_y, z=1.5, yaw=-math.pi / 2, speed=0.3, frame_id='aruco_map', auto_arm=True)
rospy.sleep(15)

navigate(x=lake_x, y=lake_y, z=0.8, yaw=-math.pi / 2, speed=0.3, frame_id='aruco_map', auto_arm=True)
rospy.sleep(6)

print('Successful water withdrawal')

navigate(x=lake_x, y=lake_y, z=1.25, yaw=-math.pi / 2, speed=0.3, frame_id='aruco_map', auto_arm=True)
rospy.sleep(15)

# 'Navigate to oil start point'
navigate(x=x, y=y, z=1.25, yaw=-math.pi / 2, speed=0.3, frame_id='aruco_map', auto_arm=True)
rospy.sleep(15)

# 'Moving for line'

can_line_fly = True

rospy.spin()
```python3

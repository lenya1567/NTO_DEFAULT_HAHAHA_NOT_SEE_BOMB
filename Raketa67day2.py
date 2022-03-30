import math
import cv2
import rospy
from clover import srv
from std_srvs.srv import Trigger
from clover.srv import SetLEDEffect
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar import pyzbar
import sys

rospy.init_node('barcode_test')

bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
land = rospy.ServiceProxy('land', Trigger)

qrcode = None
telem = None

high = 0.87

BLUE = ((156, 100, 80), (184, 255, 100))
YELLOW = ((20, 127, 100), (40, 255, 255))

#oil_pub = rospy.Publisher('oil', Image, queue_size=1)
line_pub = rospy.Publisher('oil_detect', Image, queue_size=1)

def readFirstTelemetriy():
    T = get_telemetry(frame_id='aruco_map')
    return T.x, T.y

can = False

def image_callback(data):
    global qrcode

    img = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

    barcodes = pyzbar.decode(img)
    for barcode in barcodes:
        b_data = barcode.data.decode("utf-8")
        b_type = barcode.type
        (x, y, w, h) = barcode.rect
        xc = x + w/2
        yc = y + h/2
        qrcode = b_data

    YOUR_COLOR = YELLOW

    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    oil_image = cv2.inRange(img, (10, 60, 60), (23, 255, 255))
    line_image = cv2.inRange(img, YOUR_COLOR[0], YOUR_COLOR[1])

    # Поиск всех контуров на картинке с линией

    contours_blk, _ = cv2.findContours(line_image.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Сортировка по контурам

    contours_blk.sort(key=cv2.minAreaRect)

    if len(contours_blk) > 0:

        # Движение по линии

        cnt = contours_blk[0]
        if cv2.contourArea(cnt) > 300:

            # распознавание прямоугольного контура линии

            rect = cv2.minAreaRect(cnt)

            # Поиск координат и наклона прямоугольника с контура

            (x_min, y_min), (w_min, h_min), angle = rect
            if angle < -45:
                angle = 90 + angle
            if w_min < h_min and angle > 0:
                angle = (90 - angle) * -1
            if w_min > h_min and angle < 0:
                angle = 90 + angle

            # Нахождения центра изображния и расчёт ошибки для дальнейшего движения коптера по П-регулятору

            center = img.shape[1] / 2
            error = x_min - center

            # Задание нужных скоростей для движения по линии

            print(round(angle, 2), error)
            set_velocity(vx=0.1, vy=error*(-0.01), vz=0, yaw=float('nan'), yaw_rate=angle*(-0.008), frame_id='body')
    else:

        # Условие возрата в начальную точку

        if can:

            # Если это сработало, то мы не видим линии и уже по ней летали => надо возвращаться на начальную точку и приземляться

            navigate(x=myTelem[0], y=myTelem[1], z=1.5, yaw=0, speed=0.3, frame_id='aruco_map', auto_arm=True)
            rospy.sleep(30)
            land()
            sys.exit(0)

    # print(cv2.findContours(oil_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE))

    #oil_point.sort(key = cv2.contourArea)

    #line_pub.publish(bridge.cv2_to_imgmsg(line_image, 'mono8'))

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)

# Взлетаем и ожидаем распознавания QRCODE

navigate(z=0.6, yaw=0, speed=0.3, frame_id='body', auto_arm=True)
rospy.sleep(10)

myTelem = readFirstTelemetriy()
print("First")
print(round(myTelem[0], 2), round(myTelem[1], 2))

while (qrcode == None):
    rospy.sleep(1)

print("Navigation area x=" + qrcode.split()[0], "y=" + qrcode.split[1])

x, y = map(float, qrcode.split())

rospy.sleep(1)

# Полёт в начало нефтепровода

navigate(x=x, y=y, z=0.6, yaw=0, speed=0.3, frame_id='aruco_map', auto_arm=True)
rospy.sleep(10)

can = True

# Движения по линии

rospy.spin()

land()
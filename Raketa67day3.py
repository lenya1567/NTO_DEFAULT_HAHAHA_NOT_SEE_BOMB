import math
from pyzbar import pyzbar
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import srv
from std_srvs.srv import Trigger
import numpy

rospy.init_node('computer_vision_sample')
bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

BLUE = ((156, 127, 80), (184, 255, 100))
YELLOW = ((20, 90, 80), (40, 255, 255))

x = 90

oil_pub = rospy.Publisher('oil', Image, queue_size=1)
line_pub = rospy.Publisher('line', Image, queue_size=1)
test_pub = rospy.Publisher('test', Image, queue_size=1)
pyt_pub = rospy.Publisher('PYTNASHKO', Image, queue_size=1)

l_error_a = 0
l_error = 0

can = False
qrcode = None

can_rotate = True
coords = None

def image_callback(data):
    global x, l_error, l_error_a, qrcode, can_rotate, coords
    img = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

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

    oil_image = cv2.inRange(img, (12, 127, 80), (23, 255, 255))
    line_image = cv2.inRange(img, YOUR_COLOR[0], YOUR_COLOR[1])
    # oil_image = oil_image[img.shape[0]-240:img.shape[1]+240][:]
    contours_blk, _ = cv2.findContours(line_image.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_blk.sort(key=cv2.contourArea)
    if len(contours_blk) > 0 and can:
        cv2.drawContours(img, [contours_blk[-1]], 0, (255, 0, 0), 5)
        cnt = contours_blk[-1]
        rect = cv2.minAreaRect(cnt)

        # Поиск координат и наклона прямоугольника с контура
        (x_min, y_min), (w_min, h_min), angle = rect
        if angle < -45:
            angle = 90 + angle
        if w_min < h_min and angle > 0:
            angle = (90 - angle) * -1
        if w_min > h_min and angle < 0:
            angle = 90 + angle
        #rows, cols = img.shape[:2]
        #[vx, vy, x, y] = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)
        #lefty = int((-x * vy/vx) + y)
        #righty = int(((cols - x)*vy/vx)+y)
        #alpha = math.atan((1 - cols)/(lefty-righty)) * 180 / math.pi
        #print(alpha)
        # Нахождения центра изображния и расчёт ошибки для дальнейшего движения коптера по П-$

        center = oil_image.shape[1] / 2
        error = x_min - center

        # Задание нужных скоростей для движения по линии

        dA = angle - l_error_a
        dL = error - l_error

        l_error = error
        l_error_a = angle

        #print(round(angle, 2), error)

        mn = -1 if angle < 0 else 1

        set_velocity(vx=0.08, vy=-(error*(0.005)+dL*0.002), vz=0, yaw=float('nan'),
        yaw_rate=-(abs(angle)**4*mn*(0.003) + dA * 0.0001), frame_id='body')

        stencil = numpy.zeros(img.shape).astype(img.dtype)
        color = [255, 255, 255]
        cv2.fillPoly(stencil, [cnt], color)
        img = cv2.bitwise_and(img, stencil)

        mask = cv2.inRange(img.copy(), (0, 0, 0), (255, 100, 100))

        cnts, _ = cv2.findContours(oil_image.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        cnts.sort(key = cv2.contourArea)
        now_cnt = None
        i = 0
        for e in cnts:

            if cv2.contourArea(e) > 50 and i != len(cnts) - 1:
                now_cnt = e
                break
            i += 1

        if now_cnt != None:
            m = cv2.moments(now_cnt)
            pyt_pub.publish(bridge.cv2_to_imgmsg(mask, 'mono8'))
            cx = int(m['m10'] / m['m00'])
            cy = int(m['m01'] / m['m00'])
            if 270 <= cx <= 370 and 190 <= cy <= 290:
                print("OIL DATA:")
                print(get_telemetry('aruco_map').x, get_telemetry('aruco_map').y)

                cnts_oil = cv2.findContours(oil_image.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                cnts_oil.sort(key = cv2.contourArea)

                print(cv2.contourArea(cnts_oil[-1]))
                print()


    oil_pub.publish(bridge.cv2_to_imgmsg(oil_image, 'mono8'))
    line_pub.publish(bridge.cv2_to_imgmsg(line_image, 'mono8'))
    test_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)

high = 0.87

navigate(x=0, y=0, z=0.68, speed=0.3, auto_arm=True, frame_id='body')
rospy.sleep(5)

startT = get_telemetry(frame_id='aruco_map')

navigate(x=startT.x, y=startT.y, z=0.6, yaw=-math.pi / 2, frame_id='aruco_map', speed=0.3)
rospy.sleep(5)

while (qrcode == None):
    rospy.sleep(1)

print("Navigation area x=" + qrcode.split()[0], "y=" + qrcode.split[1])

x, y = map(float, qrcode.split())

rospy.sleep(1)

# Полёт в начало нефтепровода

navigate(x=x, y=y, z=0.5, yaw=-math.pi / 2, speed=0.3, frame_id='aruco_map', auto_arm=True)
rospy.sleep(15)

can = True

coords = (x, y)

rospy.spin()
import cv2
from networktables import NetworkTables
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import clock

import grip

NetworkTables.initialize(server='roborio-167-frc.local')
table = NetworkTables.getTable('myContoursReport')

camera = PiCamera()
camera.resolution = (416, 320)
camera.framerate = 24
camera.exposure_compensation = 0
rawCapture = PiRGBArray(camera, size=(416, 320))
processor = grip.GripPipeline()
for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
    contours = processor.process(frame.array)
    datax, datay, dataw, datah = [], [], [], []
    for contour in contours:
        data = cv2.boundingRect(contour)
        datax.append(data[0])
        datay.append(data[1])
        dataw.append(data[2])
        datah.append(data[3])
    table.putNumberArray('x', datax)
    table.putNumberArray('y', datay)
    table.putNumberArray('w', dataw)
    table.putNumberArray('h', datah)
    print("[%d] Put one frame with %d contours" % (1000 * clock(), len(contours)))
    rawCapture.truncate(0)

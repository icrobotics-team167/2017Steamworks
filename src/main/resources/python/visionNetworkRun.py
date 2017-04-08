import cv2
from networktables import NetworkTables
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import clock

import grip

NetworkTables.initialize(server='roborio-167-frc.local')
table = NetworkTables.getTable('gearPlacerVision')

camera = PiCamera()
camera.resolution = (416, 320)
camera.framerate = 24
camera.exposure_compensation = 0
camera.hflip = camera.vflip = True
rawCapture = PiRGBArray(camera, size=(416, 320))
processor = grip.GripPipeline()
for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
    contours = processor.process(frame.array)
    datax, datay, dataw, datah = [], [], [], []
    for contour in contours:
        data = cv2.boundingRect(contour)
        if data[2] * data[3] < 170:
            continue
        datax.append(data[0])
        datay.append(data[1])
        dataw.append(data[2])
        datah.append(data[3])
    table.putNumberArray('x', datax)
    table.putNumberArray('y', datay)
    table.putNumberArray('w', dataw)
    table.putNumberArray('h', datah)
    print("[%d] Put one frame with %d contours" % (1000 * clock(), len(datax)))
    if len(datax) >= 2:
        print("(%.2f, %.2f, %.2f, %.2f) | (%.2f, %.2f %.2f %.2f)" % (datax[0], datay[0], dataw[0], datah[0], datax[1], datay[1], dataw[1], datah[1]))
    rawCapture.truncate(0)

# Face Detection Example
#
# This example shows off the built-in face detection feature of the OpenMV Cam.
#
# Face detection works by using the Haar Cascade feature detector on an image. A
# Haar Cascade is a series of simple area contrasts checks. For the built-in
# frontalface detector there are 25 stages of checks with each stage having
# hundreds of checks a piece. Haar Cascades run fast because later stages are
# only evaluated if previous stages pass. Additionally, your OpenMV Cam uses
# a data structure called the integral image to quickly execute each area
# contrast check in constant time (the reason for feature detection being
# grayscale only is because of the space requirment for the integral image).

import sensor, time, image
from pyb import UART


# Reset sensor
sensor.reset()

# Sensor settings
sensor.set_contrast(1)
sensor.set_gainceiling(16)
# HQVGA and GRAYSCALE are the best for face tracking.
sensor.set_framesize(sensor.HQVGA)
sensor.set_pixformat(sensor.GRAYSCALE)

# Load Haar Cascade
# By default this will use all stages, lower satges is faster but less accurate.
face_cascade = image.HaarCascade("frontalface", stages=25)
print(face_cascade)

# FPS clock

#UART init
uart = UART(3, 9600, timeout_char=1000)
uart.init(9600, bits=8, parity=None, stop=1, timeout_char=1000)
count = 0

clock = time.clock()


while (True):
    count = count+1
    print(count)
    clock.tick()
    #print('flag0')
    # Capture snapshot
    #img = sensor.snapshot()
    #print('flag2')
    # Find objects.
    # Note: Lower scale factor scales-down the image more and detects smaller objects.
    # Higher threshold results in a higher detection rate, with more false positives.
    #objects = img.find_features(face_cascade, threshold=0.75, scale_factor=1.25)
    #print('flag3')
    # Draw objects
    #for r in objects:
     #   img.draw_rectangle(r)

    #time.sleep(1000)
    #print('flag4')
    # Print FPS.
    # Note: Actual FPS is higher, streaming the FB makes it slower.
    #print(count % 10)
    if ((count % 10) == 0):
        #print('flag1')
        time.sleep(100)
        print(uart.readline())

    #time.sleep(1000)
#print(clock.fps())

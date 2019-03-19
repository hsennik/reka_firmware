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
from pyb import I2C

i2c = I2C(2, I2C.SLAVE, addr=0x42)
i2c.deinit()
i2c = I2C(2, I2C.SLAVE, addr=0x42)

# Reset sensor
sensor.reset()

# Sensor settings
sensor.set_contrast(1)
sensor.set_gainceiling(16)
# HQVGA and GRAYSCALE are the best for face tracking.
sensor.set_framesize(sensor.HQVGA)
sensor.set_pixformat(sensor.GRAYSCALE)

# Load Haar Cascade
# By default this will use all stages (25?), lower stages is faster but less accurate.
face_cascade = image.HaarCascade("frontalface", stages=15)
print(face_cascade)

# FPS clock
clock = time.clock()

while (True):
    clock.tick()

    # Capture snapshot
    img = sensor.snapshot()
    box_data = "-01,-01,-01,-01"
    print("flag")

    # Find objects.
    # Note: Lower scale factor scales-down the image more and detects smaller objects.
    # Higher threshold results in a higher detection rate, with more false positives.
    objects = img.find_features(face_cascade, threshold=0.75, scale_factor=1.25)
    print("flag2")

    # Draw objects
    for r in objects:
        x_coord = '{:0>3}'.format(r[0])
        y_coord = '{:0>3}'.format(r[1])
        width = '{:0>3}'.format(r[2])
        height = '{:0>3}'.format(r[3])
        # box_data is what we want to send over i2c
        box_data = x_coord + "," + y_coord + "," + width + "," + height
        #print(box_data)
        img.draw_rectangle(r)

    print(box_data)

    # Print FPS.
    # Note: Actual FPS is higher, streaming the FB makes it slower.
    #print(clock.fps())
    try:
        i2c.send(box_data, timeout=10000) # Send the data second.
        print("flag try")
        #print("Sent Data!") # Only reached on no error.
    except OSError as err:
        #print("here")
        pass # Don't care about errors - so pass.
        print("flag except")
        # Note that there are 3 possible errors. A timeout error, a general purpose error, or
    # a busy error. The error codes are 116, 5, 16 respectively for "err.arg[0]".

    #print(i2c.recv(3))

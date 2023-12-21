#!/usr/bin/env python

import rospy
import gxipy as gx
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt

def main():
    rospy.init_node('daheng_camera_node', anonymous=True)

    # Initialize the Daheng camera
    device_manager = gx.DeviceManager()
    dev_num, dev_info_list = device_manager.update_device_list()
    if dev_num == 0:
        rospy.logerr("Number of enumerated devices is 0")
        return

    # Open the first device
    cam = device_manager.open_device_by_index(1)

    # Check if the camera is a color camera
    if not cam.PixelColorFilter.is_implemented():
        rospy.logerr("This script does not support mono cameras.")
        cam.close_device()
        return

    # Set camera settings
    cam.TriggerMode.set(gx.GxSwitchEntry.OFF)
    cam.ExposureTime.set(10000.0)
    cam.Gain.set(10.0)

    # Get parameters for improving image quality
    gamma_lut = gx.Utility.get_gamma_lut(cam.GammaParam.get()) if cam.GammaParam.is_readable() else None
    contrast_lut = gx.Utility.get_contrast_lut(cam.ContrastParam.get()) if cam.ContrastParam.is_readable() else None
    color_correction_param = cam.ColorCorrectionParam.get() if cam.ColorCorrectionParam.is_readable() else 0

    # Start data acquisition
    cam.stream_on()

    plt.ion()  # Interactive mode on
    fig, ax = plt.subplots()  # Create a figure and axes

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Get raw image
        raw_image = cam.data_stream[0].get_image()
        if raw_image is None:
            rospy.logwarn("Getting image failed.")
            continue

        # Convert to RGB and improve image quality
        rgb_image = raw_image.convert("RGB")
        if rgb_image is not None:
            rgb_image.image_improvement(color_correction_param, contrast_lut, gamma_lut)

            # Convert to numpy array and show image
            numpy_image = rgb_image.get_numpy_array()
            if numpy_image is not None:
                ax.imshow(numpy_image)
                ax.set_title('Camera Output')
                plt.draw()
                plt.pause(0.001)  # Pause needed to update the plot
                ax.clear()

        rate.sleep()

    # Stop data acquisition and close device
    cam.stream_off()
    cam.close_device()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception")


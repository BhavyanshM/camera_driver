#include <cstdio>

#include <iostream>
#include <iomanip>

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char *argv[])
{
   const bool DISPLAY = false;

   printf("Initializing ROS Node: Logitech Brio 4k\n");

   ros::init(argc, argv, "logitech_cam");
   ros::NodeHandle nh;

   image_transport::ImageTransport transport(nh);
   image_transport::Publisher leftCamImagePublisher = transport.advertise("/logitech/left/cam/color/image_raw", 1);
   image_transport::Publisher rightCamImagePublisher = transport.advertise("/logitech/right/cam/color/image_raw", 1);

   ros::Rate rate(60); // FPS

   cv::VideoCapture capLeft("/dev/v4l/by-id/usb-046d_Logitech_BRIO_6895AE2F-video-index0");
   capLeft.set(cv::CAP_PROP_BUFFERSIZE, 3);
   capLeft.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
   capLeft.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
   capLeft.set(cv::CAP_PROP_FPS, 30);
   capLeft.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
   capLeft.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
   capLeft.set(cv::CAP_PROP_AUTO_WB, 0.25);
   capLeft.set(cv::CAP_PROP_AUTOFOCUS, 0.25);
   capLeft.set(cv::CAP_PROP_MODE, cv::CAP_OPENCV_MJPEG);

   cv::VideoCapture capRight("/dev/v4l/by-id/usb-046d_Logitech_BRIO_9B0FC657-video-index0");
   capRight.set(cv::CAP_PROP_BUFFERSIZE, 3);
   capRight.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
   capRight.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
   capRight.set(cv::CAP_PROP_FPS, 30);
   capRight.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
   capRight.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
   capRight.set(cv::CAP_PROP_AUTO_WB, 0.25);
   capRight.set(cv::CAP_PROP_AUTOFOCUS, 0.25);
   capRight.set(cv::CAP_PROP_MODE, cv::CAP_OPENCV_MJPEG);

   int count = 0;
   while (ros::ok())
   {

      cv::Mat leftFrame;
      capLeft.read(leftFrame);

      // printf("Getting leftFrame: %d %d\n", leftFrame.rows, leftFrame.cols);

      if (leftFrame.data != nullptr)
      {

         sensor_msgs::ImagePtr camMsgLeft = cv_bridge::CvImage(std_msgs::Header(), "bgr8", leftFrame).toImageMsg();

         leftCamImagePublisher.publish(camMsgLeft);

         if (DISPLAY)
         {
            cv::imshow("RGB Left", leftFrame);

            int key = cv::waitKey(1);
            if (key == 'q' || key == 'Q') // Quit
               break;
         }
      }

      cv::Mat rightFrame;
      capRight.read(rightFrame);

      // printf("Getting rightFrame: %d %d\n", rightFrame.rows, rightFrame.cols);

      if (rightFrame.data != nullptr)
      {

         sensor_msgs::ImagePtr camMsgRight = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rightFrame).toImageMsg();

         rightCamImagePublisher.publish(camMsgRight);

         if (DISPLAY)
         {
            cv::imshow("RGB Right", rightFrame);

            int key = cv::waitKey(1);
            if (key == 'q' || key == 'Q') // Quit
               break;
         }
      }

      ros::spinOnce();
      rate.sleep();
      ++count;
   }

   return EXIT_SUCCESS;
}


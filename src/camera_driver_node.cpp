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
   image_transport::Publisher camImagePublisher = transport.advertise("/logitech/cam/color/image_raw", 1);

   ros::Rate rate(60); // FPS

   cv::VideoCapture cap("/dev/v4l/by-id/usb-046d_Logitech_BRIO_6895AE2F-video-index0");
   cap.set(cv::CAP_PROP_BUFFERSIZE, 3);
   cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
   cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
   cap.set(cv::CAP_PROP_FPS, 30);
   cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
   cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
   cap.set(cv::CAP_PROP_AUTO_WB, 0.25);
   cap.set(cv::CAP_PROP_AUTOFOCUS, 0.25);
   cap.set(cv::CAP_PROP_MODE, cv::CAP_OPENCV_MJPEG);

   int count = 0;
   while (ros::ok())
   {

      cv::Mat frame;
      cap.read(frame);

      printf("Getting Frame: %d %d\n", frame.rows, frame.cols);

      if (frame.data != nullptr)
      {

         sensor_msgs::ImagePtr camMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

         camImagePublisher.publish(camMsg);

         if (DISPLAY)
         {
            cv::imshow("RGB", frame);

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


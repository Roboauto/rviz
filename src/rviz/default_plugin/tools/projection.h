//
// Created by robo on 10/26/17.
//

#ifndef RVIZ_SATELLITE_PROJECTION_H
#define RVIZ_SATELLITE_PROJECTION_H

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>

class Projection {


public:

    Projection(ros::NodeHandle node, std::string camera_frame, std::string origin_frame, int xRange, int yRange, int pixelPerMeter)
                                                                                    : node_(node),
                                                                                      camera_frame_(camera_frame),
                                                                                      origin_frame_(origin_frame),
                                                                                      xRange_(xRange),
                                                                                      yRange_(yRange),
                                                                                      pixelPerMeter_(pixelPerMeter)
    { };

    void warp_image_to_bird_view(const cv::Mat& inputImage, cv::Mat& outputImage);

    void set_camera_info(const sensor_msgs::CameraInfo camera_info) { camera_info_ = camera_info; };

    void set_xRange(const int xRange) { xRange_ = xRange; };

    void set_yRange(const int yRange) { yRange_ = yRange; };

    void set_pixelPerMeter(const int pixelPerMeter) { pixelPerMeter_ = pixelPerMeter; };

    void set_originFrame(const std::string origin_frame) { origin_frame_ = origin_frame; };

    void set_cameraFrame(const std::string camera_frame) { camera_frame_ = camera_frame; };

private:

    ros::NodeHandle node_;

    tf::TransformListener listener_;

    std::string camera_frame_;

    int xRange_;

    int yRange_;

    int pixelPerMeter_ = 50;

    sensor_msgs::CameraInfo camera_info_;

    std::string origin_frame_;



    void get_ground_points_3d( cv::Point3f* ground_points_3d);

    void get_ground_pts_2d(cv::Point2f* ground_pts);

    cv::Matx33f get_camera_rotation_matrix();

    cv::Point3f get_camera_translation_vector();

    cv::Matx33f get_camera_intrinsic_matrix();

    void get_camera_pts(cv::Point2f* camera_pts);

    cv::Mat getWarpMatrix();




};


#endif //RVIZ_SATELLITE_PROJECTION_H

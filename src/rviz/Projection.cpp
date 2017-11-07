//
// Created by robo on 10/26/17.
//

#include "Projection.h"

void Projection::warp_image_to_bird_view(const cv::Mat& inputImage, cv::Mat& outputImage) {

    cv::warpPerspective(inputImage,
                        outputImage,
                        getWarpMatrix(),
                        cv::Size(xRange_ * pixelPerMeter_,
                                 yRange_ * pixelPerMeter_));
}


void Projection::get_ground_points_3d( cv::Point3f* ground_points_3d) {

    ground_points_3d[0] = cv::Point3f(20,5,0);
    ground_points_3d[1] = cv::Point3f(20,-5,0);
    ground_points_3d[2] = cv::Point3f(50,-5,0);
    ground_points_3d[3] = cv::Point3f(50,5,0);
}


void Projection::get_ground_pts_2d(cv::Point2f* ground_pts) {
    cv::Point3f ground_points_3d[4] ;
    get_ground_points_3d(ground_points_3d);

    for (int i = 0; i < 4; i++) {
        ground_pts[i] = cv::Point2f(ground_points_3d[i].x * pixelPerMeter_,
                                    -ground_points_3d[i].y*pixelPerMeter_ +(((float) yRange_) / 2)*pixelPerMeter_);
    }
}



cv::Matx33f Projection::get_camera_rotation_matrix() {

    tf::StampedTransform transform;
    try { listener_.lookupTransform(camera_frame_, origin_frame_, ros::Time(0), transform); }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    tf::Matrix3x3 camera_rot_matrix(transform.getRotation());

    cv::Matx33f camera_to_ground_rot_matrix( 3, 3, CV_32F );
    for (int i = 0 ; i < 3 ; i++) {
        camera_to_ground_rot_matrix(i,0) = (float)camera_rot_matrix.getRow(i).getX();
        camera_to_ground_rot_matrix(i,1) = (float)camera_rot_matrix.getRow(i).getY();
        camera_to_ground_rot_matrix(i,2) = (float)camera_rot_matrix.getRow(i).getZ();
    }



    return camera_to_ground_rot_matrix;
}

cv::Point3f Projection::get_camera_translation_vector() {

    tf::StampedTransform transform;
    try { listener_.lookupTransform(camera_frame_, origin_frame_, ros::Time(0), transform); }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    return cv::Point3f((float)transform.getOrigin().getX(), (float)transform.getOrigin().getY(), (float)transform.getOrigin().getZ());
}


cv::Matx33f Projection::get_camera_intrinsic_matrix() {

    cv::Matx33f camera_intrinsic_matrix( 3, 3, CV_32F );
    camera_intrinsic_matrix(0,0) = (float)camera_info_.P[0] / camera_info_.binning_x;
    camera_intrinsic_matrix(0,1) = (float)camera_info_.P[1];
    camera_intrinsic_matrix(0,2) = (float)(camera_info_.P[2] - camera_info_.roi.x_offset) / camera_info_.binning_x;
    camera_intrinsic_matrix(1,0) = (float)camera_info_.P[4];
    camera_intrinsic_matrix(1,1) = (float)camera_info_.P[5] / camera_info_.binning_y;
    camera_intrinsic_matrix(1,2) = (float)(camera_info_.P[6] - camera_info_.roi.y_offset) / camera_info_.binning_y;
    camera_intrinsic_matrix(2,0) = (float)camera_info_.P[8];
    camera_intrinsic_matrix(2,1) = (float)camera_info_.P[9];
    camera_intrinsic_matrix(2,2) = (float)camera_info_.P[10];

    return camera_intrinsic_matrix;
}


void Projection::get_camera_pts(cv::Point2f* camera_pts) {

    cv::Point3f ground_points_3d[4];
    get_ground_points_3d(ground_points_3d);

    cv::Point3f camera_to_ground_translation = get_camera_translation_vector();

    cv::Point3f camera_rotated_3d_pts[4];
    for (int i = 0 ; i < 4 ; i++) {
        cv::Point3f rotated_point = get_camera_rotation_matrix() * ground_points_3d[i];
        camera_rotated_3d_pts[i] = cv::Point3f(rotated_point.x + camera_to_ground_translation.x,
                                               rotated_point.y + camera_to_ground_translation.y,
                                               rotated_point.z + camera_to_ground_translation.z);
    }

    for (int i = 0 ; i < 4 ; i++) {
        cv::Point3f rotated_point = get_camera_intrinsic_matrix() * camera_rotated_3d_pts[i];
        camera_pts[i] = cv::Point2f(rotated_point.x/rotated_point.z , rotated_point.y /rotated_point.z); // watch dev by 0
    }
}

cv::Mat Projection::getWarpMatrix() {

    cv::Point2f camera_pts[4];
    get_camera_pts(camera_pts);

    cv::Point2f ground_pts[4];
    get_ground_pts_2d(ground_pts);

    return cv::getPerspectiveTransform( camera_pts, ground_pts );
}


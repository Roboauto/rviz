#pragma once

#define ROS_DEBUG_NAMED(X, ...) if(false) { printf(X); printf(__VA_ARGS__); printf("\n"); }
#define ROS_ERROR_NAMED(X, ...) if(true) { fprintf(stderr, X); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); }

#define ROS_ERROR(...)  if(true) {fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n");}
#define ROS_DEBUG(...)  if(false) {printf( __VA_ARGS__); printf("\n");}
#define ROS_WARN(...)  if(true) {printf( __VA_ARGS__); printf("\n");}
#define ROS_INFO(...)  if(true) {printf( __VA_ARGS__); printf("\n");}
#define ROS_FATAL(...) if(true) {printf( __VA_ARGS__); printf("\n");}
#define ROS_LOG(...)

#define ROS_DEBUG_STREAM(...)
#define ROS_ERROR_STREAM(...) if(true) { std::cerr <<  __VA_ARGS__ << std::endl;}
#define ROS_WARN_STREAM(...)
#define ROS_INFO_STREAM(...)

#define ROS_INFO_ONCE(...)
#define ROS_WARN_ONCE_NAMED(...)

#define ROS_ERROR_THROTTLE(...)
#define ROS_WARN_THROTTLE(...)

#define ROS_DEBUG_COND_NAMED(...)
#define ROS_ERROR_STREAM_NAMED(...)

#define ROS_DEPRECATED
#define ROS_BREAK()

#define ROS_PACKAGE_NAME "rviz"

#define ROSTIME_DECL

namespace ros {
}
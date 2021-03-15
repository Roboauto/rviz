#pragma once

#define ROS_DEBUG_NAMED(X, ...) printf(X), printf(__VA_ARGS__)
#define ROS_ERROR_NAMED(X, ...) printf(X), printf(__VA_ARGS__)

#define ROS_ERROR(...) printf( __VA_ARGS__), printf("\n")
#define ROS_DEBUG(...) printf( __VA_ARGS__), printf("\n")
#define ROS_WARN(...) printf( __VA_ARGS__), printf("\n")
#define ROS_INFO(...) printf( __VA_ARGS__), printf("\n")
#define ROS_FATAL(...) printf( __VA_ARGS__), printf("\n")
#define ROS_LOG(...)

#define ROS_DEBUG_STREAM(...)
#define ROS_ERROR_STREAM(...)
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
#ifndef POLYGON_COVERAGE_GEOMETRY_ROS_INTERFACE_
#define POLYGON_COVERAGE_GEOMETRY_ROS_INTERFACE_

#include <iostream>

#define ROS_ASSERT(cond)  \
  do{ \
    if(!(cond)){ \
      std::cout << "ROS_ASSERT failed" << std::endl; \
      std::exit(1); \
    } \
  }while(0)


#define ROS_ASSERT_MSG(cond,...)  \
  do{ \
    if(!(cond)){ \
      std::cout << "ROS_ASSERT failed: " <<  printf(__VA_ARGS__) <<std::endl; \
      std::exit(1); \
    } \
  }while(0)


#define ROS_INFO(...) \
    do{ \
        std::cout << "Info: " << printf(__VA_ARGS__) << std::endl; \
    }while(0)

#define ROS_INFO_STREAM(args)   \
    do{ \
        std::cout << "Info: " << args << std::endl; \
    }while(0)

#define ROS_WARN_STREAM(args)   \
    do{ \
        std::cout << "Warn: " << args << std::endl; \
    }while(0)

#define ROS_WARN(...)   \
    do{ \
        std::cout << "Warn: " << printf(__VA_ARGS__) << std::endl; \
    }while(0)

#define ROS_ERROR(args)   \
    do{ \
        std::cout << "Error: " << args << std::endl; \
    }while(0)

#define ROS_ERROR_STREAM(args)  \
    do{ \
        std::cout << "Error: " << args << std::endl; \
    }while(0)

#define ROS_ERROR_COND(cond, ...)    \
do{ \
    if(!(cond)){    \
        std::cout << "Error: " << __VA_ARGS__ << std::endl; \
    }   \
}while(0)

#define ROS_DEBUG(...)   \
    do{ \
        std::cout << "DEBUG: " << printf(__VA_ARGS__) << std::endl; \
    }while(0)

#define ROS_DEBUG_STREAM(args)   \
    do{ \
        std::cout << "DEBUG: " << args << std::endl; \
    }while(0)

#define ROS_WARN_COND(cond, ...)    \
    do{ \
        if(!(cond)){    \
          std::cout << "Warn: " << __VA_ARGS__ << std::endl; \
        }   \
    }while(0)

#define mDeg2rad(deg)   deg/180.0*3.14159265359
#define mRad2deg(rad)   rad/3.14159265359*180.0
#define gkmaPath = "/home/cwz/Documents/HectoDrone/ground-station/grid-service/build/lib";


#endif
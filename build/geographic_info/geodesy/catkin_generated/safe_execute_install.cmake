execute_process(COMMAND "/home/bhargav-u20/drone_ws/build/geographic_info/geodesy/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/bhargav-u20/drone_ws/build/geographic_info/geodesy/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

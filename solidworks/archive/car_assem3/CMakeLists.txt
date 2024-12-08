cmake_minimum_required(VERSION 3.3)
project(car_assem3)

find_package(ament_cmake REQUIRED)

install (DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()

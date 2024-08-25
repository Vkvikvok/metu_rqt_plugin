cmake_minimum_required(VERSION 3.5)
project(metu_rqt_plugin)

# Eğer ROS2 kullanıyorsanız, aşağıdaki satırları da ekleyin.
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Bu, ROS2'deki Python modülleriyle uyumlu olarak Python kurulumunu ayarlar
ament_python_install_package(${PROJECT_NAME})

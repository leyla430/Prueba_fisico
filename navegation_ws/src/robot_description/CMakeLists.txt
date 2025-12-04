cmake_minimum_required(VERSION 3.8)
project(robot_description)

# Recomendado para diagnósticos, pero opcional
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



find_package(ament_cmake REQUIRED)
# find_package(xacro REQUIRED)  # <-- solo si vas a invocar xacro desde cmake (no necesario aquí)



# Instala TODO lo que se resuelve con package://robot_description/...
install(DIRECTORY
  launch
  urdf
  rviz
  meshes
  world
  config
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".git*" EXCLUDE
  PATTERN "*~"    EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)




install(DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY meshes
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY world
        DESTINATION share/${PROJECT_NAME})



install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)


install(PROGRAMS


  scripts/robot_height_scan.py


  DESTINATION lib/${PROJECT_NAME}
)




ament_package()

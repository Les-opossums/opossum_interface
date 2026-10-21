cmake_minimum_required(VERSION 3.8)
project(opossum_interfaces NONE)

# Package ROS 2 : installe le code DEJA genere (generated/), sans lancer gen.py.
# Le test `test_protocol` echoue si generated/ n'est pas a jour avec protocol.yaml.

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# En-tete C++ : #include <opossum_interfaces/eth_protocol.hpp>
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/generated/cpp>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
install(DIRECTORY generated/cpp/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})

# Module Python : from opossum_interfaces import eth_protocol
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR generated/python/${PROJECT_NAME})

# ICD et protocole texte historique (lus par opossum_comm/comm.py et opossum_simu/zynq_simu.py)
install(FILES protocol.yaml com_msgs.yaml format_msgs.yaml version.yaml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_protocol tests/test_protocol.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()

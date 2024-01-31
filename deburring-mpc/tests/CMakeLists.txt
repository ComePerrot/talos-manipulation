set(${PROJECT_NAME}_FACTORY_TEST
    factory/robot_designer.hpp
    factory/robot_designer.cpp)

add_library(${PROJECT_NAME}_unittest SHARED ${${PROJECT_NAME}_FACTORY_TEST})
target_link_libraries(${PROJECT_NAME}_unittest PUBLIC ${PROJECT_NAME})
target_compile_definitions(${PROJECT_NAME}_unittest
                           PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

add_unit_test(test_OCP test_OCP.cpp)
target_link_libraries(test_OCP PUBLIC ${PROJECT_NAME}_unittest yaml-cpp)
target_compile_definitions(test_OCP
                           PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

add_unit_test(test_MPC test_MPC.cpp)
target_link_libraries(test_MPC PUBLIC ${PROJECT_NAME}_unittest yaml-cpp)
target_compile_definitions(test_MPC
                           PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")


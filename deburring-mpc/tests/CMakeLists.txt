add_unit_test(test_OCP test_OCP.cpp)
target_link_libraries(test_OCP PUBLIC ${PROJECT_NAME} yaml-cpp)
target_compile_definitions(test_OCP
                           PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

add_unit_test(test_MPC test_MPC.cpp)
target_link_libraries(test_MPC PUBLIC ${PROJECT_NAME} yaml-cpp)
target_compile_definitions(test_MPC
                           PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")


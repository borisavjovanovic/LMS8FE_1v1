add_executable(LMS8FE_USB_example LMS8FE_USB_example.cpp)
set_target_properties(LMS8FE_USB_example PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(LMS8FE_USB_example LMS8FEAPI)

add_executable(LMS8FE_LMS8_example LMS8FE_LMS8_example.cpp)
set_target_properties(LMS8FE_LMS8_example PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(LMS8FE_LMS8_example LMS8FEAPI LMS8API)

add_executable(LMS8FE_example LMS8FE_example.cpp)
set_target_properties(LMS8FE_example PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(LMS8FE_example LMS8FEAPI LMS8API)
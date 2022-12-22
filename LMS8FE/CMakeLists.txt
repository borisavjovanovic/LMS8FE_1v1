set(wxWidgets_USE_DEBUG OFF)
set(wxWidgets_USE_UNICODE ON)
set(wxWidgets_USE_UNIVERSAL OFF)
set(wxWidgets_USE_STATIC ON)
find_package(wxWidgets COMPONENTS richtext xml html xrc adv aui core base gl)

include(${wxWidgets_USE_FILE})

message(STATUS "wxWidgets_DEFINITIONS: ${wxWidgets_DEFINITIONS}")
message(STATUS "wxWidgets_INCLUDE_DIRS: ${wxWidgets_INCLUDE_DIRS}")
message(STATUS "wxWidgets_LIBRARIES: ${wxWidgets_LIBRARIES}")

add_subdirectory(LMS8001)
include_directories(LMS8001)

set(LMS8FEAPI_SOURCES
    LMS8FE_cmd.cpp
    LMS8FE_api.cpp
    LMS8FE_Device.cpp
#milans 221128	
	LMS8001/API/lms8_device.cpp
	LMS8001/lms8001/connectionManager/ConnectionManager.cpp
	LMS8001/lms8001/connectionManager/ConnectionCOM.cpp
	LMS8001/lms8001/LMS8001.cpp
	LMS8001/lms8001/lmsComms.cpp
	LMS8001/lms8001/LMS8001_RegistersMap.cpp
	LMS8001/lms8001/LMS8001_parameters.cpp    
)

SET(LMS8FEAPI_INCLUDES
    external/cpp-feather-ini-parser
#    LMS8001
#milans 221128
	LMS8001/ADF4002
	LMS8001/API
	LMS8001/lms8001
)

#if(MSVC)
#    list(APPEND LMS8FEAPI_SOURCES external/termiWin/termiWin.c)
#    list(APPEND LMS8FEAPI_INCLUDES external/termiWin)    
#endif()

add_library(LMS8FEAPI SHARED ${LMS8FEAPI_SOURCES})
target_include_directories(LMS8FEAPI PUBLIC ${LMS8FEAPI_INCLUDES})

set(LMS8FE_SOURCES
    LMS8FE_wxgui.cpp
    LMS8FE_gui.cpp
    LMS8FE_app.cpp
    LMS8FE_cmd.cpp    
#    LMS8001/lms8suiteApp.cpp
)

add_executable(LMS8FE ${LMS8FE_SOURCES} resources/resource.rc)

set(BINARY_OUTPUT_DIR "${CMAKE_BINARY_DIR}/bin")

set_target_properties(LMS8FE PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${BINARY_OUTPUT_DIR})

LINK_DIRECTORIES(/usr/local/lib)

target_link_libraries(
    LMS8FEAPI
    LimeSuite
    LMS8001
    lms8suite     
)

target_link_libraries(
    LMS8FE 
    LMS8FEAPI
    LimeSuite
    LMS8001
    lms8suite    
    ${wxWidgets_LIBRARIES}    
)

target_compile_definitions(LMS8FE PRIVATE WIN32_LEAN_AND_MEAN)

if (MSVC)
    set_property(TARGET LMS8FE APPEND PROPERTY LINK_FLAGS /SUBSYSTEM:WINDOWS)
endif(MSVC)

set_target_properties(LMS8FEAPI PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_subdirectory(examples)

install(TARGETS LMS8FEAPI
    LIBRARY DESTINATION lib${LIB_SUFFIX} # .so file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)


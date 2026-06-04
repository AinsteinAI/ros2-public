cmake_minimum_required(VERSION 3.20)
project(radar_muniu)

# ===================== Basic Configuration (Standardization + Cross-Platform) =====================
# C++ standard (C++17 recommended by ROS2, effective globally)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ROS2 scoped header installation (eliminate Kilted Kaiju warnings)
set(USE_SCOPED_HEADER_INSTALL_DIR ON CACHE BOOL "" FORCE)
set(AMENT_AUTO_SKIP_HEADER_INSTALL ON CACHE BOOL "" FORCE)

# Third-party library root directory (unified management)
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

# ===================== Windows-Specific Configuration (MSVC) =====================
if(MSVC)
    # C++ compilation options (enforce C++17, disable extensions, turn off warnings)
    add_compile_options(/std:c++17 /permissive- /Zc:__cplusplus /W0)
    # Windows system macro definitions (simplification + compatibility)
    add_compile_definitions(
        _WIN32_WINNT=0x0601
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
        _CRT_SECURE_NO_WARNINGS
        _DISABLE_EXTENDED_ALIGNED_STORAGE
    )
    # VS toolset/SDK version (FORCE to ensure effectiveness)
    set(CMAKE_GENERATOR_TOOLSET "v143" CACHE STRING "MSVC toolset version" FORCE)
    set(CMAKE_SYSTEM_VERSION "10.0.20348.0" CACHE STRING "Windows SDK version" FORCE)
endif()

# ===================== Global Compilation Macros (ABI Compatibility) =====================
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)

# ===================== ROS2 Core Dependencies =====================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ===================== Third-Party Library Lookup (Standardization + Cross-Platform) =====================
# 1. PCAN library (cross-platform find_library, replace hard-coded EXISTS)
if(WIN32)
    set(PCAN_LIB_NAMES "PCANBasic")
    set(PCAN_SEARCH_PATHS "${THIRD_PARTY_DIR}/PCAN-Basic/x64/VC_LIB")
    set(PCAN_INC_PATH "${THIRD_PARTY_DIR}/PCAN-Basic/Include")
else()
    set(PCAN_LIB_NAMES "pcanbasic" "libpcanbasic")
    set(PCAN_SEARCH_PATHS "${THIRD_PARTY_DIR}/PCAN-Basic/x64/lib")
    set(PCAN_INC_PATH "${THIRD_PARTY_DIR}/PCAN-Basic/include")
endif()
find_library(
    PCAN_LIB_FILE
    NAMES ${PCAN_LIB_NAMES}
    PATHS ${PCAN_SEARCH_PATHS}
    NO_DEFAULT_PATH
)
# PCAN library status handling
if(PCAN_LIB_FILE)
    message(STATUS "Found PCAN library: ${PCAN_LIB_FILE}")
    set(PCAN_FOUND TRUE)
else()
    message(WARNING "PCAN library not found, CAN support disabled")
    set(PCAN_FOUND FALSE)
endif()

# 2. dbcppp library (standardized find_library, supports Windows/Linux)
if(WIN32)
    set(DBCPPP_LIB_NAMES "dbcppp")
    set(DBCPPP_SEARCH_PATHS "${THIRD_PARTY_DIR}/dbcpp/lib")
else()
    set(DBCPPP_LIB_NAMES "dbcppp" "libdbcppp")
    set(DBCPPP_SEARCH_PATHS "${THIRD_PARTY_DIR}/dbcpp/lib")
endif()
find_library(
    DBCPPP_LIB_FILE
    NAMES ${DBCPPP_LIB_NAMES}
    PATHS ${DBCPPP_SEARCH_PATHS}
    NO_DEFAULT_PATH
)
# dbcppp library status handling
if(DBCPPP_LIB_FILE)
    message(STATUS "Found dbcppp library: ${DBCPPP_LIB_FILE}")
    set(DBCPPP_FOUND TRUE)
else()
    message(WARNING "dbcppp library not found, DBC parsing disabled")
    set(DBCPPP_FOUND FALSE)
endif()

# 3. vsomeip3 library (batch lookup + unified status management)
set(VSOMEIP_FOUND TRUE)
set(VSOMEIP_LIBS "")
# Define vsomeip library list (cross-platform names)
if(WIN32)
    set(VSOMEIP_LIB_LIST "vsomeip3" "vsomeip3-cfg" "vsomeip3-sd")
else()
    set(VSOMEIP_LIB_LIST "vsomeip3" "vsomeip3-sd")
endif()
# Batch lookup vsomeip libraries
foreach(lib ${VSOMEIP_LIB_LIST})
    find_library(
        VSOMEIP_${lib}_FILE
        NAMES ${lib}
        PATHS "${THIRD_PARTY_DIR}/vsomeip/lib"
        NO_DEFAULT_PATH
    )
    if(VSOMEIP_${lib}_FILE)
        list(APPEND VSOMEIP_LIBS ${VSOMEIP_${lib}_FILE})
        message(STATUS "Found vsomeip3 library: ${VSOMEIP_${lib}_FILE}")
    else()
        message(WARNING "vsomeip3 library ${lib} not found")
        set(VSOMEIP_FOUND FALSE)
    endif()
endforeach()
# Final vsomeip3 status handling
if(NOT VSOMEIP_FOUND)
    message(WARNING "Some vsomeip3 libraries missing, SOME/IP support disabled")
    set(VSOMEIP_LIBS "")
endif()

# ===================== Include Directories (Unified + Conditional Addition) =====================
set(INCLUDE_DIRS
    include
    include/${PROJECT_NAME}  # ROS2 scoped header directory
    ${THIRD_PARTY_DIR}/asio/include
)
# Conditionally add third-party library headers
if(PCAN_FOUND)
    list(APPEND INCLUDE_DIRS ${PCAN_INC_PATH})
endif()
if(DBCPPP_FOUND)
    list(APPEND INCLUDE_DIRS "${THIRD_PARTY_DIR}/dbcpp/include")
endif()
if(VSOMEIP_FOUND)
    list(APPEND INCLUDE_DIRS "${THIRD_PARTY_DIR}/vsomeip/include")
    message(STATUS "Added vsomeip include dir: ${THIRD_PARTY_DIR}/vsomeip/include")
endif()
# Unified setting of include directories
include_directories(${INCLUDE_DIRS})

# ===================== Library Compilation (Modularization + Cross-Platform) =====================
# 1. Communication library (comm_lib)
ament_auto_add_library(comm_lib
    src/udp_comm.cpp
    src/peakcan_comm.cpp
    src/socketcan_comm.cpp
    src/someip_client.cpp
)
# Link third-party libraries (conditional + batch)
target_link_libraries(comm_lib
    $<$<BOOL:${PCAN_FOUND}>:${PCAN_LIB_FILE}>
    $<$<BOOL:${DBCPPP_FOUND}>:${DBCPPP_LIB_FILE}>
    $<$<BOOL:${VSOMEIP_FOUND}>:${VSOMEIP_LIBS}>
)
# Windows system dependencies
if(WIN32)
    target_link_libraries(comm_lib ws2_32)
endif()

# 2. Model library (model_lib)
ament_auto_add_library(model_lib
    src/radar_model.cpp
)

# 3. Parser library (parser_lib)
ament_auto_add_library(parser_lib
    src/radar_parser.cpp
)
target_link_libraries(parser_lib comm_lib model_lib)

# ===================== Main Node Compilation (Including Windows Resource Files) =====================
ament_auto_add_executable(radar_muniu_node
    src/radar_muniu_node.cpp
)
# Windows resource file (version.rc)
if(WIN32 AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/res/version.rc)
    target_sources(radar_muniu_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/res/version.rc)
    target_link_libraries(radar_muniu_node user32.lib)
    message(STATUS "Added Windows resource file: res/version.rc")
endif()
# Link parser library
target_link_libraries(radar_muniu_node parser_lib)

# ===================== Installation Configuration (ROS2 Standard Paths) =====================
# 1. Scoped headers (ROS2 Kilted Kaiju compatibility)
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# 2. Config/Launch/RViz files (ROS2 standard paths)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# 3. Third-party library headers (conditional installation + standard paths)
if(DBCPPP_FOUND)
    install(DIRECTORY ${THIRD_PARTY_DIR}/dbcpp/include/
        DESTINATION include/dbcpp
    )
	if(WIN32)
		set(DBCPPP_TARGET_DLL "${THIRD_PARTY_DIR}/dbcpp/bin/dbcppp.dll")
		install(
			FILES "${DBCPPP_TARGET_DLL}"
			DESTINATION lib/${PROJECT_NAME}/
		)
	endif()
endif()

if(PCAN_FOUND)
	if(WIN32)
		set(PCAN_TARGET_DLL "${THIRD_PARTY_DIR}/PCAN-Basic/x64/PCANBasic.dll")
		install(
			FILES "${PCAN_TARGET_DLL}"
			DESTINATION lib/${PROJECT_NAME}/
		)
	endif()
endif()


if(VSOMEIP_FOUND)
    install(DIRECTORY ${THIRD_PARTY_DIR}/vsomeip/include/
        DESTINATION include/vsomeip
    )
	if(WIN32)
		file(GLOB VSOMEIP_TARGET_FILES
			"${THIRD_PARTY_DIR}/vsomeip/bin/vsomeip3*.dll"
		)
		foreach(file ${VSOMEIP_TARGET_FILES})
            install(
                FILES "${file}"
                DESTINATION lib/${PROJECT_NAME}/
            )
            message(STATUS "Will install vsomeip file: ${file} to lib/${PROJECT_NAME}/")
        endforeach()
	endif()
    message(STATUS "vsomeip headers will be installed to include/vsomeip")
endif()

# ===================== Final ROS2 Package Configuration =====================
ament_auto_package()
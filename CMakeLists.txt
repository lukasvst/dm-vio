SET(PROJECT_NAME DMVIO)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 3.1)


IF(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()
set(BUILD_TYPE RelWithDebInfo)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# required libraries
find_package(SuiteParse REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread filesystem chrono serialization date_time timer)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(realsense2 QUIET)

IF(${Boost_VERSION} GREATER_EQUAL 106500)
    message("Building with stacktrace support.")
	set(STACKTRACE_LIBRARIES dl)
	set(STACKTRACE_DEFINES "-DSTACKTRACE -DBOOST_STACKTRACE_USE_ADDR2LINE")
ENDIF()

# optional libraries
find_package(LibZip QUIET)
find_package(Pangolin 0.2 QUIET)
find_package(OpenCV QUIET)


IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    MESSAGE(Mac OS X specific code)
    include_directories(/usr/local/include)
    link_directories(/usr/local/lib)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


# flags
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS
    "${SSE_FLAGS}"
)
set(CMAKE_CXX_FLAGS_DEBUG
	"-O2 -g -fno-omit-frame-pointer -DEIGEN_INITIALIZE_MATRICES_WITH_NAN -DDEBUG ${STACKTRACE_DEFINES}"
)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO
	"-O3 -g -fno-omit-frame-pointer ${STACKTRACE_DEFINES}"
)
set(CMAKE_CXX_FLAGS_RELEASE
	"-O3 -DNDEBUG"
)

set(CMAKE_CXX_STANDARD 14)

if (MSVC)
     set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif (MSVC)

set(DSO_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src/dso)

# DSO Source files
set(dso_SOURCE_FILES
        ${DSO_SOURCE_DIR}/FullSystem/FullSystem.cpp
        ${DSO_SOURCE_DIR}/FullSystem/FullSystemOptimize.cpp
        ${DSO_SOURCE_DIR}/FullSystem/FullSystemOptPoint.cpp
        ${DSO_SOURCE_DIR}/FullSystem/FullSystemDebugStuff.cpp
        ${DSO_SOURCE_DIR}/FullSystem/FullSystemMarginalize.cpp
        ${DSO_SOURCE_DIR}/FullSystem/Residuals.cpp
        ${DSO_SOURCE_DIR}/FullSystem/CoarseTracker.cpp
        ${DSO_SOURCE_DIR}/FullSystem/CoarseInitializer.cpp
        ${DSO_SOURCE_DIR}/FullSystem/ImmaturePoint.cpp
        ${DSO_SOURCE_DIR}/FullSystem/HessianBlocks.cpp
        ${DSO_SOURCE_DIR}/FullSystem/PixelSelector2.cpp
		${DSO_SOURCE_DIR}/OptimizationBackend/EnergyFunctional.cpp
		${DSO_SOURCE_DIR}/OptimizationBackend/AccumulatedTopHessian.cpp
		${DSO_SOURCE_DIR}/OptimizationBackend/AccumulatedSCHessian.cpp
		${DSO_SOURCE_DIR}/OptimizationBackend/EnergyFunctionalStructs.cpp
		${DSO_SOURCE_DIR}/util/settings.cpp
		${DSO_SOURCE_DIR}/util/Undistort.cpp
		${DSO_SOURCE_DIR}/util/globalCalib.cpp
		)

set(dmvio_SOURCE_FILES
		src/IMU/IMUIntegration.cpp
		src/GTSAMIntegration/Sim3GTSAM.cpp
        src/IMUInitialization/GravityInitializer.cpp
		src/IMU/IMUTypes.cpp
		src/IMU/IMUSettings.cpp
		src/util/TimeMeasurement.cpp
		src/util/SettingsUtil.cpp
		src/GTSAMIntegration/BAGTSAMIntegration.cpp
		src/IMU/CoarseIMULogic.cpp
		src/IMU/BAIMULogic.cpp
		src/GTSAMIntegration/PoseTransformation.cpp
		src/GTSAMIntegration/Marginalization.cpp
		src/GTSAMIntegration/PoseTransformationIMU.cpp
		src/GTSAMIntegration/PoseTransformationFactor.cpp
        src/IMUInitialization/CoarseIMUInitOptimizer.cpp
        src/IMUInitialization/IMUInitializer.cpp
		src/IMU/IMUUtils.cpp
        src/IMUInitialization/IMUInitSettings.cpp
		src/GTSAMIntegration/GTSAMUtils.cpp
		src/GTSAMIntegration/DelayedMarginalization.cpp
        src/IMUInitialization/PoseGraphBundleAdjustment.cpp
        src/GTSAMIntegration/FEJValues.cpp
        src/IMUInitialization/IMUInitializerStates.cpp
        src/IMUInitialization/IMUInitializerLogic.cpp
        src/IMUInitialization/IMUInitializerTransitions.cpp
		src/GTSAMIntegration/AugmentedScatter.cpp
		src/live/FrameContainer.cpp
		src/live/IMUInterpolator.cpp
        src/util/MainSettings.cpp
        src/live/FrameSkippingStrategy.cpp
		src/live/DatasetSaver.cpp
		)


include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/src/dso
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
  ${PROJECT_SOURCE_DIR}/thirdparty/sse2neon
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
) 


# decide if we have pangolin
if (Pangolin_FOUND)
	message("--- found PANGOLIN, compiling with pangolin library.")
	include_directories( ${Pangolin_INCLUDE_DIRS} ) 
	set(dso_pangolin_SOURCE_FILES
			${DSO_SOURCE_DIR}/IOWrapper/Pangolin/KeyFrameDisplay.cpp
			${DSO_SOURCE_DIR}/IOWrapper/Pangolin/PangolinDSOViewer.cpp
			${DSO_SOURCE_DIR}/IOWrapper/Pangolin/FollowCamMode.cpp)
	set(HAS_PANGOLIN 1)
else ()
	message("--- could not find PANGOLIN, not compiling with pangolin library.")
	message("    this means there will be no 3D display / GUI available for dmvio_dataset.")
	set(dso_pangolin_SOURCE_FILES )
	set(HAS_PANGOLIN 0)
endif ()

# decide if we have openCV
if (OpenCV_FOUND)
	message("--- found OpenCV, compiling with opencv library.")
	include_directories( ${OpenCV_INCLUDE_DIRS} )
	set(dso_opencv_SOURCE_FILES
			${DSO_SOURCE_DIR}/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
			${DSO_SOURCE_DIR}/IOWrapper/OpenCV/ImageRW_OpenCV.cpp)
	set(HAS_OPENCV 1)
else ()
	message("--- could not find OpenCV, not compiling with opencv library.")
	message("    this means there will be no image display, and image read / load functionality.")
	set(dso_opencv_SOURCE_FILES
			${DSO_SOURCE_DIR}/IOWrapper/ImageDisplay_dummy.cpp
			${DSO_SOURCE_DIR}/IOWrapper/ImageRW_dummy.cpp)
	set(HAS_OPENCV 0)
endif ()

# decide if we have ziplib.
if (LIBZIP_LIBRARY)
	message("--- found ziplib (${LIBZIP_VERSION}), compiling with zip capability.")
	add_definitions(-DHAS_ZIPLIB=1)
	include_directories( ${LIBZIP_INCLUDE_DIR_ZIP} ${LIBZIP_INCLUDE_DIR_ZIPCONF} ) 
else()
	message("--- not found ziplib (${LIBZIP_LIBRARY}), compiling without zip capability.")
	set(LIBZIP_LIBRARY "")
endif()

# compile main library.
include_directories( ${CSPARSE_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR} ${YAML_CPP_INCLUDE_DIR})
add_library(dmvio ${dso_SOURCE_FILES} ${dmvio_SOURCE_FILES} ${dso_opencv_SOURCE_FILES} ${dso_pangolin_SOURCE_FILES})


if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin") # OSX
    set(BOOST_THREAD_LIBRARY boost_thread-mt)
else()
    set(BOOST_THREAD_LIBRARY boost_thread)
endif()


# build main executable (only if we have both OpenCV and Pangolin)
if (OpenCV_FOUND AND Pangolin_FOUND)
	message("--- compiling dmvio_dataset.")
	add_executable(dmvio_dataset ${PROJECT_SOURCE_DIR}/src/main_dmvio_dataset.cpp)
	set(DMVIO_LINKED_LIBRARIES boost_system cxsparse ${BOOST_THREAD_LIBRARY} ${LIBZIP_LIBRARY} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} gtsam ${YAML_CPP_LIBRARIES} ${STACKTRACE_LIBRARIES})
    target_link_libraries(dmvio_dataset dmvio ${DMVIO_LINKED_LIBRARIES})

	if(realsense2_FOUND)
		message("--- compiling dmvio_t265.")
		set(dmvio_t265_SOURCE_FILES ${PROJECT_SOURCE_DIR}/src/live/RealsenseT265.cpp)
		add_executable(dmvio_t265 ${PROJECT_SOURCE_DIR}/src/main_dmvio_t265.cpp ${dmvio_t265_SOURCE_FILES})
		target_link_libraries(dmvio_t265 dmvio ${DMVIO_LINKED_LIBRARIES} realsense2::realsense2)
	endif()

else()
	message("--- not building dmvio_dataset, since either don't have openCV or Pangolin.")
endif()

add_subdirectory(test)

SET(PROJECT_NAME DMVIO)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)


set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# required libraries
#find_package(SuiteParse REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread filesystem chrono serialization date_time timer stacktrace)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Sophus REQUIRED)

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

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions("-DENABLE_SSE")

if (MSVC)
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /MP /bigobj")
	add_definitions("-DNOMINMAX")
	add_definitions("-DLEAN_AND_MEAN")
else()
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
endif()

set(DSO_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src/dso)

# DSO Source files
set(dso_SOURCE_FILES
		${DSO_SOURCE_DIR}/OptimizationBackend/AccumulatedSCHessian.cpp
		${DSO_SOURCE_DIR}/OptimizationBackend/AccumulatedTopHessian.cpp
		${DSO_SOURCE_DIR}/OptimizationBackend/EnergyFunctional.cpp
		${DSO_SOURCE_DIR}/OptimizationBackend/EnergyFunctionalStructs.cpp
		${DSO_SOURCE_DIR}/util/Undistort.cpp
		${DSO_SOURCE_DIR}/util/globalCalib.cpp
		${DSO_SOURCE_DIR}/util/settings.cpp
        ${DSO_SOURCE_DIR}/FullSystem/CoarseInitializer.cpp
        ${DSO_SOURCE_DIR}/FullSystem/CoarseTracker.cpp
        ${DSO_SOURCE_DIR}/FullSystem/FullSystem.cpp
        ${DSO_SOURCE_DIR}/FullSystem/FullSystemDebugStuff.cpp
        ${DSO_SOURCE_DIR}/FullSystem/FullSystemMarginalize.cpp
        ${DSO_SOURCE_DIR}/FullSystem/FullSystemOptPoint.cpp
        ${DSO_SOURCE_DIR}/FullSystem/FullSystemOptimize.cpp
        ${DSO_SOURCE_DIR}/FullSystem/HessianBlocks.cpp
        ${DSO_SOURCE_DIR}/FullSystem/ImmaturePoint.cpp
        ${DSO_SOURCE_DIR}/FullSystem/PixelSelector2.cpp
        ${DSO_SOURCE_DIR}/FullSystem/Residuals.cpp
		
		${DSO_SOURCE_DIR}/FullSystem/CoarseInitializer.h
		${DSO_SOURCE_DIR}/FullSystem/CoarseTracker.h
		${DSO_SOURCE_DIR}/FullSystem/FullSystem.h
		${DSO_SOURCE_DIR}/FullSystem/HessianBlocks.h
		${DSO_SOURCE_DIR}/FullSystem/ImmaturePoint.h
		${DSO_SOURCE_DIR}/FullSystem/PixelSelector.h
		${DSO_SOURCE_DIR}/FullSystem/PixelSelector2.h
		${DSO_SOURCE_DIR}/FullSystem/ResidualProjections.h
		${DSO_SOURCE_DIR}/FullSystem/Residuals.h
		${DSO_SOURCE_DIR}/IOWrapper/ImageDisplay.h
		${DSO_SOURCE_DIR}/IOWrapper/ImageRW.h
		${DSO_SOURCE_DIR}/IOWrapper/Output3DWrapper.h
		${DSO_SOURCE_DIR}/IOWrapper/OutputWrapper/SampleOutputWrapper.h
		${DSO_SOURCE_DIR}/OptimizationBackend/AccumulatedSCHessian.h
		${DSO_SOURCE_DIR}/OptimizationBackend/AccumulatedTopHessian.h
		${DSO_SOURCE_DIR}/OptimizationBackend/EnergyFunctional.h
		${DSO_SOURCE_DIR}/OptimizationBackend/EnergyFunctionalStructs.h
		${DSO_SOURCE_DIR}/OptimizationBackend/MatrixAccumulators.h
		${DSO_SOURCE_DIR}/OptimizationBackend/RawResidualJacobian.h
		${DSO_SOURCE_DIR}/util/DatasetReader.h
		${DSO_SOURCE_DIR}/util/FrameShell.h
		${DSO_SOURCE_DIR}/util/ImageAndExposure.h
		${DSO_SOURCE_DIR}/util/IndexThreadReduce.h
		${DSO_SOURCE_DIR}/util/MinimalImage.h
		${DSO_SOURCE_DIR}/util/NumType.h
		${DSO_SOURCE_DIR}/util/Undistort.h
		${DSO_SOURCE_DIR}/util/globalCalib.h
		${DSO_SOURCE_DIR}/util/globalFuncs.h
		${DSO_SOURCE_DIR}/util/nanoflann.h
		${DSO_SOURCE_DIR}/util/settings.h
		)

set(dmvio_SOURCE_FILES
		src/GTSAMIntegration/AugmentedScatter.cpp
		src/GTSAMIntegration/BAGTSAMIntegration.cpp
		src/GTSAMIntegration/DelayedMarginalization.cpp
		src/GTSAMIntegration/GTSAMUtils.cpp
		src/GTSAMIntegration/Marginalization.cpp
		src/GTSAMIntegration/PoseTransformation.cpp
		src/GTSAMIntegration/PoseTransformationFactor.cpp
		src/GTSAMIntegration/PoseTransformationIMU.cpp
		src/GTSAMIntegration/Sim3GTSAM.cpp
		src/IMU/BAIMULogic.cpp
		src/IMU/CoarseIMULogic.cpp
		src/IMU/IMUIntegration.cpp
		src/IMU/IMUSettings.cpp
		src/IMU/IMUTypes.cpp
		src/IMU/IMUUtils.cpp
		src/util/SettingsUtil.cpp
		src/util/TimeMeasurement.cpp
        src/GTSAMIntegration/FEJValues.cpp
        src/IMUInitialization/CoarseIMUInitOptimizer.cpp
        src/IMUInitialization/GravityInitializer.cpp
        src/IMUInitialization/IMUInitSettings.cpp
        src/IMUInitialization/IMUInitializer.cpp
        src/IMUInitialization/IMUInitializerLogic.cpp
        src/IMUInitialization/IMUInitializerStates.cpp
        src/IMUInitialization/IMUInitializerTransitions.cpp
        src/IMUInitialization/PoseGraphBundleAdjustment.cpp
		
		src/GTSAMIntegration/AugmentedScatter.hpp
		src/GTSAMIntegration/BAGTSAMIntegration.h
		src/GTSAMIntegration/DelayedMarginalization.h
		src/GTSAMIntegration/ExtUtils.h
		src/GTSAMIntegration/FEJNoiseModelFactor.h
		src/GTSAMIntegration/FEJValues.h
		src/GTSAMIntegration/GTSAMUtils.h
		src/GTSAMIntegration/Marginalization.h
		src/GTSAMIntegration/PoseTransformation.h
		src/GTSAMIntegration/PoseTransformationFactor.h
		src/GTSAMIntegration/PoseTransformationIMU.h
		src/GTSAMIntegration/Sim3GTSAM.h
		src/IMUInitialization/CoarseIMUInitOptimizer.h
		src/IMUInitialization/GravityInitializer.h
		src/IMUInitialization/IMUInitSettings.h
		src/IMUInitialization/IMUInitStateChanger.h
		src/IMUInitialization/IMUInitializer.h
		src/IMUInitialization/IMUInitializerLogic.h
		src/IMUInitialization/IMUInitializerStates.h
		src/IMUInitialization/IMUInitializerTransitions.h
		src/IMUInitialization/PoseGraphBundleAdjustment.h
		src/IMU/BAIMULogic.h
		src/IMU/CoarseIMULogic.h
		src/IMU/IMUIntegration.hpp
		src/IMU/IMUSettings.h
		src/IMU/IMUTypes.h
		src/IMU/IMUUtils.h
		src/util/GTData.hpp
		src/util/SettingsUtil.h
		src/util/TimeMeasurement.h
		)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/src/dso
) 

# decide if we have pangolin
if (Pangolin_FOUND)
	message("--- found PANGOLIN, compiling with pangolin library.")
	include_directories( ${Pangolin_INCLUDE_DIRS} ) 
	set(dso_pangolin_SOURCE_FILES
			${DSO_SOURCE_DIR}/IOWrapper/Pangolin/KeyFrameDisplay.cpp
			${DSO_SOURCE_DIR}/IOWrapper/Pangolin/PangolinDSOViewer.cpp
			${DSO_SOURCE_DIR}/IOWrapper/Pangolin/KeyFrameDisplay.h
			${DSO_SOURCE_DIR}/IOWrapper/Pangolin/PangolinDSOViewer.h
			)
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
target_link_libraries(dmvio PUBLIC
	Eigen3::Eigen
	GTSAM::gtsam
	Boost::system Boost::thread Boost::filesystem Boost::chrono Boost::serialization Boost::date_time Boost::timer
	#Boost::stacktrace
	yaml-cpp::yaml-cpp
	Sophus::Sophus
	opencv::opencv_highgui
	)
#target_compile_definitions(dmvio PRIVATE
#	STACKTRACE
#	BOOST_STACKTRACE_USE_ADDR2LINE
#)
if(MSVC)
	target_compile_options(dmvio PRIVATE
		"/wd4244;/wd4267;/wd4305;/wd4996"
	)
endif()

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin") # OSX
    set(BOOST_THREAD_LIBRARY boost_thread-mt)
else()
    set(BOOST_THREAD_LIBRARY boost_thread)
endif()


# build main executable (only if we have both OpenCV and Pangolin)
if (OpenCV_FOUND AND Pangolin_FOUND)
	message("--- compiling dmvio_dataset.")
	add_executable(dmvio_dataset ${PROJECT_SOURCE_DIR}/src/main_dmvio_dataset.cpp)
	target_link_libraries(dmvio_dataset
		dmvio
		${STACKTRACE_LIBRARIES} ${LIBZIP_LIBRARY}
		${Pangolin_LIBRARIES}
		)
else()
	message("--- not building dmvio_dataset, since either don't have openCV or Pangolin.")
endif()

add_subdirectory(test)

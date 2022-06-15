project(Tests)

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/googletest/CMakeLists.txt")
    add_subdirectory(googletest)
    include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

    add_executable(Google_Tests_run test_PoseTransformationFactor.cpp test_IMUInterpolator.cpp)
    target_link_libraries(Google_Tests_run gtest gtest_main dmvio ${DMVIO_LINKED_LIBRARIES})
endif()
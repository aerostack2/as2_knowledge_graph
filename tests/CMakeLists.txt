#Gtest

#find dependencies

set(TEST_DEPENDENCIES
    ament_cmake_gtest
)

foreach(DEPENDENCY ${TEST_DEPENDENCIES})
    find_package(${DEPENDENCY} REQUIRED)
endforeach()

file(GLOB TEST_SOURCE "*_gtest.cpp")

foreach(TEST_FILE ${TEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    #Add Gtest executable and link necessary dependencies
    ament_add_gtest(${TEST_NAME} ${TEST_SOURCE})
    ament_target_dependencies(${TEST_NAME} ${PROJECT_DEPENDENCIES})
    target_link_libraries(${TEST_NAME} gtest_main ${PROJECT_NAME})
endforeach()


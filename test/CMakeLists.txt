project(VoxelEngineTest)

file(GLOB_RECURSE sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

find_package(GTest)

add_executable(VoxelEngineTest ${sources})

target_link_libraries(VoxelEngineTest PRIVATE VoxelEngineSrc GTest::gtest_main)

# HACK: copy res to test/ folder for fixing problem compatibility MultiConfig
# and non MultiConfig builds. Delete in future and use only root res folder Also
# this resolve problem with ctests, because it set cwd to
# ${CMAKE_CURRENT_BINARY_DIR}
add_custom_command(
    TARGET VoxelEngineTest
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
            ${CMAKE_SOURCE_DIR}/res ${CMAKE_CURRENT_BINARY_DIR}/res)

include(GoogleTest)
gtest_add_tests(TARGET VoxelEngineTest SOURCES ${sources})

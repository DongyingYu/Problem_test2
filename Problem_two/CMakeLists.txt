cmake_minimum_required( VERSION 2.8 )
project( SyncInter )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )



add_library(${PROJECT_NAME} SHARED
Sync_inter.cpp
)

include_directories(
${PROJECT_SOURCE_DIR}/src
)

link_libraries(pthread)

add_executable( test2 main.cpp )
target_link_libraries( test2
         ${PROJECT_NAME}
)

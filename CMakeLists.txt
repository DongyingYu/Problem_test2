cmake_minimum_required( VERSION 2.8 )
project( SyncInter )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )



add_library(${PROJECT_NAME} SHARED
src/Sync_inter.cpp
)

include_directories(
${PROJECT_SOURCE_DIR}/src
)

add_executable( test2 src/main.cpp )
target_link_libraries( test2
         ${PROJECT_NAME}
)

SET(LIB_NAME common)

find_package(gflags REQUIRED)
find_package(glog REQUIRED)

aux_source_directory(. SRCS_LIST)

add_library(${LIB_NAME} ${SRCS_LIST})

target_link_libraries(${LIB_NAME} PUBLIC 
                      "glog" 
                      "gflags")

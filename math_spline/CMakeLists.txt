SET(LIB_NAME math_spline)

aux_source_directory(. SRCS_LIST)

add_library(${LIB_NAME} ${SRCS_LIST})

# target_link_directories(${LIB_NAME} PUBLIC 
#                         "/usr/local/qpOASES/libs")

target_include_directories(${LIB_NAME} PUBLIC 
                          "/usr/local/qpOASES/include" 
                          "/usr/include/eigen3")

target_link_libraries(${LIB_NAME} PUBLIC 
                      "qpOASES")

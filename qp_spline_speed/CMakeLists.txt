SET(LIB_NAME qp_spline_speed)

aux_source_directory(. SRCS_LIST)

add_library(${LIB_NAME} ${SRCS_LIST})

# target_include_directories(${LIB_NAME} PUBLIC 
#                           "/usr/local/qpOASES/include" 
#                           "/usr/include/eigen3")

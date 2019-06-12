# create an executable
ADD_EXECUTABLE (hw2-p1-sol p1-main-sol.cpp)
ADD_EXECUTABLE (hw2-p2-sol p2-main-sol.cpp)
ADD_EXECUTABLE (hw2-p3-sol p3-main-sol.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (hw2-p1-sol ${CS327A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (hw2-p2-sol ${CS327A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (hw2-p3-sol ${CS327A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CS327A_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/hw2_sol)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world_1_puma.urdf world_2_iiwa.urdf world_3_iiwa.urdf DESTINATION ${APP_RESOURCE_DIR})

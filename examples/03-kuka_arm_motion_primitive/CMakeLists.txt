# create an executable
ADD_EXECUTABLE (03-kuka_arm_motion_primitive 03-kuka_arm_motion_primitive.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (03-kuka_arm_motion_primitive
	${SAI2-PRIMITIVES_LIBRARIES}
	${SAI2-PRIMITIVES_EXAMPLES_COMMON_LIBRARIES}
	)

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY world.urdf kuka_iiwa.urdf DESTINATION ${RESOURCE_DIR})

# create an executable
ADD_EXECUTABLE (05-two_pumas 05-two_pumas.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (05-two_pumas
	${SAI2-PRIMITIVES_LIBRARIES}
	${SAI2-PRIMITIVES_EXAMPLES_COMMON_LIBRARIES}
	)

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY puma_gripper.urdf world_puma.urdf object.urdf DESTINATION ${RESOURCE_DIR})

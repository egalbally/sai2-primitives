# create an executable
ADD_EXECUTABLE (05-kuka_screwing 05-kuka_screwing.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (05-kuka_screwing
	${SAI2-PRIMITIVES_LIBRARIES}
	${SAI2-PRIMITIVES_EXAMPLES_COMMON_LIBRARIES}
	)

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY world.urdf kuka_iiwa.urdf bottle.urdf Cap_NoThreads.mtl Cap_NoThreads.obj Cap2_NoThreads.mtl Cap2_NoThreads.obj Bottle_NoThreads.mtl Bottle_NoThreads.obj  DESTINATION ${RESOURCE_DIR})

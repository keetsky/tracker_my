cmake_minimum_required(VERSION 2.8.3)
project(tracker_my)
find_package(catkin REQUIRED COMPONENTS geometry_msgs
					      #orocos_kdl
					roscpp
					roslib
					message_generation
					#tf
					)
# Find OpenNI2
#find_package(PkgConfig)
#pkg_check_modules(OpenNI2 REQUIRED libopenni2)

#############################
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
FIND_PACKAGE(Freetype REQUIRED)
################################

find_path(OpenNI2_INCLUDEDIR
	  NAMES OpenNI.h
	  HINTS /usr/include/openni2
	)
find_library(OpenNI2_LIBRARIES
	     NAMES OpenNI2 DummyDevice OniFile PS1090
	     HINTS /usr/lib/ /usr/lib/OpenNI2/Drivers
	     PATH_SUFFIXES lib
		) 
message(STATUS ${OpenNI2_LIBRARIES})

# Find Nite2
message(status $ENV{NITE2_INCLUDE})
message(status $ENV{NITE2_REDIST64})
find_path(Nite2_INCLUDEDIR
	  NAMES NiTE.h
	  HINTS $ENV{NITE2_INCLUDE})
find_library(Nite2_LIBRARY
	     NAMES NiTE2
	     HINTS $ENV{NITE2_REDIST64}
	     PATH_SUFFIXES lib) 


catkin_package( )

include_directories(${catkin_INCLUDEDIR}
		    ${OpenNI2_INCLUDEDIR}
		    ${Nite2_INCLUDEDIR}
          ##############################
                    ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} ${FREETYPE_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
                #################

                       )
add_executable(tracker_my src/tracker_my.cpp)
target_link_libraries(tracker_my ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES} ${Nite2_LIBRARY}
                                          #######
                                        ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLUT_glut_LIBRARY}
${FREETYPE_LIBRARIES}                    
                                             ########## 
                                                    )

install(TARGETS tracker_my RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES tracker_my.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

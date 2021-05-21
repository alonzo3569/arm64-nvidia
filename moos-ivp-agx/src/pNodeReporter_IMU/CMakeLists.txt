#--------------------------------------------------------
# The CMakeLists.txt for:                   pNodeReporter
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

# Set System Specific Libraries
SET(SYSTEM_LIBS m pthread)
# Set System Specific Libraries
#if (${WIN32})
#  SET(SYSTEM_LIBS
#    wsock32)
#else (${WIN32})
#  SET(SYSTEM_LIBS
#    m
#    pthread)
#endif (${WIN32})


SET(SRC
   NodeReporter.cpp
   NodeReporter_Info.cpp
   main.cpp
   MOOSGeodesy.cpp
)

ADD_EXECUTABLE(pNodeReporter_IMU ${SRC})
   
TARGET_LINK_LIBRARIES(pNodeReporter_IMU
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   contacts
   geometry
   apputil
   mbutil
   ${SYSTEM_LIBS})


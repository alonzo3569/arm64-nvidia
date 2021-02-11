#--------------------------------------------------------
# The CMakeLists.txt for:                       pThrustControl
# Author(s):                              logan
#--------------------------------------------------------

SET(SRC
  ThrustControl.cpp
  ThrustControl_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pThrustControl ${SRC})

TARGET_LINK_LIBRARIES(pThrustControl
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)


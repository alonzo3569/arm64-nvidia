#===========================================================================
# FILE:  CMakeLists.txt for pImuHeading
# DATE:  2021/05/22
# NAME:  logan
#===========================================================================

SET(SRC
  ImuHeading.cpp
  ImuHeading_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pImuHeading ${SRC})

TARGET_LINK_LIBRARIES(pImuHeading
  ${MOOS_LIBRARIES}
  geometry
  mbutil
  apputil
  m
  pthread
)


#===========================================================================
# FILE:  CMakeLists.txt for pPinger
# DATE:  2021/06/10
# NAME:  logan
#===========================================================================

SET(SRC
  Pinger.cpp
  Pinger_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pPinger ${SRC})

TARGET_LINK_LIBRARIES(pPinger
  ${MOOS_LIBRARIES}
  geometry
  mbutil
  apputil
  m
  pthread
)


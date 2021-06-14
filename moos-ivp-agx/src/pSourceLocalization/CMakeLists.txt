#===========================================================================
# FILE:  CMakeLists.txt for pSourceLocalization
# DATE:  2021/06/11
# NAME:  logan
#===========================================================================

SET(SRC
  SourceLocalization.cpp
  SourceLocalization_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pSourceLocalization ${SRC})

TARGET_LINK_LIBRARIES(pSourceLocalization
  ${MOOS_LIBRARIES}
  geometry
  mbutil
  apputil
  m
  pthread
)


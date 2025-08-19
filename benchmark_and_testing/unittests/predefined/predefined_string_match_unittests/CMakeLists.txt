####################################
# Automatic component project naming
get_filename_component(FNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${FNAME})


##################
# Unittest setings
set( LIB_TO_TEST
  dte3611::lib3611 )

set( UNITTESTS
  predefined_string_match_unittests )

set( OTHER_LINK_TARGETS
  dte3611::predefined_utils )


#######################
# Unittest build driver
option(DTE3611_BUILD_UNITTEST_${FNAME} "Build unittests: ${FNAME}" OFF)
if(DTE3611_BUILD_UNITTEST_${FNAME})
  ADD_UNITTESTS( ${LIB_TO_TEST} UNITTESTS ${OTHER_LINK_TARGETS} )
endif(DTE3611_BUILD_UNITTEST_${FNAME})

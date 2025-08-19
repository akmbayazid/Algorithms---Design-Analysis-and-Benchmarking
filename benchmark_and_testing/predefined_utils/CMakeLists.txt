project(predefined_utils)

# Add library target
add_library(${PROJECT_NAME} INTERFACE )

# Headers
target_include_directories( ${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include> )

# Make ${PROJECT_NAME} available as a direct linkable target
add_library(dte3611::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

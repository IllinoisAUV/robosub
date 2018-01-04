#additional target to perform cppcheck run, requires cppcheck
find_program(PYLINT pylint)
# get all python project files
file(GLOB_RECURSE ALL_SOURCE_FILES *.py)
MESSAGE( STATUS "${ALL_SOURCE_FILES}")
add_custom_target(
  pylint
  COMMAND ${PYLINT}
  ${ALL_SOURCE_FILES}
)

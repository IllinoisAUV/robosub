#additional target to perform cppcheck run, requires cppcheck
find_program(PYLINT pylint)
# get all python project files
file(GLOB_RECURSE ALL_SOURCE_FILES *.py)
MESSAGE( STATUS "${ALL_SOURCE_FILES}")
add_custom_target(
  pylint
  COMMAND ${PYLINT}
  --init-hook
  "import sys; sys.path.extend(\"${CMAKE_CURRENT_SOURCE_DIR}\".split(";"))"
  ${ALL_SOURCE_FILES}
  VERBATIM
)

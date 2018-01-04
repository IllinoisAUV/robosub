#additional target to perform cppcheck run, requires cppcheck
find_program(CLANGFORMAT clang-format)
# get all c++ project files
file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp *.h)

add_custom_target(
  clangformat
  COMMAND ${CLANGFORMAT}
  -style=Google
  -i
  ${ALL_SOURCE_FILES}
)

# For use in CI
add_custom_target(
  clangformat-check
  COMMAND ${CMAKE_CURRENT_LIST_DIR}/../tools/ci/clang-format-check.sh
  -style=Google
  ${ALL_SOURCE_FILES}
)

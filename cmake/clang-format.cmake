#additional target to perform cppcheck run, requires cppcheck
find_program(CLANGFORMAT clang-format)
if(CLANGFORMAT)
  # get all c++ project files
  file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp *.h)

  add_custom_target(
    clangformat
    COMMAND ${CLANGFORMAT}
    -style=Google
    -i
    ${ALL_SOURCE_FILES}
  )
endif()

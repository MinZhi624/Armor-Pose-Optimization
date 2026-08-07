# This project uses its checked-in .clang-format as the C++ formatter.
# ament_uncrustify's default style conflicts with the established codebase.
if(TEST uncrustify)
  set_tests_properties(uncrustify PROPERTIES DISABLED TRUE)
endif()

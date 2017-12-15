file(REMOVE_RECURSE
  "test_gpu.pdb"
  "test_gpu"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/test_gpu.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()

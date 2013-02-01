FILE(REMOVE_RECURSE
  "msg_gen"
  "src/ardrone_brown/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/ardrone_brown/msg/__init__.py"
  "src/ardrone_brown/msg/_Navdata.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)

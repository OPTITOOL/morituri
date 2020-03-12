

if(SHP_INCLUDE_DIR AND SHP_LIBRARIES)
  set(SHP_FOUND TRUE)
else(SHP_INCLUDE_DIR AND SHP_LIBRARIES)

  find_path(SHP_INCLUDE_DIR NAMES shapefil.h PATH_SUFFIXES libshp)
  find_library(SHP_LIBRARIES NAMES shp shapelib)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(libshp DEFAULT_MSG SHP_INCLUDE_DIR SHP_LIBRARIES)

  mark_as_advanced(SHP_INCLUDE_DIR SHP_LIBRARIES)
endif(SHP_INCLUDE_DIR AND SHP_LIBRARIES)
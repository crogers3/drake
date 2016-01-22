add_library_with_exports(LIB_NAME drakeShapes SOURCE_FILES Geometry.cpp Element.cpp VisualElement.cpp)
target_link_libraries(drakeShapes spruce -lpng -lpthread)
set_target_properties(drakeShapes PROPERTIES
  COMPILE_FLAGS "-Dcimg_display=0 -Dcimg_use_png"
)

#enable_testing()
#add_subdirectory(test)

pods_install_libraries(drakeShapes)
drake_install_headers(DrakeShapes.h Geometry.h Element.h VisualElement.h)
  

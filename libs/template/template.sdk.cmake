ADD_IMPORTED_LIBRARY(template ${CMAKE_CURRENT_LIST_DIR}/libtemplate.a)

#install all headers related to this library.
FOREACH(header )
	ARC_INSTALL_LIBARAY_HEADERS(template ${header})
ENDFOREACH()

# - Try to find libkdtree++ lib
# Once done this will define
#
#  KDTREE_FOUND - system has kdtree++ lib
#  KDTREE_INCLUDE_DIR - the kdtree++ include directory

# Copyright (c) 2006, 2007 Montel Laurent, <montel@kde.org>
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

if (KDTREE_INCLUDE_DIR)

  # in cache already
  set(KDTREE_FOUND TRUE)

else (KDTREE_INCLUDE_DIR)

find_path(KDTREE_INCLUDE_DIR NAMES kdtree++/kdtree.hpp
     PATH_SUFFIXES kdtree++
     HINTS
     ${INCLUDE_INSTALL_DIR}
     ${KDE4_INCLUDE_DIR}
   )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(KDTree DEFAULT_MSG KDTREE_INCLUDE_DIR )


mark_as_advanced(KDTREE_INCLUDE_DIR)

endif(KDTREE_INCLUDE_DIR)


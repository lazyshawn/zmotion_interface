###############################################################################
# Install package
###############################################################################
# Include gnuinstalldir to get the platform's standard directories:
include(GNUInstallDirs)

# Now, we install the export set. This will generate a CMake file exporting all the target for other projects to use:
install(EXPORT ${PROJECT_NAME}Targets
  DESTINATION "${CMAKE_INSTALL_DATADIR}/cmake/${PROJECT_NAME}"
  NAMESPACE ${PROJECT_NAME}::
)

# Now, we also export the current buildtree. Other project will be able to import the project directly from a build dir:
configure_file(${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake ${PROJECT_NAME}Config.cmake COPYONLY)
export(EXPORT ${PROJECT_NAME}Targets
  NAMESPACE ${PROJECT_NAME}::
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
)

# The file we created earlier:
install(FILES ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake
  DESTINATION "${CMAKE_INSTALL_DATADIR}/cmake/${PROJECT_NAME}"
)

###############################################################################
# Pack project
###############################################################################
include(InstallRequiredSystemLibraries)

set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${CMAKE_SYSTEM_NAME}-${CMAKE_BUILD_TYPE}-${PROJECT_VERSION}")
set(CPACK_PACKAGE_VENDOR "Lazyshawn")
set(CPACK_RESOURCE_FILE_README "${PROJECT_SOURCE_DIR}/README.md")
# set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
set(CPACK_SOURCE_GENERATOR "ZIP")

include(CPack)

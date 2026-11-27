# VoxelCore.app packed to VoxelCore.dmg with cpack
set(VC_BUNDLE_CONTENTS VoxelCore.app/Contents)

file(READ ${CMAKE_CURRENT_SOURCE_DIR}/src/constants.hpp VC_CONSTANTS)
string(REGEX MATCH "ENGINE_VERSION_MAJOR = ([0-9]+)" _ "${VC_CONSTANTS}")
set(VC_VERSION_MAJOR ${CMAKE_MATCH_1})
string(REGEX MATCH "ENGINE_VERSION_MINOR = ([0-9]+)" _ "${VC_CONSTANTS}")
set(VC_VERSION_MINOR ${CMAKE_MATCH_1})

if(VC_VERSION_PATCH STREQUAL "")
    set(VC_BUNDLE_VERSION ${VC_VERSION_MAJOR}.${VC_VERSION_MINOR}.0)
else()
    set(VC_BUNDLE_VERSION
        ${VC_VERSION_MAJOR}.${VC_VERSION_MINOR}.${VC_VERSION_PATCH})
endif()

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/dev/macos/Info.plist.in
    ${CMAKE_CURRENT_BINARY_DIR}/Info.plist @ONLY
)

install(
    PROGRAMS $<TARGET_FILE:VoxelCore>
    DESTINATION ${VC_BUNDLE_CONTENTS}/MacOS
    RENAME VoxelCore-bin
)
install(
    PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/dev/macos/launcher.sh
    DESTINATION ${VC_BUNDLE_CONTENTS}/MacOS
    RENAME VoxelCore
)
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/Info.plist
    DESTINATION ${VC_BUNDLE_CONTENTS}
)
install(
    FILES ${CMAKE_CURRENT_SOURCE_DIR}/dev/macos/VoxelCore.icns
    DESTINATION ${VC_BUNDLE_CONTENTS}/Resources
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/res
    DESTINATION ${VC_BUNDLE_CONTENTS}/Resources
)


execute_process(
    COMMAND brew --prefix
    OUTPUT_VARIABLE VC_BREW_PREFIX
    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET
)

install(
    CODE "set(VC_LIB_DIRS \"${VC_BREW_PREFIX}/lib\")"
)
install(
    CODE [[
    include(BundleUtilities)
    set(app "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/VoxelCore.app")
    fixup_bundle("${app}" "${app}/Contents/MacOS/VoxelCore-bin"
                 "${VC_LIB_DIRS}")
    execute_process(COMMAND codesign --force --deep --sign - "${app}"
                    COMMAND_ERROR_IS_FATAL ANY)
]])

set(CPACK_GENERATOR DragNDrop)
set(CPACK_PACKAGE_NAME VoxelCore)
set(CPACK_PACKAGE_VERSION ${VC_BUNDLE_VERSION})
set(CPACK_PACKAGE_FILE_NAME VoxelCore)
set(CPACK_DMG_VOLUME_NAME VoxelCore)
set(CPACK_DMG_BACKGROUND_IMAGE ${CMAKE_CURRENT_SOURCE_DIR}/dev/macos/dmg-background.tiff)
set(CPACK_DMG_DS_STORE_SETUP_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/dev/macos/dmg-setup.applescript)
include(CPack)

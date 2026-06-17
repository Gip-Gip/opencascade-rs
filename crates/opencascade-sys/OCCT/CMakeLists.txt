cmake_minimum_required (VERSION 3.10 FATAL_ERROR)
project (OpenCASCADEPackageConfig)

# Fix for WASM: https://github.com/emscripten-core/emscripten/issues/19243
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE "BOTH" CACHE PATH "")

if (NOT OCCT_CMAKE_FALLBACK)
    find_package(OpenCASCADE CONFIG QUIET)
    get_property(flags DIRECTORY PROPERTY COMPILE_DEFINITIONS)
    # OCCT 7.5 adds this define that causes hundreds of compiler warnings with Qt5.x, so remove it again
    list(FILTER flags EXCLUDE REGEX [[GL_GLEXT_LEGACY]])
    set_property(DIRECTORY PROPERTY COMPILE_DEFINITIONS ${flags})
endif ()
if (OpenCASCADE_FOUND)
    set(OCC_FOUND ${OpenCASCADE_FOUND})
    set(OCC_INCLUDE_DIR ${OpenCASCADE_INCLUDE_DIR})
    set(OCC_LIBRARY_DIR ${OpenCASCADE_LIBRARY_DIR})
    set(OCC_LIBRARIES ${OpenCASCADE_LIBRARIES})
    set(OCC_OCAF_LIBRARIES TKCAF TKXCAF)
else ()
    if (WIN32)
        if (CYGWIN OR MINGW)
            find_path(OCC_INCLUDE_DIR Standard_Version.hxx
                    /usr/include/opencascade
                    /usr/local/include/opencascade
                    /opt/opencascade/include
                    /opt/opencascade/inc
            )
            find_path(OCC_LIBRARY TKernel
                    /usr/lib
                    /usr/local/lib
                    /opt/opencascade/lib
            )
        else ()
            find_path(OCC_INCLUDE_DIR Standard_Version.hxx
                    "[HKEY_LOCAL_MACHINE\\SOFTWARE\\SIM\\OCC\\2;Installation Path]/include"
            )
            find_library(OCC_LIBRARY TKernel
                    "[HKEY_LOCAL_MACHINE\\SOFTWARE\\SIM\\OCC\\2;Installation Path]/lib"
            )
        endif ()
    else ()
        find_path(OCC_INCLUDE_DIR Standard_Version.hxx
                /usr/include/occt
                /usr/include/opencascade
                /usr/local/include/opencascade
                /opt/opencascade/include
                /opt/opencascade/inc
        )
        find_library(OCC_LIBRARY TKernel
                /usr/lib
                /usr/local/lib
                /opt/opencascade/lib
        )
    endif ()
    if (OCC_LIBRARY)
        get_filename_component(OCC_LIBRARY_DIR ${OCC_LIBRARY} PATH)
        if (NOT OCC_INCLUDE_DIR)
            find_path(OCC_INCLUDE_DIR Standard_Version.hxx
                    ${OCC_LIBRARY_DIR}/../inc
            )
        endif ()
    endif ()
endif ()

if (OCC_INCLUDE_DIR)
    file(STRINGS ${OCC_INCLUDE_DIR}/Standard_Version.hxx OCC_MAJOR
            REGEX "#define OCC_VERSION_MAJOR.*"
    )
    string(REGEX MATCH "[0-9]+" OCC_MAJOR ${OCC_MAJOR})
    file(STRINGS ${OCC_INCLUDE_DIR}/Standard_Version.hxx OCC_MINOR
            REGEX "#define OCC_VERSION_MINOR.*"
    )
    string(REGEX MATCH "[0-9]+" OCC_MINOR ${OCC_MINOR})
    file(STRINGS ${OCC_INCLUDE_DIR}/Standard_Version.hxx OCC_MAINT
            REGEX "#define OCC_VERSION_MAINTENANCE.*"
    )
    string(REGEX MATCH "[0-9]+" OCC_MAINT ${OCC_MAINT})

    set(OCC_VERSION_STRING "${OCC_MAJOR}.${OCC_MINOR}.${OCC_MAINT}")
endif ()

file (WRITE ${CMAKE_BINARY_DIR}/occ_info.txt
    "VERSION_MAJOR=${OCC_MAJOR}\n"
    "VERSION_MINOR=${OCC_MINOR}\n"
    "INCLUDE_DIR=${OCC_INCLUDE_DIR}\n"
    "LIBRARY_DIR=${OCC_LIBRARY_DIR}\n"
    "BUILD_SHARED_LIBS=${OpenCASCADE_BUILD_SHARED_LIBS}\n")

install (FILES ${CMAKE_BINARY_DIR}/occ_info.txt TYPE DATA)

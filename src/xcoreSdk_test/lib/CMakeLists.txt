cmake_minimum_required(VERSION 3.12)

# 导入库
add_library(xCoreSDK_lib SHARED IMPORTED GLOBAL)
add_library(xCoreSDK_static STATIC IMPORTED GLOBAL)
add_library(xMateModel_lib STATIC IMPORTED GLOBAL)

if(WIN32)
    if(${CMAKE_SIZEOF_VOID_P} EQUAL 8) #64bit
        set(_ARCH_NAME "64bit")
    elseif(${CMAKE_SIZEOF_VOID_P} EQUAL 4) #32bit
        set(_ARCH_NAME "32bit")
    endif()

    set_target_properties(xMateModel_lib PROPERTIES
        IMPORTED_LOCATION_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Release/${_ARCH_NAME}/xMateModel.lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Debug/${_ARCH_NAME}/xMateModeld.lib)

    set_target_properties(xCoreSDK_lib PROPERTIES
        IMPORTED_IMPLIB_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Release/${_ARCH_NAME}/xCoreSDK.lib
        IMPORTED_LOCATION_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Release/${_ARCH_NAME}/xCoreSDK.dll
        IMPORTED_IMPLIB_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Debug/${_ARCH_NAME}/xCoreSDK.lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Debug/${_ARCH_NAME}/xCoreSDK.dll
    )

    set_target_properties(xCoreSDK_static PROPERTIES
        IMPORTED_LOCATION_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Release/${_ARCH_NAME}/xCoreSDK_static.lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Debug/${_ARCH_NAME}/xCoreSDK_static.lib
        INTERFACE_LINK_LIBRARIES xMateModel_lib
    )

elseif(UNIX)
    if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
        set_target_properties(xCoreSDK_lib PROPERTIES
            INTERFACE_LINK_LIBRARIES xMateModel_lib)
    elseif((NOT ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64") OR ${XCORE_USE_XMATE_MODEL})
        message(FATAL_ERROR "Unsupported: target ${CMAKE_SYSTEM_PROCESSOR} OR XCORE_USE_XMATE_MODEL option")
    endif()

    # 设置路径
    set_target_properties(xMateModel_lib PROPERTIES
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/Linux/x86_64/libxMateModel.a)

    set(XCORESDK_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Linux/${CMAKE_SYSTEM_PROCESSOR})
    set_target_properties(xCoreSDK_lib PROPERTIES
        IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/libxCoreSDK.so.0.6.0
    )
    set_target_properties(xCoreSDK_static PROPERTIES
        IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/libxCoreSDK.a
        INTERFACE_LINK_LIBRARIES xMateModel_lib
    )
endif()

# 提供别名给顶层使用
if(XCORE_LINK_SHARED_LIBS)
    add_library(Rokae::Rokae ALIAS xCoreSDK_lib)
else()
    add_library(Rokae::Rokae ALIAS xCoreSDK_static)
endif()

# 安装规则保留
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.21")
    install(IMPORTED_RUNTIME_ARTIFACTS xCoreSDK_lib
        RUNTIME
        DESTINATION ${CMAKE_INSTALL_BINDIR})
else()
    string(TOUPPER ${CMAKE_BUILD_TYPE} build_type_upper)
    get_target_property(runtime_dll_location xCoreSDK_lib IMPORTED_LOCATION_${build_type_upper})
    if(${runtime_dll_location} MATCHES "NOTFOUND$")
        get_target_property(runtime_dll_location xCoreSDK_lib IMPORTED_LOCATION)
    endif()
    if(NOT ${runtime_dll_location} MATCHES "NOTFOUND$")
        install(FILES ${runtime_dll_location} DESTINATION ${CMAKE_INSTALL_BINDIR})
    endif()
endif()

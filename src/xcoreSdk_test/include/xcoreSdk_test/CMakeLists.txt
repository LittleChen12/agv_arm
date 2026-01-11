# Public headers
# 定义一个变量 SHARED_INCLUDEDIR，值为当前目录路径（${CMAKE_CURRENT_SOURCE_DIR}）。
# PARENT_SCOPE 表示这个变量会传递到 上一级 CMake 作用域，而不仅仅在当前文件中有效。
set(SHARED_INCLUDEDIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

# Here puts required headers
# 定义一个变量 SHARED_HEADERS，里面存放了一组头文件的完整路径。${CMAKE_CURRENT_SOURCE_DIR} 表示当前目录，后面拼接 rokae/xxx.h。
set(SHARED_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/rokae/base.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rokae/data_types.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rokae/exception.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rokae/force_control.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rokae/model.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rokae/motion_control_rt.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rokae/planner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rokae/robot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rokae/utility.h
    )


# 将 SHARED_HEADERS 变量设置为缓存变量，类型为 INTERNAL，描述信息为 "Shared headers with examples"。
set(SHARED_HEADERS ${SHARED_HEADERS} CACHE INTERNAL "Shared headers with examples")
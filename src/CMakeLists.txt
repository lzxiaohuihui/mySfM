find_package(OpenCV 3 REQUIRED)
FIND_PACKAGE(GLEW REQUIRED)
FIND_PACKAGE(glfw3 REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(CUDA REQUIRED)
find_package(Boost      REQUIRED COMPONENTS system chrono filesystem unit_test_framework program_options)
find_package(Ceres REQUIRED)


include_directories(${Boost_INCLUDE_DIR})
link_directories   (${Boost_LIBRARY_DIR})

# ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
# INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
# LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

# FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters )
# list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

include_directories(${OpenGL_INCLUDE_DIR})
link_directories(/usr/lib64)


add_executable(
    main 
    main.cpp
    sfm.cpp
    common.cpp
    featureTool.cpp
    image.cpp
    myStereo.cpp
    sfmBA.cpp
    )
target_link_libraries(main
    libsiftgpu.so
    ${Boost_LIBRARIES}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS} 
    ${GLEW_LIBRARIES} 
    ${GLUT_LIBRARIES} 
    ${OPENGL_LIBRARIES}
    # ${PCL_LIBRARIES}
)

# add_executable(test test.cpp)
# target_link_libraries(test
#     libsiftgpu.so
#     ${OpenCV_LIBS} 
#     ${GLEW_LIBRARIES} 
#     ${GLUT_LIBRARIES} 
#     ${OPENGL_LIBRARIES}
# )
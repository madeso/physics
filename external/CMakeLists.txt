set(OpenGL_GL_PREFERENCE "GLVND")
add_subdirectory(raylib)

###################################################################################################
# glm
add_library(external_glm INTERFACE)
target_include_directories(external_glm SYSTEM
    INTERFACE
        glm
)
add_library(external::glm ALIAS external_glm)
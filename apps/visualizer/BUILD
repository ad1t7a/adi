cc_binary(
    name = "visualizer",
    srcs = ["main.cpp",],

    deps = [
        "//common:common", 
        "//io/camera:camera", 
        "//visualization:visualization",
    ],
    linkopts = ["-framework OpenGL", "-framework GLUT"],
)
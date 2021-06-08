cc_binary(
    name = "visualizer",
    srcs = ["main.cpp",],

    deps = [
        "//common:common", 
        "//visualization:visualization",
        "//multibody:multibody",
        "//io/camera:camera",
    ],
    linkopts = ["-framework OpenGL", "-framework GLUT"],
)
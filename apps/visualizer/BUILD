cc_binary(
    name = "visualizer",
    srcs = ["main.cpp",],

    deps = [
        "//common:common", 
        "//visualization:visualization",
        "//multibody:multibody",
    ],
    linkopts = ["-framework OpenGL", "-framework GLUT"],
)
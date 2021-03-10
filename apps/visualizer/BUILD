cc_binary(
    name = "visualizer",
    srcs = ["main.cpp",],

    deps = [
        "//common:common", 
        "//io/camera:camera", 
        "//visualization:visualization",
        "//systems/fastplanner:fastplanner",
    ],
    linkopts = ["-framework OpenGL", "-framework GLUT"],
)
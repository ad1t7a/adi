cc_binary(
    name = "visualizer",
    srcs = ["main.cpp",],

    deps = [
        "//common:common", 
        "//io/camera:camera", 
        "//visualization:visualization",
        "//systems/fastplanner:fastplanner",
        "//systems/taskplanner:taskplanner",
    ],
    linkopts = ["-framework OpenGL", "-framework GLUT"],
)
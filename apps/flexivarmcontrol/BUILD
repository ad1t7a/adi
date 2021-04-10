cc_binary(
    name = "flexivarmcontrol",
    srcs = ["main.cpp", "robotdb.hpp"],

    deps = [
        "//common:common", 
        "//visualization:visualization",
        "//controllers:controllers",
    ],
    linkopts = ["-framework OpenGL", "-framework GLUT"],
)
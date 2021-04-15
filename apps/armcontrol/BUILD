cc_binary(
    name = "armcontrol",
    srcs = ["main.cpp", "robotdb.hpp", "ConnectionManager.hpp"],

    deps = [
        "//common:common", 
        "//visualization:visualization",
        "//controllers:controllers",
    ],
    linkopts = ["-framework OpenGL", "-framework GLUT"],
)
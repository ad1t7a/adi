cc_binary(
    name = "armcontrol",
    srcs = ["main.cpp", "robotdb.hpp",],

    deps = [
        "//common:common", 
        "//visualization:visualization",
        "//controllers/cartesian:cartesian",
        "//io/joystick:joystick",
    ],
    linkopts = ["-framework OpenGL", "-framework GLUT"],
)
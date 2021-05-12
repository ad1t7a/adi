cc_binary(
    name = "armcontrol",
    srcs = ["main.cpp", ],

    deps = [
        "//common:common", 
        "//visualization:visualization",
        "//controllers/cartesian:cartesian",
        "//io/joystick:joystick",
        "//multibody:multibody",
        "//io/arm:arm",
    ],
    linkopts = ["-framework OpenGL", "-framework GLUT"],
)
cc_binary(
    name = "obstaclefreeregion",
    srcs = ["main.cpp",],

    deps = [
        "//common:common", 
        "//mathematics:mathematics", 
        "//systems/regioninflation:regioninflation",
        "//visualization:visualization",
    ],
    linkopts = ["-framework OpenGL", "-framework GLUT"],
)
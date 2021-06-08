cc_library(
    name = "arm",
    hdrs = ["config.hpp", "robotinterface.hpp", "urinterface.hpp", "flexivinterface.hpp", ],
    srcs = ["urinterface.cpp", "flexivinterface.cpp"],
    deps = ["@eigen//:eigen", 
            "//common:common", 
            "@urdriver//:urdriver",
            "@fvrethernetclient//:fvrethernetclient"],
    visibility = ["//visibility:public"],
)
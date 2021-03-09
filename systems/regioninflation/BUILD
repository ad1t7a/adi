cc_library(
    name = "regioninflation",
    srcs = ["IRISProblem.cpp", "IRIS.cpp", "mosekinterface.cpp", ],
    hdrs = ["IRISProblem.hpp", "IRISOptions.hpp", "IRISRegion.hpp", "IRIS.hpp", "mosekinterface.hpp", ],
    deps = ["//common:common", "//mathematics:mathematics", "@mosek//:mosek",],
    visibility = ["//visibility:public"],
)
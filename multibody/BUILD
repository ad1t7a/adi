cc_library(
    name = "multibody",
    srcs = ["multibodyplant.cpp", "rbdlmultibody.cpp", "inversekinematics/inversekinematics.cpp",],
    hdrs = ["multibodyplant.hpp", "rbdlmultibody.hpp", "inversekinematics/inversekinematics.hpp",],
    deps = ["//common:common", "@fmt//:fmt", "@spdlog//:spdlog", "@rbdl//:rbdl", "@gurobi//:gurobi",],
    visibility = ["//visibility:public"],
)
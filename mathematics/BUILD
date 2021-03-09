cc_library(
    name = "mathematics",
    srcs = ["polyhedron.cpp", "ellipsoid.cpp", ],
    hdrs = ["polyhedron.hpp", "ellipsoid.hpp", "interfaces/cddinterface.hpp", ],
    deps = ["//common:common", "@libcdd//:libcdd", "@fmt//:fmt", "@spdlog//:spdlog",],
    visibility = ["//visibility:public"],
)
cc_library(
    name = "common",
    srcs = ["adi_assert_and_throw.cpp", "eigen_types.cpp", "adi_assert.cpp", "frame.cpp",],
    hdrs = ["adi_assert.hpp", "adi_assertion_error.hpp", "adi_throw.hpp", 
            "cond.hpp", "constants.hpp", "converters.hpp", "copyable.hpp", 
            "doubleoverloads.hpp", "dummyvalue.hpp", "eigen_types.hpp", 
            "is_approx_equal.hpp", "never_destroyed.hpp", "saturate.hpp",
            "wrap_to.hpp", "factorial.hpp", "frame.hpp", "arg_sort.hpp", ],
    deps = ["@eigen//:eigen", "@fmt//:fmt", "@spdlog//:spdlog",],
    visibility = ["//visibility:public"],
)
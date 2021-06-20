cc_library(
  name = "octomap",
  hdrs = glob(["octomap/include/**", "octovis/include/**",]),
  includes = ["octomap/include/", "octovis/include/", ],
  srcs = [
    "lib/liboctomap.a",
    "lib/liboctomath.a",
    "lib/libdynamicedt3d.a",
  ],
  visibility = ["//visibility:public"],
)

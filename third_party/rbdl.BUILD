cc_library(
  name = "rbdl",
  hdrs = glob(["build/include/**", "build/addons/**", "addons/urdfreader/thirdparty/**"]),
  includes = ["build/include/", "build/addons/", "addons/urdfreader/thirdparty/"
  ],
  srcs = [
    "build/librbdl.dylib", 
    "build/librbdl.2.6.0.dylib", 
    "build/addons/urdfreader/librbdl_urdfreader.2.6.0.dylib", 
    "build/addons/urdfreader/librbdl_urdfreader.dylib", 
  ],
  visibility = ["//visibility:public"],
)
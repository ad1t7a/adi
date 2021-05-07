cc_library(
  name = "bullet",
  hdrs = glob(["src/**",]),
  includes = ["src/",
  ],
  srcs = [
    "build_cmake/src/Bullet3Collision/libBullet3Collision.3.08.dylib",
    "build_cmake/src/Bullet3Common/libBullet3Common.dylib",
    "build_cmake/src/Bullet3Dynamics/libBullet3Dynamics.dylib",
    "build_cmake/src/LinearMath/libLinearMath.dylib",
  ],
  visibility = ["//visibility:public"],
)
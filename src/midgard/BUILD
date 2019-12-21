package(default_visibility=["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

load("//:conf/flags.bzl", "LLVM_LAZY_FLAGS")

cc_library(
    name="midgard",
    srcs=[
        "linesegment2.cc",
        "tiles.cc",
        "gridded_data.cc",
        "polyline2.cc",
        "obb2.cc",
        "pointll.cc",
        "aabb2.cc",
        "point2.cc",
        "util.cc",
        "ellipse.cc",
        "logging.cc",
    ],
    hdrs=[
        "//valhalla:headers",
    ],
    copts=LLVM_LAZY_FLAGS,
)

package(default_visibility=["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

load("//:conf/flags.bzl", "LLVM_LAZY_FLAGS")

cc_library(
    name="skadi",
    srcs=[
        "sample.cc",
        "util.cc",
    ],
    hdrs=[
        "//valhalla:headers",
    ],
    copts=LLVM_LAZY_FLAGS,
    deps=[
        "@boost//:optional",
    ],
)

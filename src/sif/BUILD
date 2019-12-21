package(default_visibility=["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

load("//:conf/flags.bzl", "LLVM_LAZY_FLAGS")

cc_library(
    name="sif",
    srcs=[
        "autocost.cc",
        "bicyclecost.cc",
        "hierarchylimits.cc",
        "motorcyclecost.cc",
        "motorscootercost.cc",
        "pedestriancost.cc",
        "transitcost.cc",
        "truckcost.cc",
        "dynamiccost.cc",
    ],
    hdrs=[
        "//valhalla:headers",
    ],
    copts=LLVM_LAZY_FLAGS,
    deps=[
        "@boost//:lexical_cast",
        "@boost//:property_tree",
        "@boost//:variant",
        "@date",
        "@rapidjson",
        "//valhalla/proto:cc_proto_options",
        "//valhalla/proto:cc_proto_api",
    ],
)

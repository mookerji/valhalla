package(default_visibility=["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

load("//:conf/flags.bzl", "LLVM_LAZY_FLAGS")

cc_library(
    name="loki",
    srcs=[
        "search.cc",
        "worker.cc",
        "height_action.cc",
        "locate_action.cc",
        "reach.cc",
        "route_action.cc",
        "matrix_action.cc",
        "isochrone_action.cc",
        "trace_route_action.cc",
        "transit_available_action.cc",
        "node_search.cc",
    ],
    hdrs=[
        "//valhalla:headers",
    ],
    copts=LLVM_LAZY_FLAGS,
    deps=[
        "@boost//:lexical_cast",
        "@boost//:property_tree",
        "@boost//:variant",
        "@rapidjson",
        "@date",
        "//valhalla/proto:cc_proto_tripcommon",
        "//valhalla/proto:cc_proto_options",
        "//valhalla/proto:cc_proto_api",
    ],
)

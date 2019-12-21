package(default_visibility=["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

load("//:conf/flags.bzl", "LLVM_LAZY_FLAGS")

cc_library(
    name="thor",
    srcs=[
        "astar.cc",
        "bidirectional_astar.cc",
        "costmatrix.cc",
        "isochrone.cc",
        "map_matcher.cc",
        "multimodal.cc",
        "optimizer.cc",
        "triplegbuilder.cc",
        "attributes_controller.cc",
        "route_matcher.cc",
        "timedep_forward.cc",
        "timedep_reverse.cc",
        "timedistancematrix.cc",
        "worker.cc",
        "isochrone_action.cc",
        "matrix_action.cc",
        "optimized_route_action.cc",
        "route_action.cc",
        "trace_attributes_action.cc",
        "trace_route_action.cc",
    ],
    hdrs=[
        "//valhalla:headers",
    ],
    copts=LLVM_LAZY_FLAGS,
    deps=[
        "@boost//:heap",
        "@boost//:lexical_cast",
        "@boost//:optional",
        "@boost//:property_tree",
        "@boost//:variant",
        "@date",
        "@rapidjson",
        "//valhalla/proto:cc_proto_api",
    ],
)

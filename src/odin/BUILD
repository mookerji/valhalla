package(default_visibility=["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

load("//:conf/flags.bzl", "LLVM_LAZY_FLAGS")

cc_library(
    name="odin",
    srcs=[
        "directionsbuilder.cc",
        "maneuversbuilder.cc",
        "narrative_dictionary.cc",
        "narrative_builder_factory.cc",
        "narrativebuilder.cc",
        "enhancedtrippath.cc",
        "maneuver.cc",
        "sign.cc",
        "signs.cc",
        "util.cc",
        "transitrouteinfo.cc",
        "worker.cc",
    ],
    hdrs=[
        "//valhalla:headers",
    ],
    copts=LLVM_LAZY_FLAGS,
    deps=[
        "@boost//:algorithm",
        "@boost//:lexical_cast",
        "@boost//:property_tree",
        "@boost//:variant",
        "@date",
        "@rapidjson",
        "//valhalla/proto:cc_proto_directions",
        "//valhalla/proto:cc_proto_options",
        "//valhalla/proto:cc_proto_trip",
        "//valhalla/proto:cc_proto_api",
    ],
)

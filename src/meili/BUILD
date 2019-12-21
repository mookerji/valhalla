package(default_visibility=["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

load("//:conf/flags.bzl", "LLVM_LAZY_FLAGS")

cc_library(
    name="meili",
    srcs=[
        "viterbi_search.cc",
        "topk_search.cc",
        "routing.cc",
        "candidate_search.cc",
        "transition_cost_model.cc",
        "map_matcher.cc",
        "map_matcher_factory.cc",
        "match_route.cc",
    ],
    hdrs=[
        "//valhalla:headers",
    ],
    copts=LLVM_LAZY_FLAGS,
    deps=[
        "@boost//:lexical_cast",
        "@boost//:heap",
        "@boost//:intrusive",
        "@boost//:property_tree",
        "@boost//:variant",
        "@rapidjson",
        "@date",
        "//valhalla/proto:cc_proto_tripcommon",
        "//valhalla/proto:cc_proto_options",
        "//valhalla/proto:cc_proto_api",
    ],
)

package(default_visibility=["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

load("//:conf/flags.bzl", "LLVM_LAZY_FLAGS")

cc_library(
    name="tyr",
    srcs=[
        "serializers.cc",
        "isochrone_serializer.cc",
        "matrix_serializer.cc",
        "height_serializer.cc",
        "locate_serializer.cc",
        "route_serializer.cc",
        #"route_serializer_valhalla.cc",
        #"route_serializer_osrm.cc",
        "transit_available_serializer.cc",
        "trace_serializer.cc",
        "actor.cc",
    ],
    hdrs=[
        "//valhalla:headers",
    ],
    copts=LLVM_LAZY_FLAGS + ['-I./'],
    deps=[
        "@boost//:heap",
        "@boost//:lexical_cast",
        "@boost//:property_tree",
        "@boost//:variant",
        "@rapidjson",
        "@date",
        "//valhalla/proto:cc_proto_directions",
        "//valhalla/proto:cc_proto_options",
        "//valhalla/proto:cc_proto_trip",
        "//valhalla/proto:cc_proto_api",
        "//src/loki:loki",
        "//src/thor:thor",
        "//src/odin:odin",
    ],
)

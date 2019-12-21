package(default_visibility=["//visibility:public"])

load("//:conf/flags.bzl", "LLVM_LAZY_FLAGS")

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

filegroup(
    name="src",
    srcs=[
        "//src/baldr:src",
        # "//src/loki:src",
        # "//src/meili:src",
        # "//src/midgard:src",
        # "//src/mjolnir:src",
        # "//src/odin:src",
        # "//src/sif:src",
        # "//src/skadi:src",
        # "//src/thor:src",
        # "//src/tyr:src",
    ],
)

cc_binary(
    name="city_test",
    srcs=["city_test.cc"],
    copts=LLVM_LAZY_FLAGS,
    deps=[
        "@boost//:program_options",
        "//src/baldr:baldr",
        "//src/loki:loki",
        "//src/meili:meili",
        "//src/midgard:midgard",
        "//src/mjolnir:mjolnir",
        "//src/odin:odin",
        "//src/sif:sif",
        "//src/skadi:skadi",
        "//src/thor:thor",
        "//src/tyr:tyr",
    ],
)

cc_binary(
    name="unconnected_ways",
    srcs=["unconnected_ways.cc"],
)

cc_binary(
    name="valhalla_associate_segments",
    srcs=["valhalla_associate_segments.cc"],
)

cc_binary(
    name="valhalla_benchmark_adjacency_list",
    srcs=["valhalla_benchmark_adjacency_list.cc"],
)

cc_binary(
    name="valhalla_benchmark_loki",
    srcs=["valhalla_benchmark_loki.cc"],
)

cc_binary(
    name="valhalla_benchmark_skadi",
    srcs=["valhalla_benchmark_skadi.cc"],
)

cc_binary(
    name="valhalla_expand_bounding_box",
    srcs=["valhalla_expand_bounding_box.cc"],
)

cc_binary(
    name="valhalla_export_edges",
    srcs=["valhalla_export_edges.cc"],
)

cc_binary(
    name="valhalla_loki_worker",
    srcs=["valhalla_loki_worker.cc"],
)

cc_binary(
    name="valhalla_odin_worker",
    srcs=["valhalla_odin_worker.cc"],
)

cc_binary(
    name="valhalla_path_comparison",
    srcs=["valhalla_path_comparison.cc"],
)

cc_binary(
    name="valhalla_run_isochrone",
    srcs=["valhalla_run_isochrone.cc"],
)

cc_binary(
    name="valhalla_run_matrix",
    srcs=["valhalla_run_matrix.cc"],
)

cc_binary(
    name="valhalla_run_route",
    srcs=["valhalla_run_route.cc"],
)

cc_binary(
    name="valhalla_service",
    srcs=["valhalla_service.cc"],
)

cc_binary(
    name="valhalla_thor_worker",
    srcs=["valhalla_thor_worker.cc"],
)

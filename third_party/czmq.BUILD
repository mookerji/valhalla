package(default_visibility=["//visibility:public"])

load("@rules_foreign_cc//tools/build_defs:configure.bzl", "configure_make")

filegroup(
    name="all",
    srcs=glob(["**"]),
)

configure_make(
    name="czmq",
    lib_source=":all",
    out_lib_dir="lib",
    deps=[],
)

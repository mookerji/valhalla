package(default_visibility=["//visibility:public"])

load("@rules_foreign_cc//tools/build_defs:configure.bzl", "configure_cmake")

filegroup(
    name="all",
    srcs=glob(["**"]),
)

configure_cmake(
    name="primeserver",
    lib_source=":all",
    out_lib_dir="lib",
    deps=[],
)

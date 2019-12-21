package(default_visibility=["//visibility:public"])

load("@rules_foreign_cc//tools/build_defs:configure.bzl", "configure_make")

filegroup(
    name="all",
    srcs=glob(["**"]),
)

configure_make(
    name="libspatialite",
    lib_source=":all",
    configure_options=[
        "--enable-geos=no",
        "--enable-proj=no",
        "--enable-freexl=no",
        "-enable-examples=no",
        "--enable-lwgeom=no",
        "--enable-gcp",
    ],
    configure_env_vars={
        "AR": "",
    },
    out_lib_dir="lib",
)

package(default_visibility=["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])  # New BSD

cc_library(
    name="lz4",
    srcs=glob(["lib/*.c"]),
    hdrs=glob([
        "lib/*.h",
        "lib/lz4.c",
    ]),
    includes=["lib"],
)

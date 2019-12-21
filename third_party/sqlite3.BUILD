package(default_visibility=["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["unencumbered"])

cc_library(
    name="sqlite3",
    srcs=["sqlite3.c"],
    hdrs=[
        "sqlite3.h",
        "sqlite3ext.h",
    ],
    copts=["-Wno-everything"],
    includes=["."],
    linkopts=[
        "-pthread",
        "-ldl",
    ],
    visibility=["//visibility:public"],
)

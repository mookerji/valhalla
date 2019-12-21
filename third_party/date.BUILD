package(default_visibility=["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

cc_library(
    name="date",
    hdrs=glob(["include/date/**/*.h"]),
    srcs=["src/tz.cpp"],
    includes=["include"],
)

workspace(name="com_mapzen_valhalla")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

http_archive(
    name="com_google_protobuf",
    url=
    "https://github.com/protocolbuffers/protobuf/releases/download/v3.11.2/protobuf-cpp-3.11.2.tar.gz",
    sha256="b967f5b667c7041415283705c0ab07f0bcc1ff077854cd29a7e148458a910053",
    strip_prefix="protobuf-3.11.2",
)

http_archive(
    name="rules_proto",
    sha256="57001a3b33ec690a175cdf0698243431ef27233017b9bed23f96d44b9c98242f",
    strip_prefix="rules_proto-9cd4f8f1ede19d81c6d48910429fe96776e567b1",
    urls=[
        "https://mirror.bazel.build/github.com/bazelbuild/rules_proto/archive/9cd4f8f1ede19d81c6d48910429fe96776e567b1.tar.gz",
        "https://github.com/bazelbuild/rules_proto/archive/9cd4f8f1ede19d81c6d48910429fe96776e567b1.tar.gz",
    ],
)

load("@rules_proto//proto:repositories.bzl", "rules_proto_dependencies",
     "rules_proto_toolchains")
rules_proto_dependencies()
rules_proto_toolchains()

git_repository(
    name="com_github_nelhage_rules_boost",
    commit="9f9fb8b2f0213989247c9d5c0e814a8451d18d7f",
    remote="https://github.com/nelhage/rules_boost",
    shallow_since="1570056263 -0700",
)
load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()

# boost modules: date_time filesystem system program_options

git_repository(
    name="rules_foreign_cc",
    commit="ed3db61a55c13da311d875460938c42ee8bbc2a5",
    remote="https://github.com/bazelbuild/rules_foreign_cc",
    shallow_since="1574792034 +0100",
)

load("@rules_foreign_cc//:workspace_definitions.bzl",
     "rules_foreign_cc_dependencies")

rules_foreign_cc_dependencies()

http_archive(
    name="sqlite3",
    build_file="//:third_party/sqlite3.BUILD",
    sha256="ad68c1216c3a474cf360c7581a4001e952515b3649342100f2d7ca7c8e313da6",
    strip_prefix="sqlite-amalgamation-3240000",
    urls=["https://www.sqlite.org/2018/sqlite-amalgamation-3240000.zip"],
)

# all_content = """filegroup(name = "all", srcs = glob(["**"]), visibility = ["//visibility:public"])"""
# build_file_content = all_content,

http_archive(
    name="libspatialite",
    build_file="//:third_party/libspatialite.BUILD",
    strip_prefix="libspatialite-4.3.0a",
    sha256="88900030a4762904a7880273f292e5e8ca6b15b7c6c3fb88ffa9e67ee8a5a499",
    urls=["http://www.gaia-gis.it/gaia-sins/libspatialite-4.3.0a.tar.gz"],
)

http_archive(
    name="openssl",
    build_file="//:third_party/openssl.BUILD",
    sha256="fb6b5de486f1739dc34f2854a0c8f94d13c130eb9c4876cad73b3d40996f8ba6",
    strip_prefix="openssl-OpenSSL_1_1_1",
    urls=["https://github.com/openssl/openssl/archive/OpenSSL_1_1_1.tar.gz"],
)

http_archive(
    name="lz4",
    build_file="//:third_party/lz4.BUILD",
    sha256="2ca482ea7a9bb103603108b5a7510b7592b90158c151ff50a28f1ca8389fccf6",
    strip_prefix="lz4-1.8.0",
    urls=["https://github.com/lz4/lz4/archive/v1.8.0.tar.gz"],
)

http_archive(
    name="primeserver",
    build_file="//:third_party/primeserver.BUILD",
    strip_prefix="prime_server-0.6.5",
    urls=["https://github.com/kevinkreiser/prime_server/archive/0.6.5.tar.gz"],
)

http_archive(
    name="libzmq",
    build_file="//:third_party/libzmq.BUILD",
    strip_prefix="zeromq-4.3.2",
    urls=[
        "https://github.com/zeromq/libzmq/releases/download/v4.3.2/zeromq-4.3.2.tar.gz"
    ],
)

http_archive(
    name="czmq",
    build_file="//:third_party/czmq.BUILD",
    strip_prefix="czmq-4.2.0",
    urls=[
        "https://github.com/zeromq/czmq/releases/download/v4.2.0/czmq-4.2.0.tar.gz"
    ],
)

http_archive(
    name="date",
    build_file="//:third_party/date.BUILD",
    sha256="98907d243397483bd7ad889bf6c66746db0d7d2a39cc9aacc041834c40b65b98",
    strip_prefix="date-2.4.1",
    urls=["https://github.com/HowardHinnant/date/archive/v2.4.1.tar.gz"],
)

http_archive(
    name="rapidjson",
    urls=["https://github.com/Tencent/rapidjson/archive/v1.1.0.tar.gz"],
    sha256="bf7ced29704a1e696fbccf2a2b4ea068e7774fa37f6d7dd4039d0787f8bed98e",
    build_file="//:third_party/rapidjson.BUILD",
    strip_prefix="rapidjson-1.1.0",
)

http_archive(
    name="lua",
    build_file="//:third_party/lua.BUILD",
    sha256="b9e2e4aad6789b3b63a056d442f7b39f0ecfca3ae0f1fc0ae4e9614401b69f4b",
    strip_prefix="lua-5.2.4",
    urls=[
        "https://mirror.bazel.build/www.lua.org/ftp/lua-5.2.4.tar.gz",
        "https://www.lua.org/ftp/lua-5.2.4.tar.gz",
    ],
)

http_archive(
    name="curl",
    build_file="//:third_party/curl.BUILD",
    sha256="52af3361cf806330b88b4fe6f483b6844209d47ae196ac46da4de59bb361ab02",
    strip_prefix="curl-7.67.0",
    urls=[
        "https://github.com/curl/curl/releases/download/curl-7_67_0/curl-7.67.0.tar.gz"
    ],
)

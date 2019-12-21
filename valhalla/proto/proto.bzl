# -*- Python -*-

# Bazel/Skylark utilities for handling Protocol Buffer definitions.


# Boilerplate handling for specifying Protocol Buffer definitions, generated
# code.
def gen_proto_block(filename, deps=[]):
    name = filename.split(".proto")[0]
    native.proto_library(
        name="proto_%s" % name,
        srcs=[filename],
        deps=deps,
    )
    native.cc_proto_library(
        name="cc_proto_%s" % name,
        deps=[":proto_%s" % name],
    )

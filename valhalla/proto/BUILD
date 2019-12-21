package(default_visibility=["//visibility:public"])

load(":proto.bzl", "gen_proto_block")

gen_proto_block(
    "api.proto",
    deps=[
        ":proto_options",
        ":proto_trip",
        ":proto_directions",
    ],
)
gen_proto_block("directions.proto", deps=[":proto_tripcommon"])
gen_proto_block("options.proto", deps=[":proto_tripcommon"])
gen_proto_block("tripcommon.proto")
gen_proto_block("trip.proto", deps=[":proto_tripcommon"])
gen_proto_block("transit.proto")
gen_proto_block("transit_fetch.proto")

gen_proto_block("fileformat.proto")
gen_proto_block("osmformat.proto")

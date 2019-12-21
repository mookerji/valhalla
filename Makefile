.PHONY: build-example
build-example:
        bazel build //src:baldr:baldr

.PHONY: clang-format-all
clang-format-all:
        @git ls-files -- './*.cc' './*.h' './*.proto' | xargs clang-format -i

.PHONY: bazel-format-all
bazel-format-all:
        @git ls-files -- '*BUILD' 'WORKSPACE' './*.bzl' | xargs yapf -i

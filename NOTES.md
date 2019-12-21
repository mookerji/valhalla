git grep --no-color -l "#include \"worker" | xargs sed -i '' -e 's/#include \"worker/#include \"valhalla\/worker/g'
cmake -P cmake/Binary2Header.cmake lua/graph.lua --variable-name lua_graph_lua  lua/graph_lua_proc.h
cmake -P cmake/Binary2Header.cmake lua/admin.lua --variable-name lua_admin_lua  lua/admin_lua_proc.h
cmake -P cmake/Binary2Header.cmake locales valhalla/odin/locales.h  --locales


 rsync -avPr --exclude 'bazel-*' . gcloud:~/projects/mapbox-repos/valhalla/
 sudo aptitude install libcurl4-openssl-dev libsqlite3-dev libxml2 libxml2-dev

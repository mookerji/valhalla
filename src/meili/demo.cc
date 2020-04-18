#include <boost/property_tree/ptree.hpp>

#include <iostream>

#include "baldr/rapidjson_utils.h"
#include "meili/demo.h"

using namespace valhalla;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "usage: map_matching CONFIG" << std::endl;
    return 1;
  }
  boost::property_tree::ptree valhalla_config;
  rapidjson::read_json(argv[1], valhalla_config);
  const matching::Configuration matching_config(valhalla_config.get_child("meili"));

  // Load road network configuration
  // Load measurements
  // Initiate map matcher
  return 0;
}

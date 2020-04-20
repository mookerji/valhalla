#include <boost/property_tree/ptree.hpp>
#include <glog/logging.h>

#include <iostream>
#include <memory>

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "meili/demo.h"
#include "sif/costfactory.h"
#include "sif/dynamiccost.h"

using namespace valhalla;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  if (argc < 3) {
    std::cout << "usage: map_matching CONFIG DATA" << std::endl;
    return 1;
  }
  // Load Configuration
  boost::property_tree::ptree root_config;
  rapidjson::read_json(argv[1], root_config);
  const matching::Config matching_config = matching::ReadConfig(root_config.get_child("meili"));
  // Load road networking graph
  std::shared_ptr<baldr::GraphReader> graph_reader =
      std::make_shared<baldr::GraphReader>(root_config.get_child("mjolnir"));
  // Load measurements
  const matching::Trajectory meas(matching::ReadMeasurements(argv[2]));

  // Sing songs related to costing
  // Costing options;
  // CHECK(valhalla::Costing_Enum_Parse(matching_config.costing.mode, &options))
  //     << "No costing method found";
  // sif::CostFactory<sif::DynamicCost> cost_factory;
  // cost_factory.RegisterStandardCostingModels();
  // sif::cost_ptr_t cost = cost_factory.Create(options.costing(), options);
  // sif::TravelMode mode = cost->travel_mode();

  // Initiate map matcher
  return 0;
}

#include <boost/property_tree/ptree.hpp>
#include <glog/logging.h>

#include <iostream>
#include <memory>

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "meili/demo.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::sif;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  if (argc < 3) {
    std::cout << "usage: map_matching CONFIG DATA" << std::endl;
    return 1;
  }
  // Load configuration, map matching, costing configuration
  boost::property_tree::ptree root_config;
  rapidjson::read_json(argv[1], root_config);
  const matching::Config matching_config = matching::ReadConfig(root_config.get_child("meili"));
  cost_ptr_t costing = matching::MakeCosting(matching_config.costing);
  TravelMode travelmode = costing->travel_mode();

  // Load road networking graph
  std::shared_ptr<baldr::GraphReader> graph_reader =
      std::make_shared<baldr::GraphReader>(root_config.get_child("mjolnir"));

  const auto& tiles = graph_reader->GetTileSet();
  DLOG(INFO) << "tiles size: " << tiles.size();

  // Load measurements
  const matching::Trajectory meas(matching::ReadMeasurements(argv[2]));

  // Initiate map matcher
  const matching::RoadNetworkIndex road_network(graph_reader, matching_config.candidate_search,
                                                &costing, travelmode);
  const auto& result =
      road_network.GetNearestEdges(matching::Measurement{PointLL(5.09806, 52.09110)});
  DLOG(INFO) << "result size: " << result.size();

  return 0;
}

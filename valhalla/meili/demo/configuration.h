#ifndef MMP_DEMO_CONFIGURATION_H_
#define MMP_DEMO_CONFIGURATION_H_

#include <string>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/sif/costfactory.h>
#include <valhalla/sif/dynamiccost.h>

using namespace valhalla::sif;

namespace valhalla {

namespace matching {

// Constants

constexpr double kDefaultSearchRadiusMeters = 50;
constexpr double kSigmaZ = 5;
constexpr double kBeta = 3;
constexpr size_t kCacheSize = 100240;
constexpr size_t kGridSize = 500;

const std::string kDefaultCostModel = "auto";

// Configuration

// TODO(mookerji): Should this be captured in a protobuf somewhere?
struct Config {

  struct CandidateSearch {
    double search_radius_meters = kDefaultSearchRadiusMeters;
    size_t cache_size = kCacheSize;
    size_t grid_size = kGridSize;
  };

  struct TransitionLikelihood {
    double beta = kBeta;
  };

  struct EmissionLikelihood {
    double sigma_z = kSigmaZ;
  };

  struct CostModel {
    std::string mode = kDefaultCostModel;
  };

  CandidateSearch candidate_search{};
  TransitionLikelihood transition{};
  EmissionLikelihood emission{};
  CostModel costing{};
};

// TODO(mookerji): Do some kind of validation here
Config ReadConfig(const boost::property_tree::ptree& conf) {
  return Config{Config::CandidateSearch{conf.get<float>("default.search_radius"),
                                        conf.get<size_t>("grid.cache_size"),
                                        conf.get<size_t>("grid.size")},
                Config::TransitionLikelihood{conf.get<float>("default.beta")},
                Config::EmissionLikelihood{conf.get<float>("default.sigma_z")},
                Config::CostModel{conf.get<std::string>("mode")}};
}

cost_ptr_t MakeCosting(Config::CostModel conf) {
  Options options;
  for (int i = 0; i < Costing_MAX; ++i) {
    options.add_costing_options();
  }
  Costing costing;
  CHECK(Costing_Enum_Parse(conf.mode, &costing)) << "No costing method found";
  CostFactory<DynamicCost> factory;
  factory.RegisterStandardCostingModels();
  return factory.Create(options);
}

} // namespace matching
} // namespace valhalla

#endif // MMP_DEMO_CONFIGURATION_H_

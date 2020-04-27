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

constexpr float kDefaultSearchRadiusMeters = 50;
constexpr size_t kCacheSize = 100240;
constexpr size_t kGridSize = 500;
constexpr float kSigmaZ = 5;
constexpr float kBeta = 3;

const std::string kDefaultCostModel = "auto";
constexpr float kMaxInterCandidateDistanceMeters = 100;
constexpr float kMaxInterCandidateTimeSeconds = 100;
constexpr float kRoutingSearchRadiusMeters = 200;

// Configuration

// TODO(mookerji): Should this be captured in a protobuf somewhere?
struct Config {

  struct CandidateSearch {
    float search_radius_meters = kDefaultSearchRadiusMeters;
    size_t cache_size = kCacheSize;
    size_t grid_size = kGridSize;
  };

  struct TransitionLikelihood {
    float beta = kBeta;
  };

  struct EmissionLikelihood {
    float sigma_z = kSigmaZ;
  };

  struct Routing {
    std::string cost_mode = kDefaultCostModel;
    float max_intercandidate_distance_meters = kMaxInterCandidateDistanceMeters;
    float max_intercandidate_time_seconds = kMaxInterCandidateTimeSeconds;
    float search_radius_meters = kRoutingSearchRadiusMeters;
  };

  CandidateSearch candidate_search{};
  TransitionLikelihood transition{};
  EmissionLikelihood emission{};
  Routing routing{};
};

// TODO(mookerji): Do some kind of validation here
Config ReadConfig(const boost::property_tree::ptree& conf) {
  return Config{Config::CandidateSearch{conf.get<float>("default.search_radius"),
                                        conf.get<size_t>("grid.cache_size"),
                                        conf.get<size_t>("grid.size")},
                Config::TransitionLikelihood{conf.get<float>("default.beta")},
                Config::EmissionLikelihood{conf.get<float>("default.sigma_z")},
                Config::Routing{conf.get<std::string>("mode")}};
}

cost_ptr_t MakeCosting(Config::Routing conf) {
  Options options;
  for (int i = 0; i < Costing_MAX; ++i) {
    options.add_costing_options();
  }
  Costing costing;
  DLOG(INFO) << "Costing mode: " << conf.cost_mode;
  CHECK(Costing_Enum_Parse(conf.cost_mode, &costing)) << "No costing method found";
  options.set_costing(costing);
  CostFactory<DynamicCost> factory;
  factory.RegisterStandardCostingModels();
  return factory.Create(options);
}

} // namespace matching
} // namespace valhalla

#endif // MMP_DEMO_CONFIGURATION_H_

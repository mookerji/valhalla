#ifndef MMP_DEMO_CONFIGURATION_H_
#define MMP_DEMO_CONFIGURATION_H_

#include <string>

#include <boost/property_tree/ptree.hpp>

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
struct Configuration {

  struct CandidateSearch {
    double search_radius_meters = kDefaultSearchRadiusMeters;
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

  struct Grid {
    size_t cache_size = kCacheSize;
    size_t grid_size = kGridSize;
  };

  explicit Configuration() = default;

  explicit Configuration(const boost::property_tree::ptree& config)
      : candidate_search(CandidateSearch{config.get<float>("default.search_radius")}),
        transition(TransitionLikelihood{config.get<float>("default.beta")}),
        emission(EmissionLikelihood{config.get<float>("default.sigma_z")}),
        costing(CostModel{config.get<std::string>("mode")}),
        grid(Grid{config.get<size_t>("grid.cache_size"), config.get<size_t>("grid.size")})

  {
  }

  CandidateSearch candidate_search{};
  TransitionLikelihood transition{};
  EmissionLikelihood emission{};
  CostModel costing{};
  Grid grid{};
};

} // namespace matching
} // namespace valhalla

#endif // MMP_DEMO_CONFIGURATION_H_

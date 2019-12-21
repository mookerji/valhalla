#include <iostream>
#include <utility>
#include <vector>

#include "valhalla/baldr/streetnames.h"
#include "valhalla/baldr/streetnames_factory.h"
#include "valhalla/baldr/streetnames_us.h"
#include "valhalla/midgard/util.h"

namespace valhalla {
namespace baldr {

std::unique_ptr<StreetNames>
StreetNamesFactory::Create(const std::string& country_code,
                           const std::vector<std::pair<std::string, bool>>& names) {
  if (country_code == "US") {
    return std::make_unique<StreetNamesUs>(names);
  }

  return std::make_unique<StreetNames>(names);
}

} // namespace baldr
} // namespace valhalla

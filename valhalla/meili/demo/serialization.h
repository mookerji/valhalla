#ifndef MMP_DEMO_SERIALIZATION_H_
#define MMP_DEMO_SERIALIZATION_H_

#include <string>
#include <vector>

#include <valhalla/meili/demo/spatial.h>

namespace valhalla {

namespace matching {

// Serialization to Valhalla and OSRM-style responses

class OSRMResponse {

  struct Matching {};

public:
  OSRMResponse() {
  }
};

class ValhallaResponse {

public:
  ValhallaResponse() {
  }
};

// TODO(mookerji): serialize to GeoJSON

template <typename T> std::string ToGeoJSON(const T& geo) {
  return "";
}

template <> std::string ToGeoJSON(const Measurement& geo) {
  return "Measurement";
}

template <> std::string ToGeoJSON(const Tracepoint& geo) {
  return "Tracepoint";
}

template <> std::string ToGeoJSON(const ObservationSet& geo) {
  return "Measurements";
}

} // namespace matching
} // namespace valhalla

#endif // MMP_DEMO_SERIALIZATION_H_

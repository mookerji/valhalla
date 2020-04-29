#ifndef MMP_DEMO_MEASUREMENT_H_
#define MMP_DEMO_MEASUREMENT_H_

#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <glog/logging.h>

#include <valhalla/midgard/pointll.h>

#include <valhalla/meili/demo/configuration.h>
#include <valhalla/meili/demo/macros.h>

// FIX(mookerji): Don't use globbing namespace imports
using namespace valhalla::midgard;

namespace valhalla {

namespace matching {

// Internal Trajectory measurements

struct Measurement {
  struct Data {
    PointLL lnglat;
  };
  Data data{};
  size_t measurement_id = -1;
};

std::ostream& operator<<(std::ostream& os, const Measurement& meas) {
  os << "Point[lat=" << meas.data.lnglat.lat() << ", lng=" << meas.data.lnglat.lng()
     << ", measurement_id=" << meas.measurement_id << "]";
  return os;
}

Measurement ReadMeasurement(std::string line) {
  DLOG(INFO) << "measurement line: " << line;
  std::vector<float> records;
  std::istringstream ss(line);
  size_t num_cols = 0;
  for (std::string col; std::getline(ss, col, ','); ++num_cols) {
    CHECK(num_cols < 2) << "Only 'lng,lat' per line.";
    records.emplace_back(std::stof(col));
  }
  return Measurement{PointLL(records[1], records[0])};
}

struct Tracepoint {
  PointLL location;
  float distance;
  std::string name;

  size_t waypoint_index;
  size_t matchings_index;
  size_t alternatives_count;
};

class Trajectory {

public:
  Trajectory() = default;

  Trajectory(const std::vector<Measurement>& meas) : measurements_(meas) {
  }

  Measurement& operator[](size_t i) {
    return measurements_.at(i);
  }

  const Measurement& operator[](size_t i) const {
    return measurements_.at(i);
  }

  const void Add(const Measurement& meas) {
    measurements_.emplace_back(meas);
  }

  size_t size() const {
    return measurements_.size();
  }

  bool empty() const {
    return measurements_.empty();
  }

private:
  VL_DISALLOW_COPY_AND_ASSIGN(Trajectory);
  std::vector<Measurement> measurements_;
};

std::vector<Measurement> ReadMeasurements(std::string filename) {
  DLOG(INFO) << "Loading from ..." << filename;
  std::vector<Measurement> measurements;
  std::fstream fs(filename, std::fstream::in);
  std::string line;
  for (size_t line_count = 0; !fs.eof(); ++line_count) {
    std::getline(fs, line);
    if (line.empty()) {
      continue;
    }
    Measurement meas = ReadMeasurement(line);
    meas.measurement_id = line_count;
    measurements.emplace_back(meas);
  }
  return measurements;
}

} // namespace matching
} // namespace valhalla

#endif // MMP_DEMO_MEASUREMENT_H_

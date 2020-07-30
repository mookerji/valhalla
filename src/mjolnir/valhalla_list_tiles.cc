// Emits a file manifest of tiles to stdout
#include <iostream>
#include <sstream>
#include <string>

#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>

#include "baldr/graphid.h"
#include "baldr/rapidjson_utils.h"
#include "filesystem.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "mjolnir/graphbuilder.h"

namespace bpo = boost::program_options;

using namespace valhalla;
using Config = boost::property_tree::ptree;

struct ProgramOptions {
  std::string config_file_path;
  std::string inline_config;
  std::string nodes_filename;
  std::string edges_filename;
  bpo::variables_map variables;
};

ProgramOptions ReadProgramOptions(int argc, char** argv) {
  bpo::options_description spec("valhalla_build_tiles \n\n");
  ProgramOptions options;
  spec.add_options()
      //
      ("help,h", "Print help message")
      //
      ("version,v", "Print software version")
      //
      ("config,c", bpo::value<std::string>(&options.config_file_path), "JSON configuration file")
      //
      ("inline-config,i", bpo::value<std::string>(&options.inline_config), "Inline JSON config")
      //
      ("nodes-file,n", bpo::value<std::string>(&options.nodes_filename), "Nodes file")
      //
      ("edges-file,e", bpo::value<std::string>(&options.edges_filename), "Edges file");
  bpo::store(bpo::command_line_parser(argc, argv).options(spec).run(), options.variables);
  bpo::notify(options.variables);
  return options;
}

Config ReadProgramConfig(const ProgramOptions& options) {
  Config conf;
  if (options.variables.count("inline-config")) {
    std::stringstream ss;
    ss << options.inline_config;
    rapidjson::read_json(ss, conf);
  } else if (options.variables.count("config") &&
             filesystem::is_regular_file(options.config_file_path)) {
    rapidjson::read_json(options.config_file_path, conf);
  } else {
    // FIX
  }
  return conf;
}

void ConfigureLogging(const Config& conf) {
  const auto& logging_subtree = conf.get_child_optional("mjolnir.logging");
  if (logging_subtree) {
    const std::unordered_map<std::string, std::string>& logging_config =
        midgard::ToMap<const Config&, std::unordered_map<std::string, std::string>>(
            logging_subtree.get());
    midgard::logging::Configure(logging_config);
  }
}

// TODO(mookerji): This should be able to work from a PBF file (using valhalla_build_tiles) and not
// require these memory-mapped intermediates.
std::string BuildManifest(const ProgramOptions& options, const Config& conf) {
  const std::map<baldr::GraphId, size_t>& tiles =
      mjolnir::GraphBuilder::ListTiles(conf, options.nodes_filename, options.edges_filename);
  if (tiles.empty()) {
    return {};
  }
  std::stringstream manifest;
  manifest << "{\"tiles\": [ ";
  for (const auto& tile : tiles) {
    manifest << "{\"tileid\": " << std::to_string(tile.first.tileid()) << "},";
  }
  // HACK(mookerji): gag me with a spoon
  std::string result = manifest.str();
  result.pop_back();
  return result + "]}";
}

int main(int argc, char** argv) {
  const ProgramOptions& options = ReadProgramOptions(argc, argv);
  const Config& conf = ReadProgramConfig(options);
  ConfigureLogging(conf);
  std::cout << BuildManifest(options, conf);
}

#pragma once
#include <string>

namespace kuroko_isaac_config_generator::ros_resource_resolution {

// Resolve package://<pkg>/<path> to an absolute path.
// Returns empty string if it cannot be resolved.
std::string resolvePackageUri(const std::string& uri);

}  // namespace kuroko_isaac_config_generator::ros_resource_resolution

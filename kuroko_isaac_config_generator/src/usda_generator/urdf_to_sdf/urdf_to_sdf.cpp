#include <iostream>
#include <string>

#include <sdf/Root.hh>
#include <sdf/ParserConfig.hh>

int main(int argc, char** argv)
{
  if (argc < 2) {
    std::cerr << "Usage: urdf_to_sdf <path_to_urdf>\n";
    return 2;
  }

  const std::string urdf_path = argv[1];

  sdf::ParserConfig config;
  config.SetURDFPreserveFixedJoint(true);

  sdf::Root root;
  const auto errors = root.Load(urdf_path, config);
  if (!errors.empty()) {
    std::cerr << "Failed to load/convert URDF -> SDF. Errors:\n";
    for (const auto& e : errors) {
      std::cerr << "  - " << e.Message() << "\n";
    }
    return 1;
  }

  // Root::ToString() is available in newer sdformat versions; Element() exists broadly.
  if (root.Element()) {
    std::cout << root.Element()->ToString("");
  } else {
    std::cerr << "Converted SDF root has no XML element.\n";
    return 1;
  }

  return 0;
}

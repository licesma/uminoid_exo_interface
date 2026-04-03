#pragma once

#include <filesystem>
#include <string>

namespace repo_constants {

inline const std::string DATA_DIR =
    (std::filesystem::path(__FILE__).parent_path() / ".." / ".." / "data").lexically_normal().string();

}

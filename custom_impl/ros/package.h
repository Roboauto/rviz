#pragma once

#include <string>
#include <vector>

#include "macros.h"


namespace ros {
    namespace package {

        void command(const std::string &cmd, std::vector<std::string> &output);

        std::string command(std::string _cmd);

        std::string getPath(std::string package_name);

        void getPlugins(const std::string &package, const std::string &attrib_name, std::vector<std::string> &plugins,
                   bool recrawl = false);
    }
}
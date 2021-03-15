#include "package.h"

#include <rospack/rospack.h>

#include <iostream>
#include <boost/algorithm/string/split.hpp>
#include <pluginlib/class_loader.hpp>

using namespace ros::package;

inline std::string ros::package::command(std::string _cmd) {
    static rospack::ROSPack rp;
    int ret;
    try {
        ret = rp.run(_cmd);
        if (ret == 0)
            return rp.getOutput();
        else {
            if (!rp.is_quiet())
                std::cerr << "ROSPack::run returned non-zero." << std::endl;
        }
    }
    catch (std::runtime_error &e) {
        if (!rp.is_quiet())
            std::cerr << "[rospack] " << e.what() << std::endl;
    }
    return std::string("");
}

void ros::package::command(const std::string &cmd, std::vector<std::string> &output) {
    std::string out_string = command(cmd);
    std::vector<std::string> full_list;
    boost::split(full_list, out_string, boost::is_any_of("\r\n"));

    // strip empties
    std::vector<std::string>::iterator it = full_list.begin();
    std::vector<std::string>::iterator end = full_list.end();
    for (; it != end; ++it) {
        if (!it->empty()) {
            output.push_back(*it);
        }
    }
}

std::string ros::package::getPath(std::string package_name) {
    std::string path = command("find " + package_name);

    // scrape any newlines out of it
    for (size_t newline = path.find('\n'); newline != std::string::npos;
         newline = path.find('\n')) {
        path.erase(newline, 1);
    }

    return path;
}

void
ros::package::getPlugins(const std::string &package, const std::string &attribute, std::vector<std::string> &plugins,
                         bool recrawl) {


    std::vector<std::string> lines;
    command("plugins --attrib=" + attribute + " " + package, lines);

    std::vector<std::string>::iterator it = lines.begin();
    std::vector<std::string>::iterator end = lines.end();
    for (; it != end; ++it) {
        std::vector<std::string> tokens;
        boost::split(tokens, *it, boost::is_any_of(" "));

        if (tokens.size() >= 2) {
            std::string package = tokens[0];
            std::string rest = boost::join(std::vector<std::string>(tokens.begin() + 1, tokens.end()), " ");
            plugins.push_back(rest);
        }
    }
}

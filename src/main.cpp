/*
 * @Date: 2022-05-26 09:51:08
 * @LastEditors: lzxiaohuihui 895827938@qq.com
 * @LastEditTime: 2022-06-14 20:01:55
 * @FilePath: /mySfmUsingCV/src/main.cpp
 * @Description: 练练手
 */

#include "sfm.h"
#include <iostream>
#include <boost/program_options.hpp>

using namespace std;
namespace po = boost::program_options;

int main(int argc, char** argv) {
    //add command line options
    po::options_description od;
    od.add_options()
            ("help,h",                                                          "Produce help message.")
            ("downscale,s",       po::value<double>()->default_value(1.0),      "Downscale factor for input images.")
            ("input-directory,p", po::value<string>()->required(),              "Directory to find input images.")
            ("output-prefix,o",   po::value<string>()->default_value("output"), "Prefix for output files.")
            ;

    po::positional_options_description op;
    op.add("input-directory", 1);

    //parse options
    po::variables_map varMap;
    try {
        po::store(po::command_line_parser(argc, argv).positional(op).options(od).run(), varMap);
        po::notify(varMap);
    } catch (const std::exception& e) {
        cerr << "Error while parsing command line options: " << e.what() << endl
             << "USAGE " << argv[0] << " [options] <" << op.name_for_position(0) << ">" << endl << od;
        exit(0);
    }
    if (varMap.count("help")) {
        cerr << argv[0] << " [options] <" << op.name_for_position(0) << ">" << endl << od << endl;
        exit(0);
    }

    Sfm sfm(varMap["downscale"].as<double>());
    sfm.setImagesDir(varMap["input-directory"].as<string>());
    sfm.runSfm();

    return 0;

}

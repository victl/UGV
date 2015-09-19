#include "config.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <glog/logging.h>
#include <boost/program_options.hpp>

namespace victl {

bool UgvParam::loadParam(std::string configFile)
{
    using namespace boost::program_options;
    options_description conf;
    conf.add_options()
            ("Ugv.CorrectionFile", value<std::string>(&Ugv.CorrectionFile)->default_value("new_xml.txt"),"The correction file for current HDL")
            ("Hdl.HdlVersion", value<unsigned short>(&Hdl.HdlVersion)->default_value(3),"The version of Hdl file.\n"
                                                                                "Version 1: points doesn't contain xyzs. Date before: 2015-9-9\n"
                                                                                "Version 2: points contain xyzs, car coordinate. Since version 2, camera points were added. Date between: 2015-9-9 ~ 2015-9-18\n"
                                                                                "Version 3: points contain xyzs, global coordinate. Date after: 2015-9-18")
            ("Hdl.MaxCP", value<unsigned short>(&Hdl.MaxCP)->default_value(500),"Maximum camera points")
            ("Hdl.RecordLocalMap", value<bool>(&Hdl.RecordLocalMap)->default_value(1),"Indicate record local map or not.")
            ("Hdl.NegHeight", value<int>(&Hdl.NegHeight)->default_value(-1850),"Negtive height of LiDAR, used as a threshold to distinguish ground points")
            ("DivideCarTrack.EulrChangeThreshold", value<double>(&DivideCarTrack.EulrChangeThreshold)->default_value(0.005),"")
            ("DivideCarTrack.DetectPoints", value<int>(&DivideCarTrack.DetectPoints)->default_value(10),"The range of detection, Measured by points")
            ("DivideCarTrack.DetectDistance", value<int>(&DivideCarTrack.DetectDistance)->default_value(5),"The range of detection. Measured by meters")
            ("DivideCarTrack.ValidSegmentPointsNum", value<int>(&DivideCarTrack.ValidSegmentPointsNum)->default_value(80),"Defines how many points to be recognized as a valid segment")
            ("LineParallel.SimilarEulrThreshold", value<double>(&LineParallel.SimilarEulrThreshold)->default_value(0.01),"Only when eulr change exceeds this threshold do we confirm that the ugv is on a turn")
            ("SameSeg.SameDirectionThreshold", value<double>(&SameSeg.SameDirectionThreshold)->default_value(0.01),"Only when direction of two segments differ more than this threshold do we accept them as a possible same segment")
            ("SameSeg.LateralDistanceThreshold", value<double>(&SameSeg.LateralDistanceThreshold)->default_value(20),"Only when lateral distance of two segments differ more than this threshold do we accept them as a possible same segment")
            ("Scale.xMin", value<double>(&Scale.xMin)->default_value(-65),"Detect range of HdlEngine - minimum x")
            ("Scale.xMax", value<double>(&Scale.xMax)->default_value(65),"Detect range of HdlEngine - maximum x")
            ("Scale.yMin", value<double>(&Scale.yMin)->default_value(-65),"Detect range of HdlEngine - minimum y")
            ("Scale.yMax", value<double>(&Scale.yMax)->default_value(65),"Detect range of HdlEngine - maximum y")
            ("Scale.GridSize", value<double>(&Scale.GridSize)->default_value(0.2),"Size of a single grid. Measured by meters")
            ("Scale.PixelPerGrid", value<int>(&Scale.PixelPerGrid)->default_value(5),"When visualize detect result, defines how many pixels to represent a grid")
            ("ProbMap.LeftDetectAngleBoundary", value<unsigned short>(&ProbMap.LeftDetectAngleBoundary)->default_value(34000),"Left detect angle")
            ("ProbMap.RightDetectAngleBoundary", value<unsigned short>(&ProbMap.RightDetectAngleBoundary)->default_value(2000),"Right detect angle")
            ("ProbMap.unitHeight", value<unsigned short>(&ProbMap.unitHeight)->default_value(70),"Height increment unit")
            ("ProbMap.HeightThreshold", value<unsigned short>(&ProbMap.HeightThreshold)->default_value(250),"Height threshold")
            ("ProbMap.incrementUnit", value<float>(&ProbMap.incrementUnit)->default_value(0.05),"Probability increment unit")
            ("ProbMap.OccupiedThreshold", value<float>(&ProbMap.OccupiedThreshold)->default_value(0.15),"Point 'OCCUPIED' probability threshold")
            ("ProbMap.ClearThreshold", value<float>(&ProbMap.ClearThreshold)->default_value(0.3),"Point 'CLEAR' probability threshold")
            ("ProbMap.MaxGroundHeight", value<short>(&ProbMap.MaxGroundHeight)->default_value(-1600),"Height of ground points should not exceed this threshold")
            ("ProbMap.MaxAvgMidDiff", value<float>(&ProbMap.MaxAvgMidDiff)->default_value(0.25),"The interval between avrage and middle, relative to highest - lowest")
            ("LocalMap.SaveInterval", value<unsigned int>(&LocalMap.SaveInterval)->default_value(0),"Interval to save/display result, measured by frame")
            ("LocalMap.SaveNeeded", value<std::vector<int> >(),"Specify which frame to save");
    //            ("", value<int>()->default_value(),"")
    //            ("", value<int>()->default_value(),"")
    variables_map vm;
    try {
        store(parse_config_file<char>(configFile.c_str(), conf), vm);
    } catch (const reading_file& e) {
        std::cout << "Failed to open file " << configFile << ": "
                  << e.what();
    }
    notify(vm);
    //because boost_program_options does not support std::set as container, we need to insert it manually
    LocalMap.SaveNeeded.insert(vm["LocalMap.SaveNeeded"].as<std::vector<int> >().begin(), vm["LocalMap.SaveNeeded"].as<std::vector<int> >().end());
    update();
    return true;
}

void UgvParam::update()
{
    Scale.Width = (Scale.xMax - Scale.xMin)/ Scale.GridSize;
    Scale.xScale = 1 / Scale.GridSize;
    Scale.yScale = 1 / Scale.GridSize;
    //LocalMap
    // n times of Scale.Width
    LocalMap.initialHeight = 3 * round(Scale.Width);
    LocalMap.initialWidth = 3 * round(Scale.Width);
    LocalMap.ExpandUnit = 3 * round(Scale.xMax - Scale.xMin);
}
}//end namespace victl

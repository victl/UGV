/**
  * This header file contains Adjustable parameters that could change between different run.
  * These parameters could be read from a configuration file, so changes could be made within
  * that file and take effect without recompiling the whole program.
  *
  * Author: Zou Lu (victl@163.com)
  * Date: 2015-9-1
  */


#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <set>
#include <map>

namespace victl {

//this enum is not used for the moment, because using them is so tiring!!!
enum Parameter/* : unsigned char */{
    CORRECTIONFILE, HDLFORMAT
    , EULRCHANGETHRESHOLD, DETECTPOINTS, DETECTDISTANCE, VALIDSEGMENTPOINTSNUM
    , SIMILAREULRTHRESHOLD
    , SAMEDIRECTIONTHRESHOLD, LATERALDISTANCETHRESHOLD
    , XMIN, XMAX, YMIN, YMAX, GRIDSIZE, PIXELPERGRID, WIDTH, XSCALE, YSCALE
    , LEFTDETECTANGLEBOUNDARY, RIGHTDETECTANGLEBOUNDARY, UNITHEIGHT, HEIGHTTHRESHOLD, INCREMENTUNIT
    , OCCUPIEDTHRESHOLD, CLEARTHRESHOLD, MAXGROUNDHEIGHT, MAXAVGMIDDIFF
    , INITIALWIDTH, INITIALHEIGHT, EXPANDUNIT, SAVEINTERVAL, SAVENEEDED
};

class UgvParam {
public:
    UgvParam()
    {
        restoreDefault();
        loadParam();
    }

    bool loadParam(std::string configFile = "/home/denggroup/UGV/bin/ugv.conf");
    void restoreDefault();
    //Because many params were calculated by other params. Every time other params alterred,
    //invoking update() to set calculated value is required.
    void update();



private:
//Ugv
    struct Ugv_t {
        std::string CorrectionFile;
    };
    struct Hdl_t {
        //Version 1: points doesn't contain xyzs
        //Version 2: points contain xyzs, car coordinate. Since version 2, camera points were added.
        //Version 3: points contain xyzs, global coordinate
        unsigned char HdlVersion;
        //maximum camera points
        unsigned short MaxCP;
        bool RecordLocalMap;
    };

//DivideCarTrack
    struct DivideCarTrack_t {
        double EulrChangeThreshold;
        int DetectPoints;
        int DetectDistance;
        int ValidSegmentPointsNum;
    };

//LineParallel
    struct LineParallel_t {
        double SimilarEulrThreshold;
    };


//SameSeg
    struct SameSeg_t {
        double SameDirectionThreshold;
        double LateralDistanceThreshold;
    };


//Scale
    struct Scale_t {
        double xMin;
        double xMax;
        double yMin;
        double yMax;
        double GridSize;
        int PixelPerGrid;
        double Width;
        double xScale;
        double yScale;

    };

//Probability map
    struct ProbMap_t {
        //left detect angle
        unsigned short LeftDetectAngleBoundary;
        //right detect angle
        unsigned short RightDetectAngleBoundary;
        //height increment unit
        unsigned char unitHeight;
        //height threshold
        unsigned short HeightThreshold;
        //probability increment unit
        float incrementUnit;
        //Point type probability threshold
        unsigned char OccupiedThreshold;
        float ClearThreshold;
        short MaxGroundHeight;
        float MaxAvgMidDiff;
    };

//Local map
    struct LocalMap_t {
        unsigned int initialWidth;
        unsigned int initialHeight;
        unsigned short ExpandUnit;
        unsigned int SaveInterval;
        std::set<unsigned int> SaveNeeded;
    };

public:
    struct Ugv_t Ugv;
    struct Hdl_t Hdl;
    struct DivideCarTrack_t DivideCarTrack;
    struct LineParallel_t LineParallel;
    struct SameSeg_t SameSeg;
    struct Scale_t Scale;
    struct ProbMap_t ProbMap;
    struct LocalMap_t LocalMap;
};
}//end namespace victl
#endif // CONFIG_H

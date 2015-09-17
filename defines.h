/**
  * This header file contains many preprocessor defines and some fundamental types.
  * Most of them do not change unless the physical model of HDL Lader changed.
  * Adjustable parameters should be put in config.h file.
  *
  * Author: Zou Lu (victl@163.com)
  * Date: 2015-9-1
  */
#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include "config.h"

#ifndef DEFINES_H
#define DEFINES_H

namespace victl {

//define some constant
#define ANGLE_NUM 36000 //divide the whole 360 degree to small pieces
#define LASER_NUM 64    //the total num of laser beam from the ladar is 64
#define MAX_CLOUD_SIZE 256000    //maximum possible point number

//This data is observed by calculate average z of ground points when car didn't move, unit length 1mm
//Specific to Chang An CS35 model
#define HDL_HEIGHT 1854

//struct used to store raw hdl data
typedef struct
{
    unsigned short distance;    //corresponse to original 'dist'
    unsigned short rotAngle;   //corresponse to original 'rot'
    unsigned char intensity;    //corresponse to original 'i'
    unsigned char beamId;    //corresponse to original 'c'
}RawHdlPoint;

typedef struct{
    int x;
    int y;
    int z;
}HdlPointXYZ;

//The 'LPoint_t' is used for record native HDL points
//Currently, only use it when recording data online (on car) - until 2015-9-10
typedef struct
{
    int x;
    int y;
    int z;
    unsigned short distance;
    unsigned short rotAngle;  // ��ת��
    unsigned char intensity;     // ǿ����Ϣ
    unsigned char beamId;     // ɨ��������
}HdlPoint;

typedef struct{
    double x;
    double y;
    double roll;
    double pitch;
    double eulr;
    double speed;
}Carpose;

typedef struct {
    double x;
    double y;
    double eulr;
}SimpleCarpose;


//NOTE: "unsigned char" for enum is a new feature introduced in c++11 standard
//so you have to enable c++11 support of compilers.
enum PointAttrib /*: unsigned char */{
    LANELINE = 254,
    ZEBRA = 253,
    INTERSECTION = 3,
    CURB = 4,
    TREE = 210,
    TRUNK = 6,
    PIT = 252,
    LANECENTER = 8,
    CARTRACK = 128,
    TRAFFICSIGN = 255,
    DOTTEDLANELINE = 251,
    SOLIDLANELINE = 250,
    CAMERALANELINE = 249,
    CAMERASTOPLINE = 248,
    CAMERALSINTERSECT = 247, //intersection of camera laneline and stopline. This info might be useful for Wangshy
    AUNKNOWN = 0, // Attribute unknown
};

enum PointOccupation /*: unsigned char */{
    CLEAR = 2,//'CLEAR' means point could pass trough
    OCCUPIED = 1,
    OUNKNOWN = 0, // Occupation unknown
};

enum MapType /*: unsigned char*/ {
    DYNAMICMAP,//dynamic map
    ACCUMMAP,//accumulated map
    LOCALMAP,//local map
    THREEBIT,//3b format
};

//Point3B is used to output final grid map format (formerly .3b file, now has switched to .png)
struct Point3B {
public:
    //public member variable
    unsigned char base;
    unsigned char road;
    unsigned char sig;
    //public method:
    Point3B():
        base(0)
      , road(0)
      , sig(0)
    {
    }
};

//Grid type. It is used for storing dynamic map and accumulated map infos, but not for local map
typedef struct Grid_t{
    //the probability of occupation:
//    float p;
    //highest 'z' value
    short highest;
    //lowest 'z' value
    short lowest;
    //the average height//testing
//    short average;//testing
    //how many points in the grid
    unsigned short pointNum;
    unsigned char HitCount;
    //occupation
    PointOccupation o;
    //attribute
    PointAttrib a;

//    Grid_t(){
//        p = 0.5;
//        highest = 0;
//        lowest = 0;
//        average = 0;//testing
//        pointNum = 0;
//        HitCount = 0;
//        o = OUNKNOWN;
//        a = AUNKNOWN;
//    }

//    Grid_t &operator+=(const Grid_t& other);
}Grid;


//Range indicates the boundary of the map under process. The origin is at Differential GPS Station!
class Range{
public/*constructor && destructor && helper*/:
    explicit Range(const UgvParam& param);
    Range(const Carpose& currentPose, const UgvParam& param);

public/*data member*/:
    const UgvParam &params;
    double bottom;
    double top;
    double left;
    double right;
    double centerX;
    double centerY;
    int maxX;
    int maxY;

public/*function member*/:
    //if point (x,y) didn't fall into the range, return false
    inline bool contains(double x, double y);

    //translate global coordinate to local coordinate (origin at bottom-left corner of the range)
    //It returns false if the point fall outside the range
    bool toLocal(const double x, const double y, int &retX, int &retY);

    //translate local coordinate to global.
    //and returns the result coordinate as a cv::Point2d.
    //NOTE: grid's global position is on its mid-center
    cv::Point2d toGlobal(int x, int y);

    //translate (x,y) to another range 'other'. values is return by otherx, othery. if (x,y) does not fall in otherRange, return false
    bool translate(int x, int y, Range& otherRange, int& otherx, int& othery);

    void distanceTo(Range& otherRange, int& deltax, int& deltay);

    //copy assignment operator
    Range &operator=(const Range& source);

    //equality operator
    bool operator==(const Range& other);
    bool operator!=(const Range& other);

    //update maxX, maxY whenever boundaries are adjusted explicitly (because they are 'public' members).
    //THIS IS VERY IMPORTANT!!!
    bool update();
};

//Global functions
std::string to_string(int num);
bool isPresent(unsigned char value, unsigned char property);
void mergeGrid(Grid& base, Grid& addition);

//define binary values of Point3B.base
static const unsigned char ROADEDGE_UNKNOWN = 0;
static const unsigned char ROADEDGE_CLEAR = 64;
static const unsigned char ROADEDGE_OCCUPIED = 128;
static const unsigned char ROADEDGE_DYNAMIC = 192;
static const unsigned char OBSTACLE_NONE = 0;
static const unsigned char OBSTACLE_STATIC = 8;
static const unsigned char OBSTACLE_PEDESTRIAN = 16;
static const unsigned char OBSTACLE_BIKE = 24;
static const unsigned char OBSTACLE_MOTO = 32;
static const unsigned char OBSTACLE_CAR = 40;
static const unsigned char OBSTACLE_BUS = 48;
static const unsigned char OBSTACLE_TRUCK = 56;
static const unsigned char LANELINE_NONE = 0;
//static const unsigned char LANELINE_DOTTED = 2;
static const unsigned char LANELINE_HDL = 2;
//static const unsigned char LANELINE_SOLID = 4;
static const unsigned char LANELINE_CAMERA = 4;
static const unsigned char LANELINE_DOUBLE = 6;
static const unsigned char STOPLINE_NO = 0;
static const unsigned char STOPLINE_YES = 1;

//define binary values of Point3B.road
static const unsigned char CURB_NO = 0;
static const unsigned char CURB_YES = 128;
static const unsigned char FENCERAMP_NO = 0;
static const unsigned char FENCERAMP_CITY = 32;
static const unsigned char FENCERAMP_HIGHWAY = 64;
static const unsigned char FENCERAMP_RAMP = 96;
static const unsigned char REGION_STRUCTURED = 0;
static const unsigned char REGION_INTERSECTION = 8;
static const unsigned char REGION_UTURN = 16;
static const unsigned char REGION_RIM = 24;
static const unsigned char ARROW_NONE = 0;
static const unsigned char ARROW_STRAIGHT = 1;
static const unsigned char ARROW_LEFT = 2;
static const unsigned char ARROW_RIGHT = 3;
static const unsigned char ARROW_UTURN = 4;
static const unsigned char ARROW_STRAIGHTLEFT = 5;
static const unsigned char ARROW_STRAIGHTRIGHT = 6;

//define binary values of Point3B.sig
static const unsigned char LAMP_NONE = 0;
static const unsigned char LAMP_ROUND = 64;
static const unsigned char LAMP_ARROW = 128;
static const unsigned char LAMP_GROUND = 192;
//the definition of traffic sign is uncertain for the moment, to be added in the future

}//end namespace victl
#endif // DEFINES_H


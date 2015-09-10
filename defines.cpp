#include "defines.h"
#include <cmath>


Range::Range(const UgvParam& param)
    : params(param)
{
    left = right = top = bottom = 0;
    maxX = maxY = 0;
}

Range::Range(const Carpose &currentPose, const UgvParam &param)
    : params(param)
{
    left = floor((currentPose.x + params.Scale.xMin) / params.Scale.GridSize)
            * params.Scale.GridSize;
    right = floor((currentPose.x + params.Scale.xMax) / params.Scale.GridSize)
            * params.Scale.GridSize;
    bottom = floor((currentPose.y + params.Scale.xMin) / params.Scale.GridSize)
            * params.Scale.GridSize;
    top = floor((currentPose.y + params.Scale.xMax) / params.Scale.GridSize)
            * params.Scale.GridSize;
    update();
}

bool Range::contains(double x, double y)
{
    if(x < left || x >= right || y < bottom || y >= top)
    {
        return false;
    }
    return true;
}

bool Range::toLocal(double &x, double &y)
{
    if(!this->contains(x,y))
    {
        return false;
    }
    x = floor( (x - left) / params.Scale.GridSize);
    if(x == maxX) --x;
    y = floor( (y - bottom) / params.Scale.GridSize);
    if(y == maxY) --y;
    return true;
}

cv::Point2d Range::toGlobal(unsigned short x, unsigned short y)
{
    double xx = left + (x + 0.5) * params.Scale.GridSize;
    double yy = bottom + (y + 0.5) * params.Scale.GridSize;
    return cv::Point2d(xx, yy);
}

bool Range::translate(unsigned short x, unsigned short y, Range &oldRange, unsigned short& oldx, unsigned short& oldy)
{
    cv::Point2d pt = this->toGlobal(x,y);
    if(oldRange.toLocal(pt.x, pt.y)){
        oldx = pt.x;
        oldy = pt.y;
        return true;
    }
    return false;
}

Range &Range::operator=(const Range &source)
{
    bottom = source.bottom;
    top = source.top;
    left = source.left;
    right = source.right;
    maxX = source.maxX;
    maxY = source.maxY;
    return *this;
}

bool Range::update()
{
    maxX = (right - left) / params.Scale.GridSize;
    maxY = (top - bottom) / params.Scale.GridSize;
    return true;
}


Grid_t &Grid_t::operator+=(const Grid_t &other)
{
    if(abs(this->p - 0.5) < abs(other.p - 0.5) )
    {
        this->p = other.p;
    }
    this->highest < other.highest ? this->highest = other.highest : 0;
    this->lowest > other.lowest ? this->lowest = other.lowest : 0;
//    if (this->pointNum + other.pointNum == 0)
//    {
//        this->average = 0;
//    }
//    else{
//        this->average = (this->average * this->pointNum + other.average * other.pointNum) / (this->pointNum + other.pointNum);
//    }
    this->pointNum += other.pointNum;
    this->HitCount += other.HitCount;
    return *this;
}

std::string to_string(int num)
{
    char str[10];
    sprintf(str,"%d", num);
    return std::string(str);
}

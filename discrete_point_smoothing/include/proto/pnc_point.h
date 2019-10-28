#pragma once

namespace ADC {
namespace planning {

class PathPoint {
public:
    // coordinates
    double x ;
    double y ;
    double z ;

    // direction on the x-y plane
    double theta ;
    // curvature on the x-y planning
    double kappa ;
    // accumulated distance from beginning of the path
    double s ;

    // derivative of kappa w.r.t s.
    double dkappa ;
    // derivative of derivative of kappa w.r.t s.
    double ddkappa ;
    // The lane ID where the path point is on

    // derivative of x and y w.r.t parametric parameter t in CosThetareferenceline
    double x_derivative ;
    double y_derivative ;
};

class AnchorPoint {
public:
    PathPoint path_point;
    double lateral_bound = 0.0;
    double longitudinal_bound = 0.0;
    // enforce smoother to strictly follow this reference point
    bool enforced = false;
    // 增加index,为快速查找后期的平滑点距离原始参考线的最近的点
    int index;
};

class SmoothedReferencePoint {
public:
    double x;
    double y;
    double heading;
    double s;
    double kappa;
    double dkappa;
};

class dPoint {
public:
    double x;
    double y;
};

class dPoint3d {
public:
    double x;
    double y;
    double heading;

    /// get the x position
    float getX() const { return x; }
    /// get the y position
    float getY() const { return y; }

    void setX(const float& x) { this->x = x; }
    /// set the y position
    void setY(const float& y) { this->y = y; }
    /// set the heading theta
    void setT(const float& t) { this->heading = t; }
};

class InsData {
public:
    double x;
    double y;
    double heading;
};

class GaussData {
public:
    uint index;
    double x;
    double y;
    double heading;
    double s;
    double z;
};

class SLPoint {
public:
    double s;
    double l;
};

class InterpolatedIndex {
public:
    InterpolatedIndex(int id, double offset) : id(id), offset(offset) {}
    int id = 0;
    double offset = 0.0;
};

}   // planning
}   // ADC


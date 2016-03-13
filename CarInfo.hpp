#ifndef CARTRACKER_TYPE_HPP_
#define CARTRACKER_TYPE_HPP_

#include <string>

namespace CarTracker{

struct CarBaseInfo {
    std::string type;
    std::string ID;
};

struct LocInfo {
    double  locX;
    double  locY;
    double  dir;
};

struct CarAllInfo : public CarBaseInfo, public LocInfo {
    CarAllInfo() {}
    CarAllInfo(const CarBaseInfo& _cbi) : CarBaseInfo(_cbi) {}
};

}

#endif

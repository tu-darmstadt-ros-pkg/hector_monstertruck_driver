#ifndef MONSTERTRUCK_DRIVER_MONSTERTRUCK_MODEL_H
#define MONSTERTRUCK_DRIVER_MONSTERTRUCK_MODEL_H

namespace Monstertruck {

struct MonstertruckModel {
    static double getWheelTrack() { return 0.30; }
    static double getWheelBase()  { return 0.32; }
    static double getBaseHeight() { return 0.0; }
};

} // namespace Monstertruck

#endif // MONSTERTRUCK_DRIVER_MONSTERTRUCK_MODEL_H

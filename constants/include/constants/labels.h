#ifndef LABELS
#define LABELS

namespace labels {
    const int INVALID_LABEL = -1;
    const int UNSEARCHED = 0;
    const int FLAT_WOOD = 1;
    const int SAND = 50;
    const int MAGNET = 60;
    const int NO_MAGNET = 98;
    const int GRAVEL = 99;
    const int OBSTACLE = 100; // Anything greater than OBSTACLE is not traversable 
    const int PIT = 101;
    const int FIRE = 102;
    const int NO_FIRE = 103;
    const int SMALL_HOUSE = 104;
    const int BIG_HOUSE = 105;
    const int OBJECT = 106;
    const int TALL_OBJECT = 107;
}

#endif // LABELS

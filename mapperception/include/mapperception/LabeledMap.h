#ifndef LABELED_MAP
#define LABELED_MAP

#include <vector>
#include "BaseMap.h"

enum Labels {
    FLAT_WOOD = 1,
    PIT = 2,
    GRAVEL = 3,
    SAND = 4,
    MAGNET = 5,
    FIRE = 6,
    NO_FIRE = 7,
    SMALL_HOUSE = 8,
    BIG_HOUSE = 9
};

class LabeledMap : public BaseMap {
public:
    LabeledMap() {
        _resolution = 1;
        _size = 6;
        _map.resize(_size);
        _map[0].resize(_size);
        for(int i = 0; i < _size; i++) {
            for(int j = 0; j < _size; j++) {
                _map[j][i] = 0;
            }
        }
    }

    void setValue(int x, int y, Labels label) { _map[y][x] = label; }
};

#endif // LABELED_MAP

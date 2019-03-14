#ifndef COST_MAP
#define COST_MAP

#include <vector>
#include "BaseMap.h"

class CostMap : public BaseMap {
public:
    CostMap() {
        _resolution = 10;
        _size = 60;
        _map.resize(_size);
        for(int i = 0; i < _size; i++) {
            _map[i].resize(_size);
            for(int j = 0; j < _size; j++) {
                _map[i][j] = 0;
            }
        }
    }

    void setValue(int x, int y, int value) { _map[y][x] = value; }
};

#endif // COST_MAP

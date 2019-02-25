#ifndef LABELED_MAP
#define LABELED_MAP

#include <vector>
#include "BaseMap.h"

class LabeledMap : public BaseMap {
public:
    LabeledMap() {
        _resolution = 10;
        _size = 60;
        for(int i = 0; i < _size; i++) {
            for(int j = 0; j < _size; j++) {
                _map[j][i] = 0;
            }
        }
    }
    std::vector< std::vector<int> > getMap() { return _map; }
    void setValue(int x, int y, int value) { _map[y][x] = value; }

private:
    std::vector< std::vector<int> > _map;
};

#endif // LABELED_MAP

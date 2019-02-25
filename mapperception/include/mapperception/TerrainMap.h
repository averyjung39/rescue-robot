#ifndef TERRAIN_MAP
#define TERRAIN_MAP

#include <vector>
#include "BaseMap.h"

enum Terrain {
    flat_wood = 1,
    water = 2,
    gravel = 3,
    sand = 4,
    sand_with_magnet = 5,
    flat_wood_with_fire = 6
};

class TerrainMap : public BaseMap {
public:
    TerrainMap() {
        _resolution = 1;
        _size = 6;
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

#endif // TERRAIN_MAP

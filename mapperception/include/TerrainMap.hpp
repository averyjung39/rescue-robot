enum Terrain {
    flat_wood = 1,
    water = 2,
    gravel = 3,
    sand = 4,
    sand_with_magnet = 5,
    flat_wood_with_fire = 6
};

Class TerrainMap : public Map {
public:
    int** getMap() { return _map };
    void setValue(int x, int y, int value) { _map[y][x] = value; }
private:
    int _size = 6;
    int _resolution = 1;
    int _map[6*_resolution][6*_resolution];
}

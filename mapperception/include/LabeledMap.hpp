Class LabeledMap : public Map {
public:
    int** getMap() { return _map };
    void setValue(int x, int y, int value) { _map[y][x] = value; }
private:
    int _size = 60;
    int _resolution = 10;
    int _map[6*_resolution][6*_resolution];
}

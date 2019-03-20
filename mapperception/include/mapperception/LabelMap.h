#ifndef LABELED_MAP
#define LABELED_MAP

#include <vector>

class LabelMap {
public:
    LabelMap() {
        _resolution = 1;
        _size = 6;
        _map.resize(_size);
        for(int i = 0; i < _size; i++) {
            _map[i].resize(_size);
            for(int j = 0; j < _size; j++) {
                _map[i][j] = 0;
            }
        }
    }

    void setLabel(int row, int col, int label) { _map[row][col] = label; }

    int getSize() { return _size; }
    int getResolution() { return _resolution; }

    int queryMap(int row, int col) { return _map[row][col]; }

    /**
     * @brief print the current label map
     */
    void print() const;

    std::vector< std::vector<int> > getMap() { return _map; }

private:
    int _size;
    int _resolution;
    std::vector< std::vector<int> > _map;
};

#endif // LABELED_MAP

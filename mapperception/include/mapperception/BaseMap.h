#ifndef BASE_MAP
#define BASE_MAP

class BaseMap {
public:
    int getSize() { return _size; }
    int getResolution() { return _resolution; }
    void print() const;

    std::vector< std::vector<int> > getMap() { return _map; }

protected:
    int _size;
    int _resolution;
    std::vector< std::vector<int> > _map;
};

#endif // BASE_MAP

#ifndef BASE_MAP
#define BASE_MAP

class BaseMap {
public:
    int getSize() { return _size; }
    int getResolution() { return _resolution; }
protected:
    int _size;
    int _resolution;
};

#endif // BASE_MAP

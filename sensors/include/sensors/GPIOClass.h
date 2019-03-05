#ifndef GPIOCLASS_H
#define GPIOCLASS_H

#include <string>
using namespace std;

class GPIOClass
{
    public:
        GPIOClass();
        GPIOClass(string x);

        bool export_gpio();
        bool unexport_gpio();
        bool setdir_gpio(string dir);
        bool write_gpio(string val);
        bool read_gpio(bool &val);
        string get_gpionum();
    private:
        string gpionum;
};

#endif // GPIOCLASS_H

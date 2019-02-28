#ifndef GPIOCLASS_H
#define GPIOCLASS_H

#include <string>
using namespace std;

class GPIOClass
{
    public:
        GPIOClass();
        GPIOClass(string x);

        int export_gpio();
        int unexport_gpio();
        int setdir_gpio(string dir);
        int write_gpio(string val);
        int read_gpio(string &val);
        string get_gpionum();
    private:
        string gpionum;
};

#endif // GPIOCLASS_H

#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include "sensors/GPIOClass.h"

using namespace std;

GPIOClass::GPIOClass()
{
    this->gpionum = "4";
}

GPIOClass::GPIOClass(string gnum)
{
    this->gpionum = gnum;
}

int GPIOClass::export_gpio()
{
    string export_str="/sys/class/gpio/export";
    ofstream exportgpio(export_str.c_str());
    if(exportgpio<0){
        cout << "OPERATION FAILED:  Unable to export GPIO" << this->gpionum << " ." << endl;
        return -1;
    }

    exportgpio << this->gpionum;
    exportgpio.close();
    return 0;
}

int GPIOClass::unexport_gpio()
{
    string unexport_str = "/sys/class/gpio/unexport";
    ofstream unexportgpio(unexport_str.c_str());
    if(unexportgpio < 0){
        cout << "OPERATION FAILED:  Unble to unexport GPIO" << this->gpionum << " ." << endl;
        return -1;
    }

    unexportgpio << this->gpionum;
    unexportgpio.close();
    return 0;
}

int GPIOClass::setdir_gpio(string dir)
{
    string setdir_str = "/sys/class/gpio/gpio"+this->gpionum+"/direction";
    ofstream setdirgpio(setdir_str.c_str());
    if(setdirgpio <  0){
        cout << "OPERATION FAILED: Unable to set direction of GPIO"<<this->gpionum<<" ."<<endl;
        return -1;
    }

    setdirgpio << dir;
    setdirgpio.close();
    return 0;
}

int GPIOClass::write_gpio(string val){
    string write_str = "/sys/class/gpio/gpio" + this->gpionum + "/value";
    ofstream writegpio(write_str.c_str());
    if(writegpio<0){
        cout << "OPERATION FAILED: Unable to set the value of the GPIO" << this->gpionum << " ." << endl;
        return -1;
    }

    writegpio << val;
    writegpio.close();
    return 0;
}

int GPIOClass::read_gpio(string& val){
    string read_str = "/sys/class/gpio/gpio"+this->gpionum+"/value";
    ifstream readgpio(read_str.c_str());
    if(readgpio < 0){
        cout << "OPERATION FAILED: Unable to get value of GPIO" << this->gpionum << " ." << endl;
        return -1;
    }

    readgpio >> val;
    if(val != "0")
        val = "1";
    else
        val = "0";

    readgpio.close();
    return 0;
}

string GPIOClass::get_gpionum(){
    return this->gpionum;
}

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

bool GPIOClass::export_gpio()
{
    string export_str="/sys/class/gpio/export";
    ofstream exportgpio(export_str.c_str());
    if(exportgpio<0){
        cout << "OPERATION FAILED:  Unable to export GPIO" << this->gpionum << " ." << endl;
        return false;
    }

    exportgpio << this->gpionum;
    exportgpio.close();
    return true;
}

bool GPIOClass::unexport_gpio()
{
    string unexport_str = "/sys/class/gpio/unexport";
    ofstream unexportgpio(unexport_str.c_str());
    if(unexportgpio < 0){
        cout << "OPERATION FAILED:  Unble to unexport GPIO" << this->gpionum << " ." << endl;
        return false;
    }

    unexportgpio << this->gpionum;
    unexportgpio.close();
    return true;
}

bool GPIOClass::setdir_gpio(string dir)
{
    string setdir_str = "/sys/class/gpio/gpio"+this->gpionum+"/direction";
    ofstream setdirgpio(setdir_str.c_str());
    if(setdirgpio <  0){
        cout << "OPERATION FAILED: Unable to set direction of GPIO"<<this->gpionum<<" ."<<endl;
        return false;
    }

    setdirgpio << dir;
    setdirgpio.close();
    return true;
}

bool GPIOClass::write_gpio(string val){
    string write_str = "/sys/class/gpio/gpio" + this->gpionum + "/value";
    ofstream writegpio(write_str.c_str());
    if(writegpio<0){
        cout << "OPERATION FAILED: Unable to set the value of the GPIO" << this->gpionum << " ." << endl;
        return false;
    }

    writegpio << val;
    writegpio.close();
    return true;
}

bool GPIOClass::read_gpio(bool& val){
    string read_str = "/sys/class/gpio/gpio"+this->gpionum+"/value";
    ifstream readgpio(read_str.c_str());
    if(readgpio < 0){
        cout << "OPERATION FAILED: Unable to get value of GPIO" << this->gpionum << " ." << endl;
        return false;
    }

    std::string string_val;
    readgpio >> string_val;
    val = string_val != "0";

    readgpio.close();
    return true;
}

string GPIOClass::get_gpionum(){
    return this->gpionum;
}

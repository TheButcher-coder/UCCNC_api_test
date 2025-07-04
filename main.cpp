#include <cstdio>
#include <cmath>
#include <windows.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <windows.h>
#include <string>
#include <chrono>
#include <thread>
#include "source/UC100.h"
#include "timed_spots.h"
#include "g_file.h"

// typedef some UC100 function pointers
typedef int (__stdcall *AddLinMoveFP)(double, double, double, double, double, double, double, double);
typedef int (*ListDevicesFP)(int*);
typedef int (*SetEstopSettingFP)(int, bool);
typedef int (*SetEstopStateFP)();
typedef int(*DeviceInfoFP)(int, int*, int*);
typedef int(*OpenFP)(int);
typedef int(*GetStatusFP)(Stat *SStat);
typedef int(*StopFP)(void);
typedef int(*SetMotionProgressStateFP)(bool);
typedef int(*SetAxisSettingFP)(AxisSetting*);
typedef int(*AddLinearMoveRelFP)(int, double, int, double, bool);
typedef int(*SetSpindleSettingFP)(SPSetting*);
typedef int (*GetAxisPositionFP)(double*, double*, double*, double*, double*, double*);
typedef int (*SetAxisPositionFP)(double, double, double, double, double, double);
typedef int (*HomeOnFP)(int, double, double, bool);
//SET SPINDEL SETTOING!!!!
//SetSpindleSetting(SPSetting *_SPSetting);


// get the functions we want and make them global variables
ifstream path("UCCNC_PATH.txt");
const char *uccn_path = "C:\\UCCNC\\API\\DLL\\UC100.dll";
//basic_istream<char> b = getline(path, uccn_path);
HINSTANCE hGetProcLib = LoadLibrary(uccn_path);
auto AddLinMove_ = (AddLinMoveFP)GetProcAddress(hGetProcLib, "AddLinearMoveRel");
auto listDevices_ = (ListDevicesFP)GetProcAddress(hGetProcLib, "ListDevices");
auto SetEstopSetting_ = (SetEstopSettingFP)GetProcAddress(hGetProcLib, "SetEstopSetting");
auto Open_ = (OpenFP)GetProcAddress(hGetProcLib, "Open");
auto GetStatus_ = (GetStatusFP)GetProcAddress(hGetProcLib, "GetStatus");
auto Stop_ = (StopFP)GetProcAddress(hGetProcLib, "Stop");
auto SetEstopState_ = (SetEstopStateFP)GetProcAddress(hGetProcLib, "SetEstopState");
auto SetMotionProgressState_ = (SetMotionProgressStateFP)GetProcAddress(hGetProcLib, "SetMotionProgressState");
auto SetAxisSetting_ = (SetAxisSettingFP)GetProcAddress(hGetProcLib, "SetAxisSetting");
auto AddLinearMoveRel_ = (AddLinearMoveRelFP)GetProcAddress(hGetProcLib, "AddLinearMoveRel");
auto SetSpindleSetting_ = (SetSpindleSettingFP)GetProcAddress(hGetProcLib, "SetSpindleSetting");
auto GetAxisPosition_ = (GetAxisPositionFP)GetProcAddress(hGetProcLib, "GetAxisPosition");
auto SetAxisPosition_ = (SetAxisPositionFP) GetProcAddress(hGetProcLib, "SetAxisPosition");
auto HomeOn_ = (HomeOnFP) GetProcAddress(hGetProcLib, "HomeOn");



int device_count;

void sleep(int millis) {
    this_thread::sleep_for(chrono::milliseconds(millis));

}

static bool open_device() {


    // call the DLL function
    device_count = 0;
    int r1 = listDevices_(&device_count);
    //std::cout << "R1: " << r1 << std::endl;
    int r2 = Open_(1);

    //if (r1 != 0 || r2 != 0)
     //   return false;

    Stop_();

    SPSetting s_set;
    s_set.Mode = 0;
    SetSpindleSetting_(&s_set);

    return true;
}

static bool set_axes() {
    AxisSetting x_axis, y_axis, z_axis, a_axis;

    x_axis.Axis = 0;
    x_axis.Enable = true;
    x_axis.StepPin = 3;
    x_axis.DirPin = 2;
    x_axis.MaxAccel = 400;
    x_axis.MaxVel = 4200;
    x_axis.StepPer = 533;
    x_axis.HomePin = 12;
    x_axis.LimitPPin = 0;
    x_axis.LimitNPin = 12;
    x_axis.SoftLimitP = 298;
    x_axis.SoftLimitN = 0;
    x_axis.EnablePin = 0;
    x_axis.CurrentHiLowPin = 0;

    y_axis.Axis = 1;
    y_axis.Enable = true;
    y_axis.StepPin = 5;
    y_axis.DirPin = 4;
    y_axis.MaxAccel = 400;
    y_axis.MaxVel = 4200;
    y_axis.StepPer = 533;
    y_axis.HomePin = 12;
    y_axis.LimitPPin = 0;
    y_axis.LimitNPin = 12;
    y_axis.SoftLimitP = 415;
    y_axis.SoftLimitN = 0;
    y_axis.EnablePin = 0;
    y_axis.CurrentHiLowPin = 0;

    z_axis.Axis = 2;
    z_axis.Enable = true;
    z_axis.StepPin = 7;
    z_axis.DirPin = 6;
    z_axis.MaxAccel = 400;
    z_axis.MaxVel = 4200;
    z_axis.StepPer = 533;
    z_axis.HomePin = 12;
    z_axis.LimitPPin = 0;
    z_axis.LimitNPin = 12;
    z_axis.SoftLimitP = -132;
    z_axis.SoftLimitN = 0;
    z_axis.EnablePin = 0;
    z_axis.CurrentHiLowPin = 0;

    a_axis.Axis = 3;
    a_axis.Enable = true;
    a_axis.StepPin = 9;
    a_axis.DirPin = 8;
    a_axis.MaxAccel = 200;
    a_axis.MaxVel = 3600;
    a_axis.StepPer = 480;
    a_axis.HomePin = 12;
    a_axis.LimitPPin = 0;
    a_axis.LimitNPin = 12;
    a_axis.SoftLimitP = -1000000;
    a_axis.SoftLimitN = 1000000;
    a_axis.EnablePin = 0;
    a_axis.CurrentHiLowPin = 0;

    int error_checkX = SetAxisSetting_(&x_axis);
    if (error_checkX != 0) {
        std::cout << "X: error!\nUC100 error-code: " << error_checkX << std::endl;
        return false;
    }

    Stop_();
    int error_checkY = SetAxisSetting_(&y_axis);
    if (error_checkY != 0) {
        std::cout << "Y: error!\nUC100 error-code: " << error_checkY << std::endl;
        return false;
    }

    Stop_();
    int error_checkZ = SetAxisSetting_(&z_axis);
    if (error_checkZ != 0) {
        std::cout << "Z: error!\nUC100 error-code: " << error_checkZ << std::endl;
        return false;
    }

    Stop_();
    int error_checkA = SetAxisSetting_(&a_axis);
    if (error_checkA != 0) {
        std::cout << "A: error!\nUC100 error-code: " << error_checkA << std::endl;
        return false;
    }

    return true;
}

void dip() {
    AddLinearMoveRel_(2, 4, 1, 250, false);
    AddLinearMoveRel_(2, 4, 1, 250, true);
}

void moveY(double amount, double rate) {
    AddLinearMoveRel_(1, amount, 1, rate, true);
}

static void print_controller_info() {
    Stat res_stat;

    GetStatus_(&res_stat);
    std::cout << "Device count: " << device_count << std::endl;
    std::cout << "estop: " << res_stat.Estop << std::endl;
    std::cout << "Idle: " << res_stat.Idle << std::endl;
}

void homeX() {
    HomeOn_(0, 10, 10, false);
}
void homeY() {
    HomeOn_(1, 10, 10, true);
}
void homeZ() {
    HomeOn_(2, -10, 10, false);
}

void home() {
    //HomeOn(int Axis, double SpeedUp, double SpeedDown, bool Dir)
    homeX();
    homeY();
    homeZ();
}

int print_current_time() {
    const auto p1 = std::chrono::system_clock::now();

    //std::cout << "seconds since epoch: "
    //          << std::chrono::duration_cast<std::chrono::seconds>(
    //                  p1.time_since_epoch()).count() << '\n';
    return std::chrono::duration_cast<std::chrono::milliseconds>(
            p1.time_since_epoch()).count();
}



void exec_gfile(g_file &in, double feed, double dwell, double rate) {
    SetMotionProgressState_(false);
    sleep(dwell*1000);
    timed_spots ts;
    point current;
    int numlin = in.get_size();
    int starting_time = print_current_time();

    current.x = current.y = current.z = 0;

    string paath = in.get_of() + "/" + to_string(print_current_time()) + ".txt";
    for(int i = 0; i < in.get_size(); i++) {
        point temp = in.get_koord(i)-current;

        cout << i << endl;
        temp.print();
        //int AddLinearMoveRel(int Axis,double Step,int StepCount,double Speed,bool Dir); //Adds a relative coordinate linear movement to the motion buffer.
        //if(in.get_koord(i).x == 0 & in.get_koord(i).y == 0 && in.get_koord(i).z == 0) feed *= 0.1;
        if(!isnan(temp.x)){
            bool dir = true;
            if(temp.x < 0) dir = !dir;
            cout << "Moving X: " << temp.x << endl;
            AddLinearMoveRel_(0, abs(temp.x), 1, feed, dir);
            //sleep(1000);
        }
        if(!isnan(temp.y)){
            bool dir = true;
            if(temp.y < 0) dir = !dir;
            cout << "Moving Y: " << temp.y << endl;
            AddLinearMoveRel_(1, abs(temp.y), 1, feed, !dir);
            //(1000);
        }
        if(!isnan(temp.z)){
            bool dir = true;
            if(temp.z < 0) dir = !dir;
            cout << "Moving Z: " << temp.z << endl;
            AddLinearMoveRel_(2, abs(temp.z), 1, feed, dir);
            //sleep(1000);
        }
        current = current + temp;
        if(i != numlin-1) dip();
        ts.add_spot(print_current_time()-starting_time);
        sleep(rate*1000);

        //AddLinMove_(temp.x, temp.y, temp.z, 0, 0, 0, feed, 0);
    }
    //string paath = in.get_of() + "/" + to_string(print_current_time()) + ".txt";
    cout << "PATH: " << paath << endl;
    ts.write_to_file(paath);
}

void listen_usb_gfile(g_file &in, double feed, double dwell, double rate, int coom) {
    string coomport = R"(\\.\COM)" + to_string(coom);
    HANDLE hSerial = CreateFile(coomport.c_str(),
                                GENERIC_READ | GENERIC_WRITE,
                                0,
                                0,
                                OPEN_EXISTING,
                                FILE_ATTRIBUTE_NORMAL,
                                0);

    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cerr << "Fehler beim Öffnen des seriellen Ports." << std::endl;
        return;
    }

    // Serielle Verbindung konfigurieren
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "Fehler beim Abrufen des COM-Status." << std::endl;
        CloseHandle(hSerial);
        return;
    }

    dcbSerialParams.BaudRate = CBR_9600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "Fehler beim Konfigurieren des COM-Ports." << std::endl;
        CloseHandle(hSerial);
        return;
    }

    // Timeout-Einstellungen
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout         = 50;
    timeouts.ReadTotalTimeoutConstant    = 50;
    timeouts.ReadTotalTimeoutMultiplier  = 10;

    if (!SetCommTimeouts(hSerial, &timeouts)) {
        std::cerr << "Fehler beim Setzen der COM-Timeouts." << std::endl;
        CloseHandle(hSerial);
        return;
    }

    // Daten vom seriellen Port lesen und in der Konsole ausgeben
    char szBuff[1024];
    std::string port_message;
    DWORD dwBytesRead = 0;

    bool didrun = false;
    while (!didrun) {
        if (ReadFile(hSerial, szBuff, sizeof(szBuff) - 1, &dwBytesRead, nullptr)) {
            if (dwBytesRead > 0) {
                szBuff[dwBytesRead] = '\0'; // Null-terminierte Zeichenkette
                std::cout << "A" << szBuff << std::endl;

                if (szBuff[0] == '1') {
                    sleep(dwell*1000);
                    exec_gfile(in, feed, dwell, rate);
                    didrun = true;
                }
            }
        }
        else {
            std::cerr << "Couldn't read Com Port" << std::endl;
            return;
        }
    }
    sleep(20);
}

int main(int argc, char* argv[]) {
    /*
     * ***argv[1]   spot rate
     * ***argv[2]   Dwell
     * ***argv[3]   UCCNC Path
     * ***argv[4]   Gfile path
     * ***argv[5]   dest Path of stuff
     * ***argv[6]   COM port
     */
    // load the UC100 DLL
    if (!open_device()) {
        std::cerr << "Device is opened with another Software, please deactivate and try again!" << std::endl;
        return 1;
    }
    // set your axes
    if (!set_axes())
        return 1;


    if(argc == 1) {
        //listen_usb();     //WORKS!!!!!

        //  ***TEST OF GCODE PARSER ON 20*25SNAKE***
        g_file test("../gcodes/snaek.txt", "../testing");
        cout << test.get_of() << endl;
        test.parse_file();
        test.print_koords();
        listen_usb_gfile(test, 10, 0, 1, 15);
        //exec_gfile(test, 50, 0, 1);
        print_controller_info();
    }
    else {
        g_file file(argv[4], argv[5]);
        file.parse_file();
        file.print_koords();
        listen_usb_gfile(file, 10, stod(argv[2]), stod(argv[1]), stoi(argv[6]));
        //exec_gfile(file, 50, stod(argv[2]), stod(argv[1]));
        print_controller_info();
        //init_cmd(R"(C:\UCCNC\API\DLL\UC100.dll)");
        //do stuff
    }
    return 0;
}

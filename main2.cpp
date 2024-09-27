#include <cstdio>
#include <windows.h>
#include <iostream>
#include <vector>
#include <iostream>
#include <windows.h>
#include <string>
#include <chrono>
#include <thread>
#include "source/UC100.h"
#include "timed_spots.h"
#include "g_file.h"

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

using namespace std;


void sleep(int millis) {
    this_thread::sleep_for(chrono::milliseconds(millis));

}

int get_current_time() {
    const auto p1 = std::chrono::system_clock::now();

    //std::cout << "seconds since epoch: "
    //          << std::chrono::duration_cast<std::chrono::seconds>(
    //                  p1.time_since_epoch()).count() << '\n';
    return std::chrono::duration_cast<std::chrono::seconds>(
            p1.time_since_epoch()).count();
}

int main(int argc, char* argv[]) {
    /*
     * ***argv[1]   spot rate
     * ***argv[2]   Dwell
     * ***argv[3]   UCCNC Path
     * ***argv[4]   Gfile path
     * ***argv[5]   dest Path of stuff
     */
    // load the UC100 DLL
    const char *uccnc_path;
    if(argc == 1) uccnc_path = R"(C:\UCCNC\API\DLL\UC100.dll)";
    else uccnc_path = argv[3];

    HINSTANCE hGetProcLib = LoadLibrary(uccnc_path);
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


    //Open the device
    int r = Open_(1);
    sleep(1000);
    if(r == 0) {
        cerr << "Cant open device. It may be opened with another Software" << endl;
    }
    else {
        Stop_();

        SPSetting s_set;
        s_set.Mode = 0;
        SetSpindleSetting_(&s_set);
    }

    //Set the Axes
    AxisSetting x_axis;
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
    AxisSetting y_axis;
    y_axis.Axis = 0;
    y_axis.Enable = true;
    y_axis.StepPin = 3;
    y_axis.DirPin = 2;
    y_axis.MaxAccel = 400;
    y_axis.MaxVel = 4200;
    y_axis.StepPer = 533;
    y_axis.HomePin = 12;
    y_axis.LimitPPin = 0;
    y_axis.LimitNPin = 12;
    y_axis.SoftLimitP = 298;
    y_axis.SoftLimitN = 0;
    y_axis.EnablePin = 0;
    y_axis.CurrentHiLowPin = 0;
    AxisSetting z_axis;
    z_axis.Axis = 0;
    z_axis.Enable = true;
    z_axis.StepPin = 3;
    z_axis.DirPin = 2;
    z_axis.MaxAccel = 400;
    z_axis.MaxVel = 4200;
    z_axis.StepPer = 533;
    z_axis.HomePin = 12;
    z_axis.LimitPPin = 0;
    z_axis.LimitNPin = 12;
    z_axis.SoftLimitP = 298;
    z_axis.SoftLimitN = 0;
    z_axis.EnablePin = 0;
    z_axis.CurrentHiLowPin = 0;


    int error_checkX = SetAxisSetting_(&x_axis);
    if (error_checkX != 0) {
        std::cerr << "X: error!\nUC100 error-code: " << error_checkX << std::endl;
        return 1;
    }

    Stop_();
    int error_checkY = SetAxisSetting_(&y_axis);
    if (error_checkY != 0) {
        std::cerr << "Y: error!\nUC100 error-code: " << error_checkY << std::endl;
        return 1;
    }

    Stop_();
    int error_checkZ = SetAxisSetting_(&z_axis);
    if (error_checkZ != 0) {
        std::cerr << "Z: error!\nUC100 error-code: " << error_checkZ << std::endl;
        return 1;
    }

    if(argc == 1) {
        //init_cmd(R"(C:\UCCNC\API\DLL\UC100.dll)");
        // open the UC100

        //  ***TEST OF GCODE PARSER ON 20*25SNAKE***
        g_file test("../gcodes/snaek.txt", "testing/poop.txt");
        test.parse_file();
        test.print_koords();
        char **end;
        //exec_gfile(test, 50, 0, 1, "testing/AHH.txt");

    }
    else {
        //do stuff
    }
    return 0;
}
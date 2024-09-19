#include <cstdio>
#include <windows.h>
#include <iostream>
#include <vector>
#include <iostream>
#include <windows.h>
#include <string>
#include <chrono>
#include "source/UC100.h"

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
HINSTANCE hGetProcLib = LoadLibrary(R"(C:\UCCNC\API\DLL\UC100.dll)");
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

static bool open_device() {


    // call the DLL function
    device_count = 0;
    int r1 = listDevices_(&device_count);
    std::cout << "R1: " << r1 << std::endl;
    int r2 = Open_(1);

    if (r1 != 0 || r2 != 0)
        return false;

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
/*
void move_spotter() {
    // Disable motion progress state
    SetMotionProgressState_(false);

    // Initialize move ID
    int move_id = 0;

    // Define feedrate and Z-axis heights
    double feedrate = 500.0;        // Initial feedrate
    double z_safe_height = 25.0;    // Safe height for Z axis
    double z_move_height = 4.0;     // Height for move position
    double z_work_height = 1.0;     // Working height

    // Get the current axis positions
    double x_pos, y_pos, z_pos, a_pos, b_pos, c_pos;
    GetAxisPosition_(&x_pos, &y_pos, &z_pos, &a_pos, &b_pos, &c_pos);

    std::vector<double> x_positions = {28.595, 37.190, 45.785, 54.380, 62.975, 71.570, 80.165, 88.760, 97.355, 105.950, 114.545, 123.140, 131.735, 140.330, 148.925, 157.520, 166.115, 174.710, 183.305};
    std::vector<double> y_positions = {-22.017, -32.134, -42.251, -52.368, -62.485, -72.602, -82.719, -92.836, -102.953, -113.070, -123.187, -133.304, -143.421, -153.538, -163.655, -173.772, -183.889, -194.006, -204.123, -214.240, -224.357, -234.474, -244.591, -254.708};

    double xmove_rel = x_positions[1] - x_positions[0];
    double ymove_rel = y_positions[1] - y_positions[0];

    double xpos=0, ypos=0;
    for(int i = 0; i < y_positions.size(); i++) {
        dip();
        for(int j = 0; j < x_positions.size(); j++) {
            //Reset condition
            if (ReadFile(hSerial, szBuff, sizeof(szBuff) - 1, &dwBytesRead, nullptr)) {
                if (dwBytesRead > 0) {
                    szBuff[dwBytesRead] = '\0'; // Null-terminierte Zeichenkette
                    std::cout << szBuff << std::endl;

                    if(szBuff[2] == '1') {
                        std::cout << "POOP" << std::endl;
                        xpos = xmove_rel*j;
                        ypos = -ymove_rel*i;
                        break;
                    }
                }
            } else {
                std::cerr << "Fehler beim Lesen vom COM-Port. Im bewegen dings" << std::endl;
                break;
            }

            AddLinearMoveRel_(0, xmove_rel, 1, feedrate, i%2==0);
            dip();
        }
        moveY(-ymove_rel, feedrate);
    }
    AddLinearMoveRel_(0, -xpos, 1, feedrate, false);
    moveY(ypos, feedrate);
}
 */

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

void listen_usb() {
    HANDLE hSerial = CreateFile(R"(\\.\COM15)",
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


    while (true) {
        double x_pos, y_pos, z_pos, a_pos, b_pos, c_pos;
        bool stopsignal=false;
        if (ReadFile(hSerial, szBuff, sizeof(szBuff) - 1, &dwBytesRead, nullptr)) {
            if (dwBytesRead > 0) {
                szBuff[dwBytesRead] = '\0'; // Null-terminierte Zeichenkette
                std::cout <<"A" << szBuff << std::endl;

                if(szBuff[0] == '1') {
                    std::cout << "POOP" << std::endl;

                    //move spotter
// Disable motion progress state
                    SetMotionProgressState_(false);

                    // Initialize move ID
                    int move_id = 0;

                    // Define feedrate and Z-axis heights
                    double feedrate = 500.0;        // Initial feedrate
                    double z_safe_height = 25.0;    // Safe height for Z axis
                    double z_move_height = 4.0;     // Height for move position
                    double z_work_height = 1.0;     // Working height


                    // Get the current axis positions

                    GetAxisPosition_(&x_pos, &y_pos, &z_pos, &a_pos, &b_pos, &c_pos);

                    std::vector<double> x_positions = {28.595, 37.190, 45.785, 54.380, 62.975, 71.570, 80.165, 88.760, 97.355, 105.950, 114.545, 123.140, 131.735, 140.330, 148.925, 157.520, 166.115, 174.710, 183.305};
                    std::vector<double> y_positions = {-22.017, -32.134, -42.251, -52.368, -62.485, -72.602, -82.719, -92.836, -102.953, -113.070, -123.187, -133.304, -143.421, -153.538, -163.655, -173.772, -183.889, -194.006, -204.123, -214.240, -224.357, -234.474, -244.591, -254.708};

                    double xmove_rel = x_positions[1] - x_positions[0];
                    double ymove_rel = y_positions[1] - y_positions[0];

                    SetAxisPosition_(0, 0, 0, 0, 0, 0);
                    for(int i = 0; i < y_positions.size(); i++) {
                        dip();
                        for(int j = 0; j < x_positions.size(); j++) {
                            AddLinearMoveRel_(0, xmove_rel, 1, feedrate, i%2==0);
                            dip();

                            //Reset condition
                            if (ReadFile(hSerial, szBuff, sizeof(szBuff) - 1, &dwBytesRead, nullptr)) {
                                if (dwBytesRead > 0) {
                                    szBuff[dwBytesRead] = '\0'; // Null-terminierte Zeichenkette
                                    std::cout << szBuff << std::endl;

                                    if(szBuff[2] == '1') {
                                        stopsignal = true;
                                        goto start;
                                    }
                                }
                            } else {
                                std::cerr << "Fehler beim Lesen vom COM-Port. Im bewegen dings" << std::endl;
                                break;
                            }


                        }
                        moveY(-ymove_rel, feedrate);
                    }


                }
                //Move back to start
                start:
                //std::cout << "X: " << xpos << " Y: " << ypos << std::endl

                if(stopsignal) {
                    GetAxisPosition_(&x_pos, &y_pos, &z_pos, &a_pos, &b_pos, &c_pos);
                    std::cout << "currently at: " << std::endl << "X: " << x_pos << " Y: " << y_pos << " Z: " << z_pos
                              << std::endl;
                    AddLinearMoveRel_(0, abs(x_pos), 1, 10, false);
                    AddLinearMoveRel_(1, abs(y_pos), 1, 10, false);
                    //AddLinearMoveRel_(2, abs(z_pos), 1, 10, false);
                    stopsignal = false;
                }
            }
        } else {
            std::cerr << "Fehler beim Lesen vom COM-Port." << std::endl;
            break;
        }
    }

    // Serielle Verbindung schließen
    CloseHandle(hSerial);
}

int main() {// load the UC100 DLL
    HINSTANCE hGetProcLib = LoadLibrary(R"(C:\UCCNC\API\DLL\UC100.dll)");
    if (!hGetProcLib) {
        printf("could not load the dynamic library\n");
        return 1;
    }

    // open the UC100
    if (!open_device()) {
        std::cerr << "Device is opened with another Software, please deactivate and try again!" << std::endl;
        return 1;
    }

    // set your axes
    if (!set_axes())
        return 1;

    //Home
    listen_usb();     //WORKS!!!!!
    // print some info
    print_controller_info();

    return 0;
}

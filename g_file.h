//
// Created by Spielen on 20.09.2024.
//

#ifndef GESAMT_G_FILE_H
#define GESAMT_G_FILE_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include "timed_spots.h"
#include "gcode_parser/gcode_program.h"
#include "gcode_parser/parser.h"


using namespace std;

struct point {
    double x, y, z;

    point(){
        x = y = z = nan("");
    }
    point(double a, double b, double c) {
        x=a;
        y=b;
        z=c;
    }
    void print() {
        cout << "X" << x << " Y" << y << " Z" <<  z << endl;
    }
    void print_clean(ofstream &of) {
        //without XYZ, just numbers
        // ALWAYS in order XYZ
        of << x << y << z << endl;
    }
    void operator=(point p) {
        x = p.x;
        y = p.y;
        z = p.z;
    }
    point operator+(point p) {
        point temp;
        if(p.x == nan("")) temp.x=x;
        else temp.x=x+p.x;
        if(p.y == nan("")) temp.y=y;
        else temp.y=y+p.y;
        if(p.z == nan("")) temp.z=z;
        else temp.z=z+p.z;
        return temp;
    }
    point operator-(point p) {
        point temp;
        if(p.x == nan("")) temp.x=x;
        else temp.x=x-p.x;
        if(p.y == nan("")) temp.y=y;
        else temp.y=y-p.y;
        if(p.z == nan("")) temp.z=z;
        else temp.z=z-p.z;
        return temp;
    }
};
class g_file {
private:
    string outfile, infile;
    vector<point> koords_abs;   //Absolute koordinates that the machine runs through
public:
    g_file(string inp, string out);
    g_file();

    vector<point> get_koord();
    void set_koord(point &in, int line);
    point get_koord(int i);
    int get_size();
    void append_koord(point &in);
    void parse_file();
    void print_koords();
    void set_outfile(string of);
    void print_to_file();
    void print_tf_ts(timed_spots ts);   //prints with timestamps

    ~g_file()=default;
};


#endif //GESAMT_G_FILE_H

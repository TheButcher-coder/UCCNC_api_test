//
// Created by Spielen on 20.09.2024.
//

#ifndef GESAMT_G_FILE_H
#define GESAMT_G_FILE_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>


using namespace std;

struct point {
    int x, y, z;

    point(){
        x = y = z = 0;
    }
    point(int a, int b, int c) {
        x=a;
        y=b;
        z=c;
    }
    void operator=(point p) {
        x = p.x;
        y = p.y;
        z = p.z;
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
    void append_koord(point &in);
    void parse_file();

    ~g_file()=default;
};


#endif //GESAMT_G_FILE_H

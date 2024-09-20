//
// Created by Spielen on 20.09.2024.
//

#include "g_file.h"

using namespace gpr;
using namespace std;

g_file::g_file(string inp, string out) {
    outfile = out;
    infile = inp;
}

g_file::g_file() {
    cout << "datei Büdde! OUTFILE: " << endl;
    cin >> outfile;
    cout << "INFILE: " << endl;
    cin >> infile;
}

vector<point> g_file::get_koord() {
    return koords_abs;
}

void g_file::append_koord(point &in) {
    koords_abs.push_back(in);
}

void g_file::set_koord(point &in, int line) {
    if(line >= koords_abs.size()) return;
    koords_abs[line] = in;
}

void g_file::parse_file() {
    gcode_program p = parse_gcode(infile);
    cout << p << endl;
}
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
    string file = infile;

    ifstream t(file);
    string file_contents((std::istreambuf_iterator<char>(t)),
                              std::istreambuf_iterator<char>());

    gcode_program p = parse_gcode(file_contents);
    cout << "Num of lines: " << p.num_blocks() << endl;

    for(int i = 0; i < p.num_blocks(); i++) {
        auto cblock = p.get_block(i);
        int chunksize = cblock.size();

        if(cblock.get_chunk(0).get_word() == 'G' && cblock.get_chunk(0).get_address().int_value() == 0) {
            point temp;
            for(int j = 1; j < chunksize; j++) {
                char coord = cblock.get_chunk(j).get_word();
                double num = cblock.get_chunk(j).get_address().double_value();

                switch(coord) {
                    case 'X':
                        temp.x = num;
                        break;
                    case 'x':
                        temp.x = num;
                        break;
                    case 'Y':
                        temp.y = num;
                        break;
                    case 'y':
                        temp.y = num;
                        break;
                    case 'Z':
                        temp.z = num;
                        break;
                    case 'z':
                        temp.z = num;
                        break;
                    default:
                        break;
                }
                append_koord(temp);
            }
        }
        //cblock.print(cout);
    }
    cout << "Parsing done!" << endl;
    //cout << p << endl;
}

void g_file::print_koords() {
    for(auto p : koords_abs) {
        p.print();
    }
}

point g_file::get_koord(int i) {
    if(i >= koords_abs.size()) return {-1, -1, -1};
    return koords_abs[i];
}

int g_file::get_size() {
    return koords_abs.size();
}
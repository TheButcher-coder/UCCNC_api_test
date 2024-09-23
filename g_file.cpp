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

    for (int i = 0; i < p.num_blocks(); i++) {
        auto cblock = p.get_block(i);
        int chunksize = cblock.size();
        point temp;  // Assuming `point` has x, y, z members

        for (int j = 0; j < chunksize; j++) {
            auto current_chunk = cblock.get_chunk(j);
            if (current_chunk.tp() == CHUNK_TYPE_WORD_ADDRESS) {
                char coord = current_chunk.get_word();
                addr current_address = current_chunk.get_address();

                // Check the type of the address and handle accordingly
                if (current_address.tp() == ADDRESS_TYPE_DOUBLE) {
                    double num = current_address.double_value();  // It's a double value
                    switch (coord) {
                        case 'X': case 'x':
                            temp.x = num;
                            break;
                        case 'Y': case 'y':
                            temp.y = num;
                            break;
                        case 'Z': case 'z':
                            temp.z = num;
                            break;
                    }
                } else if (current_address.tp() == ADDRESS_TYPE_INTEGER) {
                    int num = current_address.int_value();  // It's an integer value
                    switch (coord) {
                        case 'X': case 'x':
                            temp.x = static_cast<double>(num);  // Convert int to double if necessary
                            break;
                        case 'Y': case 'y':
                            temp.y = static_cast<double>(num);
                            break;
                        case 'Z': case 'z':
                            temp.z = static_cast<double>(num);
                            break;
                    }
                }
            }
        }
        append_koord(temp);  // Add the parsed point to the coordinates list
    }

    cout << "Parsing done!" << endl;
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
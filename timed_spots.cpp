//
// Created by Spielen on 20.09.2024.
//
#include <fstream>
#include "timed_spots.h"

timed_spots::timed_spots() {}

void timed_spots::add_spot(int t) {
    ts.push_back(t);
}

int timed_spots::get_spot(int num) {
    if(num >= ts.size()) return -1;     //error
    return ts[num];
}

void timed_spots::write_to_file(std::string dest) {
    ofstream of;
    of.open(dest);

    for(auto num: ts) {
        of << num << " ";
    }
    of.close();
}

void timed_spots::del() {
    ts.clear();
}
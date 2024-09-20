//
// Created by Spielen on 20.09.2024.
//

#ifndef GESAMT_TIMED_SPOTS_H
#define GESAMT_TIMED_SPOTS_H

#include <vector>
#include <string>

using namespace std;

class timed_spots {
private:
    vector<int> ts;     //timestamps
public:
    timed_spots();
    void add_spot(int t);
    int get_spot(int num);
    void write_to_file(string dest);
    void del();
    ~timed_spots()=default;
};


#endif //GESAMT_TIMED_SPOTS_H

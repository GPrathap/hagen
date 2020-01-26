// Copyright (C) 2019  Geesara Kulathunga, R. Fedorenko, University of Innopolis, Russia
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef GROUND_REMOVAL_UNION_FIND_H_
#define GROUND_REMOVAL_UNION_FIND_H_

#include <opencv2/opencv.hpp>
// #include <opencv2/core.hpp>
#include <algorithm>
#include <iostream>
#include <map>
#include <cassert>
#include <tuple>
#include <queue>


namespace kamaz {
    namespace hagen{

    struct Pixel{
        // int x;
        // int y;
        int index;
        float value;
    };

class UnionFindDS{
 public:
    UnionFindDS() = default;
    ~UnionFindDS() = default;
    
    void add(Pixel object, int weight, int index);
    bool is_contains(int);
    Pixel get_items(Pixel object);
    void union_f(std::vector<Pixel> objects);
    bool is_equals_tuple(Pixel t1, Pixel t2);
    void print_tuple(Pixel object);
    void print_map_parent();
    void print_map();
    void print_vector( std::vector<Pixel> path);
    void set_max_row(int max);

   private: 
    std::map<int, int> weights;
    std::map<int, Pixel> parent;
    int length_of_row;
};
    }
}

#endif 
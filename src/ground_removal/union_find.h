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
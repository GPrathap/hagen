#include "union_find.h"

namespace kamaz {
    namespace hagen {
        
    void UnionFindDS::add(Pixel object, int weight, int index){
            if(parent.find(index) != parent.end()){
                std::cout<< "Same object already exists" <<  object.index%length_of_row << " , "<< object.index/length_of_row << "we "<< weight <<std::endl;
            }else{
                parent[index] = object;
                weights[index] = weight;
            }
    }

    bool UnionFindDS::is_contains(int object){
        if(parent.find(object) == parent.end()){
            return false;
        }else{
            return true;
        }
    }

    void UnionFindDS::set_max_row(int max){
        length_of_row = max;
    }

    void UnionFindDS::print_tuple(Pixel object){
        std::cout<< "( " << object.index%length_of_row << " , " << object.index/length_of_row << ")" << std::endl;
    }

    void UnionFindDS::print_map(){
        std::cout<<"-----print map--------"<<std::endl;
        for(auto const& x : weights){
                    std::cout<< "( " << x.first << ") -> " << x.second << std::endl;
        }
        std::cout<<"-----end map--------"<<std::endl;
    }

    void UnionFindDS::print_vector( std::vector<Pixel> path){
        for(auto const& x : path){
               print_tuple(x); 
        }
    }

    void UnionFindDS::print_map_parent(){
        std::cout<<"-----print parent map--------"<<std::endl;
        // for(auto const& x : parent){
        //             std::cout<< "( " << std::get<0>(x.first) << " , " << std::get<1>(x.first) << ") -> " << "( " << std::get<0>(x.second) << " , " << std::get<1>(x.second)  << std::endl;
                   
        // }
        std::cout<<"-----end parent map--------"<<std::endl;
    }

    // Find and return the name of the set containing the object
    Pixel UnionFindDS::get_items(Pixel object){
        std::vector<Pixel> path;
        path.push_back(object);
        int index_object = object.index;
        auto root = parent[index_object];
        // find path of objects leading to the root
        while(!is_equals_tuple(root, path.back())) {
            path.push_back(root);
            root = parent[index_object];
        }
        // compress the path and return
        for(auto const& ancestor: path){
            int index_ancestor = ancestor.index;
            parent[index_ancestor] = root;
        }
        return root;
    }

    bool UnionFindDS::is_equals_tuple(Pixel t1, Pixel t2){
        if(t1.index == t2.index){
            return true;
        }
        return false;
    } 

    // Find the sets containing the objects and merge them all
    void UnionFindDS::union_f(std::vector<Pixel> objects){
        std::vector<Pixel> roots;
        for(auto const& x: objects){
                roots.push_back(get_items(x));
        }
        int max_item = -500000;
        Pixel heaviest;
        for(auto const& r: roots){
            Pixel test_ = {r.index, r.value};
            int index_test_ = test_.index;
            int _weight = weights[index_test_];
            if(_weight > max_item){
                max_item = _weight;
                heaviest = r;
            }
        }
        for(auto const& r: roots){
            if(!(r.index == heaviest.index)){
                int index_test_ = r.index;
                parent[index_test_] = heaviest;
            }
        }
    }
}
}
#include "quadtree.h"

namespace kamaz {
namespace hagen {

    QuadTree::QuadTree(std::vector<Point> data, float width, float height, float angle_threshold, const cv::Mat& angle_image_ ,const cv::Mat& depth_image_ )
        :width(width), height(height), angle_threshold(angle_threshold), rect(0, 0, width, height), root(data, rect, 0){
            if(data.size()>0){
                _split();
            }
            angle_image = angle_image_;
            depth_image = depth_image_;
    }

    void QuadTree::_split(){
        std::deque<Node*> node_list;
        node_list.push_back(&root);
        while (!node_list.empty()){
            Node* node = node_list.front();
            node_list.pop_front();
            if(node->get_val_size() <= 1){
                continue;
            }
            if(node->leaf()){
                auto rects = node->bounds_split();
                Node nw(node, rects[0], node->get_level()+1);
                Node ne(node, rects[1], node->get_level()+1);
                Node sw(node, rects[2], node->get_level()+1);
                Node se(node, rects[3], node->get_level()+1);
                node->sons.push_back(nw);
                node->sons.push_back(ne);
                node->sons.push_back(sw);
                node->sons.push_back(se);
            }
            for(Point val : node->get_val()){
                for(int son_name : node->get_sons()){
                    // std::cout<< "son boundary" << node->sons[son_name].bounds.debug_string () << std::endl;
                    if(node->sons[son_name].bounds.contains(val)){
                        // std::cout<< "value:" << val.x << "," << val.y << std::endl;
                        node->sons[son_name].val.push_back(val);
                        break;
                    }  
                }
            }
            node->val.clear();
            for(int son_name : node->get_sons()){
                node_list.push_back(&(node->sons[son_name]));
            }
            // std::cout<< "size of node list: "<< node_list.size() << std::endl;
        }
    }

    
    void QuadTree::in_order_traversal(Node root, cv::Mat& quadtree_labed_image){
        std::stack<Node> s1;
        std::stack<Node> s2;
        s1.push(root);
        // std::cout<< "=============start======="<< std::endl;
        while(s1.size()>0){
            auto node = s1.top();
            s1.pop();
            s2.push(node);
            if(node.sons.size()>0){
                for(int son_name : node.get_sons()){
                    s1.push(node.sons[son_name]);
                }
            }
        }
        std::cout<< "s2 size: "<< s2.size()<<std::endl;
        int ggg = 0;
        int in_ggg = 0;
        if(s2.size()<2){
            return;
        }
        while(s2.size()){
            auto node = s2.top();
            s2.pop();
            if(node.leaf() && node.val.size()==1){
                ggg++;
                Point point =  node.val[0];
                auto current_coord = Point(point.x, point.y);
                // label_3rd_part.LabelOneComponent(1, current_coord);
                // quadtree_labed_image.at<float>(point.x, point.y) = 1.0;
                // for(int index=0; index<4; ++index){
                //     int neighbor_x = point.x + x_direc[index];
                //     int neighbor_y = point.y + y_direc[index];
                //     if (neighbor_x < 0 || neighbor_x >= angle_image.rows) {
                //         continue;
                //     }
                //     if (neighbor_y < 0 || neighbor_y >= angle_image.cols) {
                //         continue;
                //     }
                //     if (quadtree_labed_image.at<float>(neighbor_x, neighbor_y) > 0) {
                //         continue;
                //     }
                //     auto diff = std::fabs(point.angle-angle_image.at<float>(neighbor_x, neighbor_y));
                //     if(diff< angle_threshold){
                //                     //  std::cout<< "======>>>" << std::endl;
                //         // if(depth_image.at<float>(neighbor_x, neighbor_y)>0.0){
                //             quadtree_labed_image.at<float>(neighbor_x, neighbor_y) = 1.0;
                //             in_ggg++;
                //         // }
                //     }
                // }

                // std::cout<< "=======endnode==========" << std::endl;
                // std::cout<<"x: " << point.x << " y: " 
                // << point.y << " depth: "<< point.depth << " angle: "<< point.angle<< std::endl;
                // Node* father = node.father;

                // for(int son_name : father->get_sons()){
                //     auto son_val = father->sons[son_name].val;
                //     if(father->sons[son_name].val.size()>0){
                //         for(Point son_point: son_val){
                //             if(!(point-son_point) == 0){
                // // std::cout<<"x: " << son_point.x << " y: " 
                // // << son_point.y << " depth: "<< son_point.depth << " angle: "<< son_point.angle << " level:" <<father->sons[son_name].level << std::endl;
                //                 // std::cout<< "======>>>" << std::fabs(son_point.angle-point.angle) << std::endl;
                //                 // if(std::fabs(son_point.angle-point.angle)< angle_threshold){
                //                     //  std::cout<< "======>>>" << std::endl;
                //                     // quadtree_labed_image.at<float>(son_point.x, son_point.y) = 1.0;
                //                     in_ggg++;
                //                 // }
                //             }
                //         }
                            
                //     }
                // }
            }
        }
        std::cout<< "=========================gggg==============="<< ggg << std::endl;
        std::cout<< "=========================gggg==============="<< in_ggg << std::endl;
    }
}
}
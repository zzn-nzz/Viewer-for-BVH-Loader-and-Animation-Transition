#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Engine/loader.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Final_Project/BVH.h"
#include <fstream>
#include <sstream>
#include <iostream>

namespace VCX::Labs::Animation {
    void BVH::load_from_file(const std::string bvh_file) {
        root_joints.clear(); frames = 0; frame_time = 0;
        now_frame_idx = 0; last_time = 0; num_all_channels = 0;
        channels_data.clear();
        std::ifstream file(bvh_file);

        if(!file.is_open()) {
            std::cerr << "Failed to open bvh file " << bvh_file << std::endl;
            return;
        }

        std::string line;
        Joint *now_root;
        while(std::getline(file, line)) {
            std::istringstream iss(line);
            std::string word;
            iss >> word;
            if(word == "ROOT") {

                now_root = load_joint(file, nullptr);
                iss >> now_root->name;
                root_joints.push_back(now_root);
                
            } else if (word == "JOINT") {
                
                Joint *new_now_joint = load_joint(file, now_root);
                iss >> new_now_joint->name;

            } else if(word == "MOTION") {
                
                load_motion(file);

            } else continue;
        }
        file.close();
        //check_loading();
    }
    
    Joint* BVH::load_joint(std::ifstream &file, Joint *father_joint) {
        Joint *now_joint = new Joint();
        now_joint->parent = father_joint;
        if(father_joint) father_joint->sons.push_back(now_joint);

        std::string line;
        while(std::getline(file, line)) {
            std::istringstream iss(line);
            std::string word;
            iss >> word;
            if(word == "OFFSET") {
                float x, y, z; iss >> x >> y >> z;
                now_joint->offset = 1.0f * glm::vec3(x, y, z);
            } else if(word == "CHANNELS") {
                get_channels(iss, now_joint);
            } else if(word == "JOINT") {
                Joint *new_now_joint = load_joint(file, now_joint);
                iss >> new_now_joint->name;
            } else if(word == "End") {
                Joint *end_joint = new Joint();
                now_joint->sons.push_back(end_joint);
                end_joint->parent = now_joint;
                end_joint->name = "End Joint";
                std::getline(file, line);
                std::getline(file, line);
                std::istringstream tmpstream(line);
                std::string _; float x, y, z;
                tmpstream >> _ >> x >> y >> z;
                end_joint->offset = glm::vec3(x, y, z);
                std::getline(file, line);
            } else if(word == "}") {
                return now_joint;
            }
        }
    }
    
    void BVH::get_channels(std::istringstream &iss, Joint *now_joint) {
        iss >> now_joint->num_channels;
        //std::cout << now_joint->num_channels << std::endl;
        now_joint->channel_idx = num_all_channels;
        num_all_channels += now_joint->num_channels;

        std::string channel_name;
        for(int i = 0; i < now_joint->num_channels; ++ i) {
            iss >> channel_name;

            if(channel_name == "Xrotation") now_joint->channels_order.push_back(1);
            if(channel_name == "Yrotation") now_joint->channels_order.push_back(2);
            if(channel_name == "Zrotation") now_joint->channels_order.push_back(3);

            if(channel_name == "Xposition") now_joint->channels_order.push_back(4);
            if(channel_name == "Yposition") now_joint->channels_order.push_back(5);
            if(channel_name == "Zposition") now_joint->channels_order.push_back(6);
        }
    }
    
    void BVH::load_motion(std::ifstream &file) {
        std::string _;

        file >> _ >> frames;
        //std::cout << _ << " " << frames;
        file >> _ >> _ >> frame_time;
        for(int i = 0; i < frames; ++ i) {
            for(int j = 0; j < num_all_channels; ++ j) {
                float x = 0; file >> x;
                channels_data.push_back(x);
            }
        }
    }
    
    void BVH::check_loading() {
        std::cout << "Checking information:\n";
        std::cout << root_joints.size() << " " << frames << " " << frame_time << " " << num_all_channels << std::endl;
        std::cout << channels_data.size() << std::endl;
        Joint *now = root_joints[0];
        if(root_joints.size() > 1) now = root_joints[1];
        for(Joint *s: now->sons) {
            std::cout << "son: " <<  s->name << std::endl;
        }
        while(1) {
            std::cout << now->name << std::endl;
            if(!now->sons.size()) break;
            now = now->sons[0];
        }    
    }
    
    float interp(float x, float y, int idx, int num) {
        float k = float(idx) / float(num);
        return y * k + x * (1 - k);
    }
    
    void out(glm::vec3 v) {
        std::cout << v.x << " " << v.y << " " << v.z << std::endl;
    }

    glm::vec3 quatinterp(glm::vec3 olds, glm::vec3 news, int idx, int num) {
        float k = float(idx) / float(num);
        glm::quat oldquat = glm::quat(glm::radians(olds));
        glm::quat newquat = glm::quat(glm::radians(news));
        float theta = glm::acos(glm::dot(oldquat, newquat));
        //std::cout << "theta1:" << theta << std::endl;
        if(glm::dot(oldquat, newquat) <= 0) {
            oldquat = -oldquat;
            theta = glm::acos(glm::dot(oldquat, newquat));
        }
        float sint = glm::sin(theta);
        //std::cout << "k: " << k << std::endl;
        //std::cout << glm::sin((1 - k) * theta) / sint << " " << glm::sin(k * theta) / sint << std::endl;  
        /*if(glm::sin(k * theta) / sint > 1) {
            std::cout << theta << " " << k << " problem" << std::endl;
        }*/
        glm::quat intquat = glm::sin((1 - k) * theta) / sint * oldquat +
                            glm::sin(k * theta) / sint * newquat;
        if(theta < 0.001) {
            //std::cout << "small theta\n";
            intquat = (1 - k) * oldquat + k * newquat;
        }
        intquat = glm::normalize(intquat);
        //std::cout << "theta2:" << theta << "\n";
        //std::cout << glm::degrees(glm::pi<float>());
        return glm::degrees(glm::eulerAngles(intquat));
    }

    void BVH::load_frame_data(int frame_idx, int interp_type) {
        prev_frame_data.clear();
        for(auto x: frame_data) prev_frame_data.push_back(x);
        frame_data.clear();
        if(switching) {
            ++ switch_frame_idx;
            if(interp_type == 0) {
                for(int i = 0; i < num_all_channels; ++ i) {
                    frame_data.push_back(interp(old_data[i], new_data[i], switch_frame_idx, switch_frame_num));
                }
            } else {
                for(int i = 0; i < 3; ++ i) frame_data.push_back(interp(old_data[i], new_data[i], switch_frame_idx, switch_frame_num));
                //std::cout << old_data[3] << " " << old_data[4] << " " << old_data[5] << " " << std::endl;
                //std::cout << new_data[3] << " " << new_data[4] << " " << new_data[5] << " " << std::endl;
                for(int i = 3; i < num_all_channels; i += 3) {
                    //int start = 0.4 * switch_frame_num;
                    //int end = 0.6 * switch_frame_num;
                    /*if(i == 3 || i == 5) {
                        if(switch_frame_idx >= start && switch_frame_idx <= end) {
                            frame_data.push_back(interp(old_data[i], new_data[i], switch_frame_idx - start, end - start));
                        } else if(frame_idx < start){
                            frame_data.push_back(old_data[i]);
                        } else {
                            frame_data.push_back(new_data[i]);
                        }
                    }
                    else */
                    glm::vec3 olds(old_data[i], old_data[i + 1], old_data[i + 2]);
                    glm::vec3 news(new_data[i], new_data[i + 1], new_data[i + 2]);
                    glm::vec3 res = quatinterp(olds, news, switch_frame_idx, switch_frame_num);
                    if(i == 3) {
                        //std::cout << "1: ";out(olds); 
                        //std::cout << "2: ";out(news);
                        //std::cout << res.x << " " << res.y << " " << res.z << std::endl;
                    }
                    frame_data.push_back(res.x);
                    frame_data.push_back(res.y);
                    frame_data.push_back(res.z);
                    //frame_data.push_back(Inertia_interp(prev_frame_data[i], old_data[i], new_data[i], switch_frame_num, switch_frame_idx));
                }
                //for(int i = 6; i < num_all_channels; ++ i) frame_data.push_back(interp(old_data[i], new_data[i], switch_frame_idx, switch_frame_num));
            }

            if(switch_frame_idx == switch_frame_num) {
                switching = 0;
                now_frame_idx = 0;
            }
            return;
        }
        int start = frame_idx * num_all_channels;
        for(int i = start; i < start + num_all_channels; ++ i) {
            frame_data.push_back(channels_data[i]);
        }
    }
    
    void BVH::calc_joint(Joint *joint, int frame_idx) {
        //joint->global_rotation = glm::mat4(1.0);
        //joint->global_position = glm::vec4(0, 0, 0 ,1);
        //std::cout << joint->num_channels << std::endl;
        //std::cout << joint->offset.x << " " << joint->offset.y << " " << joint->offset.z << std::endl;
        joint->trans_matrix = glm::translate(glm::mat4(1.0), joint->offset);
        //if(joint->parent) joint->global_position = joint->parent->global_position + 
                                    //joint->parent->global_rotation * glm::vec4(joint->offset, 1);
        for(int i = 0; i < joint->num_channels; ++ i) {
            int idx = joint->channels_order[i];
            //if(!joint->parent) std::cout << idx << std::endl;
            float val = frame_data[joint->channel_idx + i];
            //std::cout << val << " ";
            if(idx <= 3) val = glm::radians(val);
            //rotation
            if(idx == 1) joint->trans_matrix = glm::rotate(joint->trans_matrix, val, glm::vec3(1, 0, 0));
            if(idx == 2) joint->trans_matrix = glm::rotate(joint->trans_matrix, val, glm::vec3(0, 1, 0));
            if(idx == 3) joint->trans_matrix = glm::rotate(joint->trans_matrix, val, glm::vec3(0, 0, 1));
            /*
            if(idx == 1) joint->global_rotation = glm::rotate(joint->global_rotation, val, glm::vec3(1, 0, 0));
            if(idx == 2) joint->global_rotation = glm::rotate(joint->global_rotation, val, glm::vec3(0, 1, 0));
            if(idx == 3) joint->global_rotation = glm::rotate(joint->global_rotation, val, glm::vec3(0, 0, 1));
            */
            //position
            if(idx == 4) joint->trans_matrix = glm::translate(joint->trans_matrix, glm::vec3(val, 0, 0));
            if(idx == 5) joint->trans_matrix = glm::translate(joint->trans_matrix, glm::vec3(0, val, 0));
            if(idx == 6) joint->trans_matrix = glm::translate(joint->trans_matrix, glm::vec3(0, 0, val));
            /*
            if(idx == 4) joint->global_position.x = val;
            if(idx == 5) joint->global_position.y = val;
            if(idx == 6) joint->global_position.z = val;
            */
        }
        //if(!joint->parent) std::cout << joint->global_position.x << " " << joint->global_position.y << " " << joint->global_position.z << " " << std::endl;
        //std::cout << std::endl;
        //if(joint->parent) joint->global_rotation = joint->parent->global_rotation * joint->global_rotation;
        if(joint->parent) joint->trans_matrix = joint->parent->trans_matrix * joint->trans_matrix;
        for(Joint *s: joint->sons) {
            calc_joint(s, frame_idx);
        }
    }

    void BVH::get_positions(Joint *joint, std::vector<glm::vec3> &vertices,
                            std::vector<int> &indices, int parent_idx) {
        glm::vec4 trans = joint->trans_matrix[3];
        glm::vec3 pos = glm::vec3(trans.x, trans.y, trans.z) / trans.w;
        
        vertices.push_back(pos);
        int now_idx = vertices.size() - 1;
        if(now_idx != parent_idx) {
            indices.push_back(parent_idx);
            indices.push_back(now_idx);
        }
        for(Joint *s: joint->sons) {
            if(joint->name == "Head" && s->name == "End Joint") {
                head_arms_idx = indices.size() / 2;
            }
            get_positions(s, vertices, indices, now_idx);
        }
    }
   
    bool BVH::next_frame() {
        long long now_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        if(now_time - last_time > 1000 * frame_time) {
            //std::cout << now_time - last_time << " ";
            //std::cout << 1000 * frame_time << std::endl;
            last_time = now_time;
            return 1;
        }
        //std::cout << "0" << std::endl;
        return 0;
    }
    
    void BVH::switch_to(const std::string bvh_file, int need_frames) {
        switch_frame_num = need_frames;
        old_data.clear();
        for(int i = 0; i < num_all_channels; ++ i) {
            old_data.push_back(frame_data[i]);
        }
        BVH new_bvh; new_bvh.load_from_file(bvh_file);
        if(new_bvh.num_all_channels != num_all_channels) {
            std::cout << "This two bvh files can't switch because the model differs.\n";
            load_from_file(bvh_file);
            return;
        }
        new_data.clear();
        for(int i = 0; i < new_bvh.num_all_channels; ++ i) {
            new_data.push_back(new_bvh.channels_data[i]);
        }
        switching = 1;
        switch_frame_idx = 0;
        load_from_file(bvh_file);
    }
}
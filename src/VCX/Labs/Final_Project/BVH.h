#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Engine/loader.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include <chrono>

namespace VCX::Labs::Animation {
    class Joint {
    public:
        std::string name;
        Joint* parent;
        std::vector<Joint*> sons;
        glm::vec3 offset;
        glm::mat4 trans_matrix;
        int num_channels = 0;
        int channel_idx;
        std::vector<int> channels_order;
    };
    class BVH {
    public:
        std::vector<Joint*> root_joints;
        int frames;
        float frame_time;
        int now_frame_idx = 0;
        long long last_time = 0;
        int num_all_channels = 0;
        std::vector<float> channels_data;
        std::vector<float> frame_data;
        std::vector<float> old_data;
        std::vector<float> new_data;
        std::vector<float> prev_frame_data;
        bool switching = 0;
        int switch_frame_idx = 0;
        int switch_frame_num = 50;
        int transit_mode = 1;
        int head_arms_idx = 0;

    public: 
        void load_from_file(const std::string bvh_file);
        Joint* load_joint(std::ifstream &file, Joint *father_joint);
        void get_channels(std::istringstream &iss, Joint *now_joint);
        void load_motion(std::ifstream &file);
        void load_frame_data(int frame_idx, int interp_type = 0);
        void check_loading();
        void calc_joint(Joint *joint, int frame_idx);
        void get_positions(Joint *joint, std::vector<glm::vec3> &vertices,
                            std::vector<int> &indices, int parent_idx = 0);
        bool next_frame();
        void switch_to(const std::string bvh_file, int need_frames = 70);
    };
}
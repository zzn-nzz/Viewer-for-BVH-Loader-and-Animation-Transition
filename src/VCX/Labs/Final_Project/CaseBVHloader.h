#pragma once
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Engine/loader.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Final_Project/BVH.h"
#include "IKSystem.h"
#include "Labs/Final_Project/SceneObject.h"
#include "Labs/Final_Project/Content.h"


namespace VCX::Labs::Animation {

    enum IKType {
        CCD_IK = 0,
        FABR_IK = 1
    };

    class BoxRenderer {
    public:
        BoxRenderer();

        void render(Engine::GL::UniqueProgram & program);
        void calc_vert_position();

        std::vector<glm::vec3>              VertsPosition;
        glm::vec3                           CenterPosition;
        glm::vec3                           MainAxis;
        float                               width = 0.05f;
        float                               length = 0.2f;

        Engine::GL::UniqueIndexedRenderItem BoxItem;  // render the box
        Engine::GL::UniqueIndexedRenderItem LineItem; // render line on box
    };

    // render x, y, z axis
    class BackGroundRender {
    public:
        BackGroundRender(IKSystem & ik_system_);
        void render(Engine::GL::UniqueProgram & program);

    public:
        Engine::GL::UniqueIndexedRenderItem LineItem;
        Engine::GL::UniqueRenderItem        PointItem;
        Engine::GL::UniqueRenderItem        GTPointItem;
        IKSystem &                          ik_system;
    };

    class CaseBVHloader : public Common::ICase {
    public:
        CaseBVHloader(std::initializer_list<Assets::ExampleScene> && scenes);
        void pre_loading();
        virtual std::string_view const GetName() override { return "BVHloader"; }

        virtual void                        OnSetupPropsUI() override;
        virtual Common::CaseRenderResult    OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                        OnProcessInput(ImVec2 const & pos) override;

    private:
        void                                BuildByJointPosition();
        void                                ResetSystem();

    private:
        std::vector<Assets::ExampleScene> const _scenes;
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueProgram           _skyboxProgram;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager          _cameraManager;
        bool                                _stopped { false };
        std::size_t                         _sceneIdx           { 0 };
        bool                                _recompute          { true };
        bool                                _uniformDirty       { true };
        Rendering::SceneObject              _sceneObject;
        int                                 _msaa               { 2 };
        std::vector<BoxRenderer>            arms; // for render the arm
        IKSystem                            ik_system;
        int                                 ik_type = IKType::CCD_IK;
        int                                 action_type = 0;
        int                                 interp_type = 0;
        int                                 view_mode = 0;
        std::vector<glm::vec3>              vertices;
        std::vector<int>                    indices;
        BVH                                 bvh;
        float                               ratio = 0.03f;
        bool                                preload = 0;
        bool                                preloaded = 0;
        std::vector<BoxRenderer>            preload_arms[15000];
        const std::string bvh_paths[7] =     {"assets/bvh/Walking.bvh",
                                            "assets/bvh/KendoKata.bvh",
                                            "assets/bvh/Xinjiang.bvh",
                                            "assets/bvh/DanceTurn.bvh",
                                            "assets/bvh/MoonWalk.bvh",
                                            "assets/bvh/Chinese&Western.bvh",
                                            "assets/bvh/Jump.bvh"};
        char const *          GetSceneName(std::size_t const i) const { 
            if(i == 0) return "None";
            return Rendering::Content::SceneNames[std::size_t(_scenes[i - 1])].c_str(); 
        }
        Engine::Scene const & GetScene    (std::size_t const i) const { return Rendering::Content::Scenes[std::size_t(_scenes[i - 1])]; }
    };
} // namespace VCX::Labs::Animation

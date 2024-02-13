#pragma once
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Engine/loader.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Final_Project/CaseBVHloader.h"
#include "IKSystem.h"


namespace VCX::Labs::Animation {


    class CaseInverseKinematics : public Common::ICase {
    public:
        CaseInverseKinematics();

        virtual std::string_view const GetName() override { return "Inverse Kinematics System"; }

        virtual void                        OnSetupPropsUI() override;
        virtual Common::CaseRenderResult    OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                        OnProcessInput(ImVec2 const & pos) override;

    private:
        void                                BuildByJointPosition(const std::vector<glm::vec3> & joint_pos_);
        void                                ResetSystem();

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager          _cameraManager;
        bool                                _stopped { false };

        BackGroundRender                    BackGround;
        std::vector<BoxRenderer>            arms; // for render the arm
        IKSystem                            ik_system;
        int                                 ik_type = IKType::CCD_IK;
        
    };
} // namespace VCX::Labs::Animation

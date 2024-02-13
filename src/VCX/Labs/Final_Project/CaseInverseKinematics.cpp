#include <iostream>
#include "Engine/app.h"
#include "Labs/Final_Project/CaseInverseKinematics.h"
#include "tasks.h"
#include "Labs/Common/ImGuiHelper.h"


namespace VCX::Labs::Animation {

    CaseInverseKinematics::CaseInverseKinematics():
        BackGround(ik_system),
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") }))
    {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
        for (int i = 0; i < 4; i++) {
            arms.emplace_back(BoxRenderer());
        }
        
        ResetSystem();
    }

    void CaseInverseKinematics::BuildByJointPosition(const std::vector<glm::vec3> & JointPosition) {
        for (int i = 0; i < JointPosition.size() - 1; i++) {
            auto MainAxis = JointPosition[i + 1] - JointPosition[i];
            arms[i].length = glm::l2Norm(MainAxis);
            arms[i].MainAxis = MainAxis / arms[i].length;
            arms[i].CenterPosition = 0.5f * (JointPosition[i] + JointPosition[i + 1]);
        }
    }

    void CaseInverseKinematics::OnSetupPropsUI() {
            const char * ik_items[] = { "ccd_ik", "fabr_ik"};
            if (ImGui::BeginCombo("Algorithm", ik_items[ik_type])) 
            {
                for (int i = 0; i < 2; i++) {
                    bool selected = i == ik_type;
                    if (ImGui::Selectable(ik_items[i], selected)) {
                        if (! selected) ik_type = i;
                    }
                }
                ImGui::EndCombo();
            }
            if (ImGui::Button(_stopped ? "Start" : "Pause")) _stopped = ! _stopped;
            ImGui::SameLine();
            if (ImGui::Button("Reset")) ResetSystem();
            if (ImGui::BeginCombo("Shape", draw_items[ik_system.TargetPositionIndex])) {
                for (int i = 0; i < 6; i++) {
                    bool selected = i == ik_system.TargetPositionIndex;
                    if (ImGui::Selectable(draw_items[i], selected)) {
                        if (! selected) {
                            ik_system.TargetPositionIndex = i;
                            ik_system.CurrIndex            = 0;
                        }
                    }
                }
                ImGui::EndCombo();
            }
    }

    Common::CaseRenderResult CaseInverseKinematics::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(0.5f);
        glPointSize(4.f);

        if (!_stopped) {
            auto index  = ik_system.CurrIndex;
            auto & target = ik_system.GetTarget();
            if (ik_type == IKType::CCD_IK) {
                InverseKinematicsCCD(ik_system, target, 100, 1e-4f);
            } else if (ik_type == IKType::FABR_IK) {
                InverseKinematicsFABR(ik_system, target, 100, 1e-4f);
            }
            ik_system.EndPositionHistory[index] = ik_system.EndEffectorPosition();
            BuildByJointPosition(ik_system.JointGlobalPosition);
            auto * ptr = ik_system.EndPositionHistory.data();
            auto hist_span = std::span<const std::byte>(reinterpret_cast<const std::byte *> (ptr), reinterpret_cast<const std::byte *>(ptr + ik_system.CurrIndex));
            BackGround.PointItem.UpdateVertexBuffer("position", hist_span);
            BackGround.GTPointItem.UpdateVertexBuffer("position", Engine::make_span_bytes<const glm::vec3>(*ik_system.GetTargetPositionList()));
        }
        
        for (std::size_t i = 0; i < arms.size(); i++) {
            arms[i].calc_vert_position();
            arms[i].render(_program);
        }
        BackGround.render(_program);

        glLineWidth(1.f);
        glPointSize(1.f);
        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseInverseKinematics::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    void CaseInverseKinematics::ResetSystem() {
        ik_system.CurrIndex = 0;
        BuildByJointPosition(ik_system.InitJointPosition);
    }
} // namespace VCX::Labs::Animation

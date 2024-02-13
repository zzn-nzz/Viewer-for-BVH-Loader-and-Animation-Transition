#include <iostream>
#include "Engine/app.h"
#include "Labs/Final_Project/CaseBVHloader.h"
#include "tasks.h"
#include "Labs/Common/ImGuiHelper.h"
#include "Assets/bundled.h"


namespace VCX::Labs::Animation {

    BackGroundRender::BackGroundRender(IKSystem & ik_system_):
        ik_system(ik_system_),
        LineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        GTPointItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Points),
        PointItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Points) {
        const std::uint32_t index[] = { 0, 1, 0, 2, 0, 3 };
        float pos[12] = { 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
        LineItem.UpdateElementBuffer(index);
        LineItem.UpdateVertexBuffer("position", std::span<std::byte>(reinterpret_cast<std::byte *>(pos), reinterpret_cast<std::byte *>(pos + 12)));
    }

    void BackGroundRender::render(Engine::GL::UniqueProgram & program) {
        LineItem.Draw({ program.Use() });
        program.GetUniforms().SetByName("u_Color", glm::vec3(0.0f, 0.8f, 0.0f));
        GTPointItem.Draw({ program.Use() });
        program.GetUniforms().SetByName("u_Color", glm::vec3(1.0f, 0.0f, 0.0f));
        PointItem.Draw({ program.Use() });
    }

    BoxRenderer::BoxRenderer():
        CenterPosition(0, 0, 0),
        MainAxis(0, 1, 0),
        BoxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        LineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        //     3-----2
        //    /|    /|
        //   0 --- 1 |
        //   | 7 - | 6
        //   |/    |/
        //   4 --- 5
        VertsPosition.resize(8);
        const std::vector<std::uint32_t> line_index = {0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7}; // line index
        LineItem.UpdateElementBuffer(line_index);

        const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 4, 0, 1, 4, 5, 1, 6, 5, 1, 2, 6, 2, 3, 7, 2, 6, 7, 0, 3, 7, 0, 4, 7, 4, 5, 6, 4, 6, 7};
        BoxItem.UpdateElementBuffer(tri_index);
    }

    void BoxRenderer::render(Engine::GL::UniqueProgram & program)
    {
        auto span_bytes = Engine::make_span_bytes<glm::vec3>(VertsPosition);
        
        program.GetUniforms().SetByName("u_Color", glm::vec3(121.0f / 255, 207.0f / 255, 171.0f / 255));
        BoxItem.UpdateVertexBuffer("position", span_bytes);
        BoxItem.Draw({ program.Use() });

        program.GetUniforms().SetByName("u_Color", glm::vec3(1.0f, 1.0f, 1.0f));
        LineItem.UpdateVertexBuffer("position", span_bytes);
        LineItem.Draw({ program.Use() });
    }

    void BoxRenderer::calc_vert_position() {
        glm::vec3 new_y = glm::normalize(MainAxis);
        glm::quat quat = glm::rotation(glm::vec3(0, 1, 0), new_y);
        glm::vec3 new_x = quat * glm::vec3(0.5f * width, 0.0f, 0.0f);
        glm::vec3 new_z = quat * glm::vec3(0.0f, 0.0f, 0.5f * width);
        const glm::vec3 & c = CenterPosition;
        new_y *= 0.5 * length;
        VertsPosition[0] = c - new_x + new_y + new_z;
        VertsPosition[1] = c + new_x + new_y + new_z;
        VertsPosition[2] = c + new_x + new_y - new_z;
        VertsPosition[3] = c - new_x + new_y - new_z;
        VertsPosition[4] = c - new_x - new_y + new_z;
        VertsPosition[5] = c + new_x - new_y + new_z;
        VertsPosition[6] = c + new_x - new_y - new_z;
        VertsPosition[7] = c - new_x - new_y - new_z;
    }

    CaseBVHloader::CaseBVHloader(std::initializer_list<Assets::ExampleScene> && scenes):
        _scenes(scenes),
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _skyboxProgram(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/skybox.vert"),
                Engine::GL::SharedShader("assets/shaders/skybox.frag") })),
        _sceneObject(2) {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        bvh.load_from_file(bvh_paths[0]);
        //bvh.check_loading();

        _skyboxProgram.BindUniformBlock("PassConstants", 2);
        _skyboxProgram.GetUniforms().SetByName("u_Skybox", 1);
        
    }

    void CaseBVHloader::BuildByJointPosition() {
        int n = indices.size() / 2;
        while(arms.size() < n) arms.push_back(BoxRenderer());
        for(int i = 0; i < n; ++ i) {
            auto MainAxis = ratio * (vertices[indices[i * 2 + 1]] - vertices[indices[i * 2]]);
            arms[i].length = glm::l2Norm(MainAxis);
            arms[i].MainAxis = MainAxis / arms[i].length;
            arms[i].CenterPosition = 0.5f * ratio * (vertices[indices[i * 2 + 1]] + vertices[indices[i * 2]]);
        }
    }

    void CaseBVHloader::pre_loading() {
        preloaded = 1;
        int total = bvh.frames;
        for(int i = 0; i < total; ++ i) {
            vertices.clear(); indices.clear();
            bvh.now_frame_idx = i;
            for(Joint *s: bvh.root_joints) {
                bvh.calc_joint(s, bvh.now_frame_idx);
            }
            for(Joint *s: bvh.root_joints) {
                bvh.get_positions(s, vertices, indices, vertices.size());
            }
            int n = indices.size() / 2;
            while(preload_arms[i].size() < n) preload_arms[i].push_back(BoxRenderer());
            for(int j = 0; j < n; ++ j) {
                //std::cout << " " << i << " " << j << " " << std::endl;
                auto MainAxis = ratio * (vertices[indices[j * 2 + 1]] - vertices[indices[j * 2]]);
                preload_arms[i][j].length = glm::l2Norm(MainAxis);
                preload_arms[i][j].MainAxis = MainAxis / preload_arms[i][j].length;
                preload_arms[i][j].CenterPosition = 0.5f * ratio * (vertices[indices[j * 2 + 1]] + vertices[indices[j * 2]]);
            }
            std::cout << i << " " << std::endl;
        }
    }

    void CaseBVHloader::OnSetupPropsUI() {
            if (ImGui::BeginCombo("Scene", GetSceneName(_sceneIdx))) {
                for (std::size_t i = 0; i < _scenes.size() + 1; ++i) {
                    bool selected = i == _sceneIdx;
                    if (ImGui::Selectable(GetSceneName(i), selected)) {
                        if (! selected) {
                            _sceneIdx           = i;
                            _recompute          = true;
                            _uniformDirty = true;
                        }
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::Spacing();
            const char * action_items[] = { "Walking", "JapaneseKendo", "XinjiangDance", "ChineseDanceTurn", "MoonWalk", "Chinese&Western", "jump"};
            if (ImGui::BeginCombo("Action", action_items[action_type])) 
            {
                for (int i = 0; i < 7; i++) {
                    bool selected = i == action_type;
                    if (ImGui::Selectable(action_items[i], selected)) {
                        if (! selected) {
                            arms.clear();
                            bvh.switch_to(bvh_paths[i]);
                            action_type = i;
                        }
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::Spacing();
            const char* interp[2] = {"linear", "quat"};
            if (ImGui::BeginCombo("Interpolation", interp[interp_type])) {
                for (std::size_t i = 0; i < 2; ++ i) {
                    bool selected = i == interp_type;
                    if (ImGui::Selectable(interp[i], selected)) {
                        if (! selected) {
                            interp_type = i;
                        }
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::Spacing();
            const char* view[3] = {"outsider", "head-view", "hip-view"};
            if (ImGui::BeginCombo("View", view[view_mode])) {
                for (std::size_t i = 0; i < 3; ++ i) {
                    bool selected = i == view_mode;
                    if (ImGui::Selectable(view[i], selected)) {
                        if (! selected) {
                            view_mode = i;
                        }
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::Spacing();
            Common::ImGuiHelper::SaveImage(_frame.GetColorAttachment(), _frame.GetSize(), true);
            //BVH my_bvh; my_bvh.load_from_file("/Users/zhangzhining/Downloads/cmuconvert-daz-141-144/144/144_01.bvh");
            //my_bvh.check_loading();
            ImGui::Spacing();
            if (ImGui::Button(_stopped ? "Start" : "Pause")) _stopped = ! _stopped;
            ImGui::SameLine();
            if (ImGui::Button("Reset")) ResetSystem();
    }

    Common::CaseRenderResult CaseBVHloader::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (_sceneIdx && _recompute) {
            _recompute = false;
            _sceneObject.ReplaceScene(GetScene(_sceneIdx));
            if (_sceneObject.Skybox.has_value())
                _sceneObject.Skybox.value().CubeMap.SetUnit(1);
            _cameraManager.Save(_sceneObject.Camera);
        }
        
        if(_sceneIdx) _frame.Resize(desiredSize, 1 << _msaa);
        else _frame.Resize(desiredSize, 1);

        
        
        gl_using(_frame);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(1.0f);//0.5f
        glPointSize(4.f);
        if(_sceneIdx) {
            glEnable(GL_DEPTH_TEST);
            auto const & skybox = _sceneObject.Skybox.value();
            glDepthFunc(GL_LEQUAL);
            skybox.Mesh.Draw({ skybox.CubeMap.Use(), _skyboxProgram.Use() });
            glDepthFunc(GL_LESS);
            glDisable(GL_DEPTH_TEST);
        }

        if (!_stopped) {
            //check if frame_time is passed
            if(preload) {
                if(!preloaded) pre_loading();
                if(bvh.next_frame()) {
                    int i = bvh.now_frame_idx;
                    for (std::size_t j = 0; j < preload_arms[i].size(); j++) {
                        //std::cout << j << " " << std::endl;
                        preload_arms[i][j].calc_vert_position();
                        preload_arms[i][j].render(_program);
                    }
                    ++ bvh.now_frame_idx;
                    if(bvh.now_frame_idx >= bvh.frames) bvh.now_frame_idx = 0;
                }
            } else if(bvh.next_frame()) {
                vertices.clear(); indices.clear();
                bvh.load_frame_data(bvh.now_frame_idx, interp_type);
                for(Joint *s: bvh.root_joints) {
                    bvh.calc_joint(s, bvh.now_frame_idx);
                }
                for(Joint *s: bvh.root_joints) {
                    bvh.get_positions(s, vertices, indices, vertices.size());
                }
                BuildByJointPosition();
                ++ bvh.now_frame_idx;
                //std::cout << bvh.now_frame_idx << " " << bvh.frames << std::endl;
                if(bvh.now_frame_idx >= bvh.frames) bvh.now_frame_idx = 0;
            }
        }
        if(!preload) {
            for (std::size_t i = 0; i < arms.size(); i++) {
                arms[i].calc_vert_position();
                arms[i].render(_program);
            }
        }
        int view_idx = 0;
        if(view_mode == 1) view_idx = bvh.head_arms_idx;
        glm::vec3 up = 0.25f * (arms[view_idx].VertsPosition[3] + arms[view_idx].VertsPosition[3] + arms[view_idx].VertsPosition[3] + arms[view_idx].VertsPosition[3]);
        glm::vec3 targ = 0.25f * (arms[view_idx].VertsPosition[7] + arms[view_idx].VertsPosition[7] + arms[view_idx].VertsPosition[7] + arms[view_idx].VertsPosition[7]);
        glm::mat4 head_view = glm::lookAt(up, up + targ, glm::vec3(0, 1, 0));
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        if(view_mode == 0) _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix()),
                        _skyboxProgram.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());
        else _program.GetUniforms().SetByName("u_View", head_view),
            _skyboxProgram.GetUniforms().SetByName("u_View", head_view);
        glLineWidth(1.f);
        glPointSize(1.f);
        glDisable(GL_LINE_SMOOTH);

        _cameraManager.Update(_camera);
        if(_sceneIdx) {
            _cameraManager.Update(_sceneObject.Camera);
            _sceneObject.PassConstantsBlock.Update(&Rendering::SceneObject::PassConstants::Projection, _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
            if(view_mode == 0) _sceneObject.PassConstantsBlock.Update(&Rendering::SceneObject::PassConstants::View, _sceneObject.Camera.GetViewMatrix());
            else _sceneObject.PassConstantsBlock.Update(&Rendering::SceneObject::PassConstants::View, head_view);
            if(view_mode == 0) _sceneObject.PassConstantsBlock.Update(&Rendering::SceneObject::PassConstants::ViewPosition, _sceneObject.Camera.Eye);
            else _sceneObject.PassConstantsBlock.Update(&Rendering::SceneObject::PassConstants::ViewPosition, up);
        }

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseBVHloader::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    void CaseBVHloader::ResetSystem() {
        //ik_system.CurrIndex = 0;
        //BuildByJointPosition(ik_system.InitJointPosition);
        bvh.now_frame_idx = 0;
    }
} // namespace VCX::Labs::Animation

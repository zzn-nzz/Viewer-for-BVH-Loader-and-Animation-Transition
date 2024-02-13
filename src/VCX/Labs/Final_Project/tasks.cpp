#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>
#include <iostream>
#include "Labs/Final_Project/tasks.h"
#include "IKSystem.h"
#include "CustomFunc.inl"


namespace VCX::Labs::Animation {
    void ForwardKinematics(IKSystem & ik, int StartIndex) {
        if (StartIndex == 0) {
            ik.JointGlobalRotation[0] = ik.JointLocalRotation[0];
            ik.JointGlobalPosition[0] = ik.JointLocalOffset[0];
            StartIndex                = 1;
        }
        
        for (int i = StartIndex; i < ik.JointLocalOffset.size(); i++) {
            // your code here: forward kinematics, update JointGlobalPosition and JointGlobalRotation
            ik.JointGlobalPosition[i] = ik.JointGlobalPosition[i - 1] + glm::rotate(ik.JointGlobalRotation[i - 1], ik.JointLocalOffset[i]);
            ik.JointGlobalRotation[i] = ik.JointGlobalRotation[i - 1] * ik.JointLocalRotation[i];
        }
    }

    void my_debug(glm::vec3 aa, char* s) {
        printf("%s: (%f, %f, %f)\n", s, aa.x, aa.y, aa.z);
    }
    void my_debug(glm::quat aa, char* s) {
        printf("%s: (%f, %f, %f, %f)\n", s, aa.w, aa.x, aa.y, aa.z);
    }
    void InverseKinematicsCCD(IKSystem & ik, const glm::vec3 & EndPosition, int maxCCDIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        //printf("eps: %f\n", eps);
        // These functions will be useful: glm::normalize, glm::rotation, glm::quat * glm::quat
        int CCDIKIteration = 0;
        for (CCDIKIteration = 0; CCDIKIteration < maxCCDIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; CCDIKIteration++) {
            // your code here: ccd ik
            int N = ik.NumJoints();
            for(int i = N - 2; i >= 0; -- i) {
                glm::vec3 target = glm::normalize(EndPosition - ik.JointGlobalPosition[i]);
                glm::vec3 init = glm::normalize(ik.JointGlobalPosition[N - 1] - ik.JointGlobalPosition[i]);
                glm::quat newR = glm::rotation(init, target);
                ik.JointLocalRotation[i] = newR * ik.JointLocalRotation[i];
                ForwardKinematics(ik, i);
            }
        }
        //printf("CCD: %d\n", CCDIKIteration);
    }

    void InverseKinematicsFABR(IKSystem & ik, const glm::vec3 & EndPosition, int maxFABRIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        int nJoints = ik.NumJoints();
        std::vector<glm::vec3> backward_positions(nJoints, glm::vec3(0, 0, 0)), forward_positions(nJoints, glm::vec3(0, 0, 0));
        int IKIteration = 0;
        for (IKIteration = 0; IKIteration < maxFABRIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; IKIteration++) {
            // task: fabr ik
            // backward update
            glm::vec3 next_position         = EndPosition;
            backward_positions[nJoints - 1] = EndPosition;

            for (int i = nJoints - 2; i >= 0; i--) {
                // your code here
                glm::vec3 dir = ik.JointGlobalPosition[i] - backward_positions[i + 1];
                backward_positions[i] = backward_positions[i + 1] + glm::normalize(dir) * glm::length(ik.JointLocalOffset[i + 1]);
            }

            // forward update
            glm::vec3 now_position = ik.JointGlobalPosition[0];
            forward_positions[0] = ik.JointGlobalPosition[0];
            for (int i = 0; i < nJoints - 1; i++) {
                // your code here
                glm::vec3 dir = backward_positions[i + 1] - forward_positions[i];
                forward_positions[i + 1] = forward_positions[i] + normalize(dir) * glm::length(ik.JointLocalOffset[i + 1]);
            }
            ik.JointGlobalPosition = forward_positions; // copy forward positions to joint_positions
        }
        //printf("FABR: %d\n", IKIteration);
        // Compute joint rotation by position here.
        for (int i = 0; i < nJoints - 1; i++) {
            ik.JointGlobalRotation[i] = glm::rotation(glm::normalize(ik.JointLocalOffset[i + 1]), glm::normalize(ik.JointGlobalPosition[i + 1] - ik.JointGlobalPosition[i]));
        }
        ik.JointLocalRotation[0] = ik.JointGlobalRotation[0];
        for (int i = 1; i < nJoints - 1; i++) {
            ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i - 1]) * ik.JointGlobalRotation[i];
        }
        //printf("bef: %f\n", glm::l2Norm(ik.EndEffectorPosition() - EndPosition));
        ForwardKinematics(ik, 0);
        //printf("aft: %f\n", glm::l2Norm(ik.EndEffectorPosition() - EndPosition));
    }

    IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
        // get function from https://www.wolframalpha.com/input/?i=Albert+Einstein+curve
        int nums = 500;
        using Vec3Arr = std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(nums));
        int index = 0;
        float height = 2.0f;
        float radius = 0.3f; 
        float twistRate = 8.0f * glm::pi<float>();
        for (int i = 0; i < nums; i++) {
            float theta = twistRate * i / nums;
            float x_val = radius * std::cos(theta);
            float y_val = radius * std::sin(theta);
            float z_val = height * i / nums;
            (*custom)[index++] = glm::vec3(x_val, y_val, z_val);
        }
        custom->resize(index);
        return custom;
    }

    static Eigen::VectorXf glm2eigen(std::vector<glm::vec3> const & glm_v) {
        Eigen::VectorXf v = Eigen::Map<Eigen::VectorXf const, Eigen::Aligned>(reinterpret_cast<float const *>(glm_v.data()), static_cast<int>(glm_v.size() * 3));
        return v;
    }

    static std::vector<glm::vec3> eigen2glm(Eigen::VectorXf const & eigen_v) {
        return std::vector<glm::vec3>(
            reinterpret_cast<glm::vec3 const *>(eigen_v.data()),
            reinterpret_cast<glm::vec3 const *>(eigen_v.data() + eigen_v.size())
        );
    }

    static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t n, std::vector<Eigen::Triplet<float>> const & triplets) {
        Eigen::SparseMatrix<float> matLinearized(n, n);
        matLinearized.setFromTriplets(triplets.begin(), triplets.end());
        return matLinearized;
    }

    // solve Ax = b and return x
    static Eigen::VectorXf ComputeSimplicialLLT(
        Eigen::SparseMatrix<float> const & A,
        Eigen::VectorXf const & b) {
        auto solver = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(A);
        return solver.solve(b);
    }

    void AdvanceMassSpringSystem(MassSpringSystem & system, float const dt) {
        // your code here:
        std::vector<glm::vec3> nabla_E(system.Positions.size(), glm::vec3(0));
        std::vector<std::vector<glm::mat3> > H;
        for(int i = 0; i < system.Positions.size(); ++ i) {
            std::vector<glm::mat3> tmp(system.Positions.size(), glm::mat3(0));
            H.push_back(tmp);
        }
        for (auto const spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
            glm::vec3 const e01 = glm::normalize(x01);
            glm::vec3 f = (system.Stiffness * (glm::length(x01) - spring.RestLength)) * e01;
            nabla_E[p0] -= f;
            nabla_E[p1] += f;
            glm::mat3 tmp = glm::outerProduct(-x01, -x01) / glm::length2(-x01);
            glm::mat3 He = system.Stiffness * tmp + system.Stiffness * (1 - spring.RestLength / glm::length(-x01)) * (glm::mat3(1.0f) - tmp);
            H[p0][p0] += He, H[p1][p1] += He, H[p0][p1] -= He, H[p1][p0] -= He;
        }
        std::vector<glm::vec3> neg_nabla_g(system.Positions.size(), glm::vec3(0));
        for (std::size_t i = 0; i < system.Positions.size(); i++) {
            if (system.Fixed[i]) continue;
            glm::vec3 y = system.Positions[i] + dt * (system.Velocities[i] + dt * glm::vec3(0, -system.Gravity, 0));
            neg_nabla_g[i] = -(1.0f / (dt * dt) * system.Mass * (system.Positions[i] - y) + nabla_E[i]);
        }
        std::vector<Eigen::Triplet<float>> triplets;
        for(int i = 0; i < system.Positions.size(); ++ i) {
            for(int j = 0; j < system.Positions.size(); ++ j) {
                if(i == j) H[i][j] += glm::mat3(system.Mass / (dt * dt));
                for(int k = 0; k < 3; ++ k) {
                    for(int l = 0; l < 3; ++ l) {
                        int x = i * 3 + k, y = j * 3 + l;
                        if(H[i][j][l][k] != 0) {
                            triplets.emplace_back(x, y, H[i][j][l][k]);
                        }
                    }
                }
            }
        }
        //printf("%d %d\n", system.Positions.size(), triplets.size());
        auto A = CreateEigenSparseMatrix(3 * system.Positions.size(), triplets);
        auto newxi = eigen2glm(ComputeSimplicialLLT(A, glm2eigen(neg_nabla_g)));
        for(int i = 0; i < system.Positions.size(); ++ i) {
            if(system.Fixed[i]) continue;
            system.Positions[i] = system.Positions[i] + newxi[i];
        }
        std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
        for (auto const spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
            glm::vec3 const e01 = glm::normalize(x01);
            glm::vec3 f = (system.Stiffness * (glm::length(x01) - spring.RestLength)) * e01;
            forces[p0] += f; 
            forces[p1] -= f;
        }
        for(int i = 0; i < system.Positions.size(); ++ i) {
            if(system.Fixed[i]) continue;
            system.Velocities[i] += (glm::vec3(0, -system.Gravity, 0) + forces[i] / system.Mass) * dt;
        }
    }
}

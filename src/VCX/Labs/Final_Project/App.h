#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/Final_Project/CaseMassSpring.h"
#include "Labs/Final_Project/CaseInverseKinematics.h"
#include "Labs/Final_Project/CaseBVHloader.h"
#include "Labs/Common/UI.h"
#include "Assets/bundled.h"

namespace VCX::Labs::Animation {

    class App : public Engine::IApp {
    private:
        Common::UI             _ui;

        CaseBVHloader          _caseBVHloader;

        std::size_t        _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = {
            _caseBVHloader
        };

    public:
        App();
        void OnFrame() override;
    };
}

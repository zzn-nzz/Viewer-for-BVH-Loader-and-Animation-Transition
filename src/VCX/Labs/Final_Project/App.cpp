#include "Labs/Final_Project/App.h"
#include "Assets/bundled.h"
namespace VCX::Labs::Animation {
    App::App() : _ui(Labs::Common::UIOptions { }),
                _caseBVHloader({ Assets::ExampleScene::Teapot, Assets::ExampleScene::Bunny }) {
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}

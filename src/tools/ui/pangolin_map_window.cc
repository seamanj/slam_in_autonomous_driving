#include "tools/ui/pangolin_map_window.h"

#include <glog/logging.h>

#include "tools/ui/pangolin_map_window_impl.h"

namespace sad::ui {

PangolinMapWindow::PangolinMapWindow() {
    impl_ = std::make_shared<PangolinMapWindowImpl>();
}

PangolinMapWindow::~PangolinMapWindow() {
    LOG(INFO) << "pangolin map window deallocated.";
    Quit();
}

bool PangolinMapWindow::Init() {
    impl_->map_need_update_.store(false);

    bool inited = impl_->Init();

    if (inited) {
        impl_->render_thread_ =
            std::thread([this]() { impl_->Render(); });
    }

    return inited;
}

void PangolinMapWindow::Quit() {

    if (impl_->render_thread_.joinable()) {

        impl_->exit_flag_.store(true);

        impl_->render_thread_.join();
    }

    impl_->DeInit();
}

void PangolinMapWindow::UpdateMap(
    CloudPtr cloud,
    const SE3& pose) {

    std::lock_guard<std::mutex> lock(
        impl_->mtx_map_);

    *impl_->local_map_ = *cloud;

    impl_->current_pose_ = pose;

    impl_->map_need_update_.store(true);
}

bool PangolinMapWindow::ShouldQuit() {
    return pangolin::ShouldQuit();
}

}
#include "tools/ui/pangolin_map_window_impl.h"

#include <glog/logging.h>

namespace sad::ui {

using UL = std::unique_lock<std::mutex>;

bool PangolinMapWindowImpl::Init() {
    pangolin::CreateWindowAndBind(
        win_name_,
        win_width_,
        win_height_);

    glEnable(GL_DEPTH_TEST);

    AllocateBuffer();

    pangolin::GetBoundWindow()->RemoveCurrent();

    local_map_.reset(new PointCloudType);

    map_ui_ = std::make_shared<UiCloud>();

    return true;
}

bool PangolinMapWindowImpl::DeInit() {
    ReleaseBuffer();
    return true;
}

bool PangolinMapWindowImpl::UpdateMap() {

    if (!map_need_update_.load())
        return false;

    UL lock(mtx_map_);

    map_ui_->SetCloud(local_map_, current_pose_);
    map_ui_->SetRenderColor(
        UiCloud::UseColor::HEIGHT_COLOR);

    map_need_update_.store(false);

    return true;
}

void PangolinMapWindowImpl::DrawAll() {

    map_ui_->Render();

    car_.SetPose(current_pose_);
    car_.Render();
}

void PangolinMapWindowImpl::CreateDisplayLayout() {

    auto proj =
        pangolin::ProjectionMatrix(
            win_width_,
            win_height_,
            cam_focus_,
            cam_focus_,
            win_width_ / 2,
            win_height_ / 2,
            cam_z_near_,
            cam_z_far_);

    auto model =
        pangolin::ModelViewLookAt(
            0,
            0,
            1000,
            0,
            0,
            0,
            pangolin::AxisY);

    s_cam_main_ =
        pangolin::OpenGlRenderState(
            std::move(proj),
            std::move(model));

    pangolin::Display(dis_3d_main_name_)
        .SetBounds(0.0, 1.0, 0.0, 1.0)
        .SetHandler(
            new pangolin::Handler3D(
                s_cam_main_));
}

void PangolinMapWindowImpl::Render() {

    pangolin::BindToContext(win_name_);

    glEnable(GL_DEPTH_TEST);

    CreateDisplayLayout();

    exit_flag_.store(false);

    while (!pangolin::ShouldQuit() &&
           !exit_flag_) {

        glClearColor(
            1.0,
            1.0,
            1.0,
            1.0);

        glClear(
            GL_COLOR_BUFFER_BIT |
            GL_DEPTH_BUFFER_BIT);

        UpdateMap();

        pangolin::Display(
            dis_3d_main_name_)
            .Activate(s_cam_main_);

        DrawAll();

        if (following_) {
            s_cam_main_.Follow(
                current_pose_.matrix());
        }

        pangolin::FinishFrame();
    }

    pangolin::GetBoundWindow()->RemoveCurrent();
}

void PangolinMapWindowImpl::AllocateBuffer() {}

void PangolinMapWindowImpl::ReleaseBuffer() {}

} // namespace sad::ui
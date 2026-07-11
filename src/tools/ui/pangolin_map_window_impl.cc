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

    traj_lidarloc_ui_.reset(new ui::UiTrajectory(Vec3f(1.0, 0.0, 0.0)));      // 红色
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

    map_ui_->SetCloud(local_map_, SE3());
    map_ui_->SetRenderColor(UiCloud::UseColor::HEIGHT_COLOR);

    map_need_update_.store(false);

    return true;
}

void PangolinMapWindowImpl::DrawAll() {

    pangolin::glDrawAxis(10.0);

    map_ui_->Render();

    car_.SetPose(current_pose_);
    car_.Render();
    traj_lidarloc_ui_->Render();
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

    // 真正的3D窗口
    pangolin::View& d_cam3d_main =
        pangolin::Display(dis_3d_main_name_)
            .SetBounds(0.0, 1.0, 0.0, 1.0)
            .SetHandler(new pangolin::Handler3D(s_cam_main_));

    // 外层Display
    pangolin::View& d_cam3d =
        pangolin::Display(dis_3d_name_)
            .SetBounds(0.0, 1.0, 0.0, 1.0)
            .SetLayout(pangolin::LayoutOverlay)
            .AddDisplay(d_cam3d_main);

    pangolin::Display(dis_main_name_)
        .SetBounds(
            0.0,
            1.0,
            pangolin::Attach::Pix(menu_width_),
            1.0)
        .AddDisplay(d_cam3d);
}

void PangolinMapWindowImpl::Render() {

    pangolin::BindToContext(win_name_);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // menu
    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(menu_width_));
    pangolin::Var<bool> menu_follow_loc("menu.Follow", false, true);
    pangolin::Var<bool> menu_reset_3d_view("menu.Reset 3D View", false, false);
    pangolin::Var<bool> menu_reset_front_view("menu.Set to front View", false, false);


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


        // menu control
        following_loc_ = menu_follow_loc;

        if (menu_reset_3d_view) {
            s_cam_main_.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0, 1000, 0, 0, 0, pangolin::AxisY));
            menu_reset_3d_view = false;
        }
        if (menu_reset_front_view) {
            s_cam_main_.SetModelViewMatrix(pangolin::ModelViewLookAt(-50, 0, 10, 50, 0, 10, pangolin::AxisZ));
            menu_reset_front_view = false;
        }




        UpdateMap();
        traj_lidarloc_ui_->AddPt(current_pose_);
        pangolin::Display(
            dis_3d_main_name_)
            .Activate(s_cam_main_);

        DrawAll();

        if (following_loc_) {
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
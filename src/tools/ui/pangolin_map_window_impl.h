#ifndef SLAM_IN_AUTO_DRIVING_PANGOLIN_MAP_WINDOW_IMPL_H
#define SLAM_IN_AUTO_DRIVING_PANGOLIN_MAP_WINDOW_IMPL_H

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include <pangolin/pangolin.h>

#include "common/point_types.h"
#include "tools/ui/ui_car.h"
#include "tools/ui/ui_cloud.h"
#include "tools/ui/ui_trajectory.h"


namespace sad::ui {

class PangolinMapWindowImpl {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PangolinMapWindowImpl() = default;
    ~PangolinMapWindowImpl() = default;

    PangolinMapWindowImpl(const PangolinMapWindowImpl&) = delete;
    PangolinMapWindowImpl& operator=(const PangolinMapWindowImpl&) = delete;

    bool Init();
    bool DeInit();

    void Render();

public:
    /// 后台渲染线程
    std::thread render_thread_;

    /// 数据同步
    std::mutex mtx_map_;

    std::atomic<bool> exit_flag_{false};
    std::atomic<bool> map_need_update_{false};

    /// 当前显示地图
    CloudPtr local_map_ = nullptr;

    /// 当前车辆Pose
    SE3 current_pose_;

private:
    /// OpenGL Buffer
    void AllocateBuffer();
    void ReleaseBuffer();

    /// 创建布局
    void CreateDisplayLayout();

    /// 更新地图
    bool UpdateMap();

    /// 绘制
    void DrawAll();

private:
    //////////////////////////////
    /// Window
    //////////////////////////////

    int win_width_ = 1920;
    int win_height_ = 1080;

    static constexpr float cam_focus_ = 5000;
    static constexpr float cam_z_near_ = 1.0;
    static constexpr float cam_z_far_ = 1e10;
    static constexpr int menu_width_ = 200;
    const std::string win_name_ = "SAD.MapViewer";
    const std::string dis_main_name_ = "main";
    const std::string dis_3d_name_ = "Cam3D";
    const std::string dis_3d_main_name_ = "Cam3DMain";

    bool following_loc_ = true;  // 相机是否追踪定位结果

    pangolin::OpenGlRenderState s_cam_main_;

    //////////////////////////////
    /// UI Objects
    //////////////////////////////

    std::shared_ptr<UiCloud> map_ui_;

    UiCar car_{Vec3f(0.2f, 0.2f, 0.8f)};
    std::shared_ptr<ui::UiTrajectory> traj_lidarloc_ui_ = nullptr;

};

}  // namespace sad::ui

#endif
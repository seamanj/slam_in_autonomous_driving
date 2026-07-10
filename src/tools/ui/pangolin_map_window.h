#ifndef SLAM_IN_AUTO_DRIVING_PANGOLIN_MAP_WINDOW_H
#define SLAM_IN_AUTO_DRIVING_PANGOLIN_MAP_WINDOW_H

#include <memory>

#include "common/point_types.h"

namespace sad::ui {

class PangolinMapWindowImpl;

class PangolinMapWindow {
public:
    PangolinMapWindow();
    ~PangolinMapWindow();

    bool Init();
    void Quit();

    void UpdateMap(CloudPtr cloud, const SE3& pose);

    bool ShouldQuit();

private:
    std::shared_ptr<PangolinMapWindowImpl> impl_;
};

}  // namespace sad::ui

#endif
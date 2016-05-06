#ifndef DYNAMIC_LAYER_H_
#define DYNAMIC_LAYER_H_
#include <ros/ros.h>
#include <map>
#include <string>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include <costmap_layers/Wall.h>

namespace costmap_layers
{

typedef struct WallInfo {
    unsigned int x1;
    unsigned int x2;
    unsigned int y1;
    unsigned int y2;
} WallInfo;

class VirtualWallLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
    VirtualWallLayer();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    virtual void onWallMessageCallback(const WallPtr& msg);

    bool isDiscretized()
    {
        return true;
    }

    virtual void matchSize();


private:

    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

    std::map<std::string, WallInfo> walls;

    void drawWallLine(int x1, int y1, int x2, int y2);

    void updateWalls();

    ros::Subscriber subscriber;
};

}
#endif

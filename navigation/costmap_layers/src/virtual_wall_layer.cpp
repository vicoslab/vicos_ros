#include <costmap_layers/virtual_wall_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_layers::VirtualWallLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace costmap_layers
{

VirtualWallLayer::VirtualWallLayer() {}

void VirtualWallLayer::onInitialize()
{
  ros::NodeHandle pnh("~/" + name_), nh;
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(pnh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &VirtualWallLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  subscriber = nh.subscribe ("walls", 100, &VirtualWallLayer::onWallMessageCallback, this);
}


void VirtualWallLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void VirtualWallLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void VirtualWallLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

}

void VirtualWallLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
        continue;
      master_grid.setCost(i, j, costmap_[index]); 
    }
  }
}


void VirtualWallLayer::onWallMessageCallback(const WallPtr& msg) {

    WallInfo wi;

    if(worldToMap(msg->from.x, msg->from.y, wi.x1, wi.y1) && worldToMap(msg->to.x, msg->to.y, wi.x2, wi.y2)){
        walls[msg->identifier] = wi;
        updateWalls();
    } else {
        walls.erase(msg->identifier);
    }

}

void VirtualWallLayer::updateWalls() {

    resetMap(0, 0, getSizeInCellsX(), getSizeInCellsY());

    for (std::map<std::string, WallInfo>::iterator it=walls.begin(); it!=walls.end(); ++it) {
        drawWallLine((int) (*it).second.x1, (int) (*it).second.y1, (int) (*it).second.x2, (int) (*it).second.y2);
    }
    
}

#define PUT_CELL(x, y) { if (x >= 0 && y >= 0 && x < getSizeInCellsX() && y < getSizeInCellsY()) costmap_[getIndex(x, y)] = LETHAL_OBSTACLE; }

void VirtualWallLayer::drawWallLine(int x1, int y1, int x2, int y2) {
    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;
    dx=x2-x1; dy=y2-y1;
    dx1=fabs(dx); dy1=fabs(dy);
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    if(dy1<=dx1) {
        if(dx>=0) {
            x=x1;
            y=y1;
            xe=x2;
        }
    else {
        x=x2;
        y=y2;
        xe=x1;
    }

    PUT_CELL(x, y);

  for(i=0;x<xe;i++)
  {
   x=x+1;
   if(px<0)
   {
    px=px+2*dy1;
   }
   else
   {
    if((dx<0 && dy<0) || (dx>0 && dy>0))
    {
     y=y+1;
    }
    else
    {
     y=y-1;
    }
    px=px+2*(dy1-dx1);
   }
    PUT_CELL(x, y);
  }
 }
 else
 {
  if(dy>=0)
  {
   x=x1;
   y=y1;
   ye=y2;
  }
  else
  {
   x=x2;
   y=y2;
   ye=y1;
  }
   PUT_CELL(x, y);
  for(i=0;y<ye;i++)
  {
   y=y+1;
   if(py<=0)
   {
    py=py+2*dx1;
   }
   else
   {
    if((dx<0 && dy<0) || (dx>0 && dy>0))
    {
     x=x+1;
    }
    else
    {
     x=x-1;
    }
    py=py+2*(dx1-dy1);
   }
    PUT_CELL(x, y);
  }
 }
}


} // end namespace

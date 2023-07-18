#include "lio_utils.h"

namespace zjloc
{
     state::state(const Eigen::Quaterniond &rotation_, const Eigen::Vector3d &translation_,
                  const Eigen::Vector3d &velocity_, const Eigen::Vector3d &ba_,
                  const Eigen::Vector3d &bg_)
         : rotation{rotation_}, translation{translation_}, velocity{velocity_}, ba{ba_}, bg{bg_}
     {
     }

     state::state(const state *state_temp, bool copy)
     {
          if (copy)
          {
               rotation = state_temp->rotation;
               translation = state_temp->translation;

               rotation_begin = state_temp->rotation_begin;
               translation_begin = state_temp->translation_begin;

               velocity = state_temp->velocity;
               ba = state_temp->ba;
               bg = state_temp->bg;

               velocity_begin = state_temp->velocity_begin;
               ba_begin = state_temp->ba_begin;
               bg_begin = state_temp->bg_begin;
          }
          else
          {
               rotation = state_temp->rotation;
               translation = state_temp->translation;

               rotation_begin = state_temp->rotation;
               translation_begin = state_temp->translation;

               velocity_begin = state_temp->velocity;
               ba_begin = state_temp->ba;
               bg_begin = state_temp->bg;

               velocity = state_temp->velocity;
               ba = state_temp->ba;
               bg = state_temp->bg;
          }
     }

     void state::release()
     {
     }

     //   --------------------------------------     //
     cloudFrame::cloudFrame(std::vector<point3D> &point_surf_, std::vector<point3D> &const_surf_,
                            state *p_state_)
     {
          point_surf.insert(point_surf.end(), point_surf_.begin(), point_surf_.end());
          const_surf.insert(const_surf.end(), const_surf_.begin(), const_surf_.end());

          // p_state = p_state_;
          p_state = new state(p_state_, true);

          success = true;
     }

     cloudFrame::cloudFrame(cloudFrame *p_cloud_frame)
     {
          time_frame_begin = p_cloud_frame->time_frame_begin;
          time_frame_end = p_cloud_frame->time_frame_end;

          frame_id = p_cloud_frame->frame_id;

          // p_state = p_cloud_frame->p_state;
          p_state = new state(p_cloud_frame->p_state, true);

          point_surf.insert(point_surf.end(), p_cloud_frame->point_surf.begin(), p_cloud_frame->point_surf.end());
          const_surf.insert(const_surf.end(), p_cloud_frame->const_surf.begin(), p_cloud_frame->const_surf.end());

          dt_offset = p_cloud_frame->dt_offset;

          success = p_cloud_frame->success;
     }

     void cloudFrame::release()
     {

          std::vector<point3D>().swap(point_surf);
          std::vector<point3D>().swap(const_surf);

          p_state = nullptr;
     }
}
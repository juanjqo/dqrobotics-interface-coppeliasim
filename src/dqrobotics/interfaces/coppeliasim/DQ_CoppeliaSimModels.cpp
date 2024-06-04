#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimModels.h>

namespace DQ_robotics
{

std::shared_ptr<DQ_CoppeliaSimInterface> DQ_CoppeliaSimModels::_get_interface_sptr() const
{
    return coppeliasim_interface_sptr_;
}

std::string DQ_CoppeliaSimModels::_get_string_from_others(const COMPONENTS &model)
{
    switch (model)
    {
    case COMPONENTS::REFERENCE_FRAME:
        return std::string("/other/reference frame.ttm");
    case COMPONENTS::ARUCO_MARKER:
        return std::string("/other/aruco marker.ttm");
    case COMPONENTS::PIONEER_P3DX:
        return std::string("/robots/mobile/pioneer p3dx.ttm");
    case COMPONENTS::GPS:
        return std::string("/components/sensors/GPS.ttm");
    case COMPONENTS::FRANKA_EMIKA_PANDA:
        return std::string("/robots/non-mobile/FrankaEmikaPanda.ttm");
    case COMPONENTS::JACO:
        return std::string("/robots/non-mobile/Jaco arm.ttm");
    case COMPONENTS::UR3:
        return std::string("/robots/non-mobile/UR3.ttm");
    case COMPONENTS::UR5:
        return std::string("/robots/non-mobile/UR5.ttm");
    case COMPONENTS::UR10:
        return std::string("/robots/non-mobile/UR10.ttm");
    }
}

DQ_CoppeliaSimModels::DQ_CoppeliaSimModels(const std::shared_ptr<DQ_CoppeliaSimInterface> &coppeliasim_interface_sptr)
    :coppeliasim_interface_sptr_(coppeliasim_interface_sptr)
{

}

void DQ_CoppeliaSimModels::_load_model(const COMPONENTS &model,
                                       const std::string &desired_model_name,
                                       const bool &load_model_only_if_missing,
                                       const bool &remove_child_script)
{
    _get_interface_sptr()->load_from_model_browser(_get_string_from_others(model),
                                                   desired_model_name,
                                                   load_model_only_if_missing,
                                                   remove_child_script);

}

void DQ_CoppeliaSimModels::load_model(const COMPONENTS &model,
                                      const std::string &desired_model_name,
                                      const DQ &pose,
                                      const bool &load_model_only_if_missing,
                                      const bool &remove_child_script)
{
    _load_model(model, desired_model_name, load_model_only_if_missing, remove_child_script);
    if (pose != DQ(0))
        _get_interface_sptr()->set_object_pose(desired_model_name, pose);
}



void DQ_CoppeliaSimModels::load_reference_frame(const std::string &desired_model_name,
                                                const DQ &pose,
                                                const bool &load_model_only_if_missing,
                                                const bool &remove_child_script)
{
    _load_model(COMPONENTS::REFERENCE_FRAME, desired_model_name, load_model_only_if_missing, remove_child_script);
    if (pose != DQ(0))
        _get_interface_sptr()->set_object_pose(desired_model_name, pose);
}

void DQ_CoppeliaSimModels::load_reference_frames(const std::vector<std::string> &desired_model_names, const std::vector<DQ> &poses)
{
    size_t n = desired_model_names.size();
    for (size_t i=0;i<n;i++)
        load_reference_frame(desired_model_names.at(i), poses.at(i), true, true);
}

void DQ_CoppeliaSimModels::load_reference_frames(const std::vector<std::string> &desired_model_names)
{
    for (auto& name : desired_model_names)
        load_reference_frame(name, DQ(1), true, true);
}

void DQ_CoppeliaSimModels::load_panda(const std::string &desired_model_name, const DQ &pose)
{
    load_model(COMPONENTS::FRANKA_EMIKA_PANDA, desired_model_name, pose, true, true);
}


}

/**
(C) Copyright 2024 DQ Robotics Developers

This file is based on DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Juan Jose Quiroz Omana
       - Responsible for the original implementation.

*/
#pragma once
#include <string>
#include <memory>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimZmqInterface.h>

namespace DQ_robotics
{
class DQ_CoppeliaSimModels
{
public:
    enum class COMPONENTS
    {
        REFERENCE_FRAME,
        ARUCO_MARKER,

        PIONEER_P3DX,

        GPS,

        FRANKA_EMIKA_PANDA,
        JACO,
        UR3,
        UR5,
        UR10,
    };

protected:
    std::shared_ptr<DQ_CoppeliaSimZmqInterface> coppeliasim_interface_sptr_;
    std::shared_ptr<DQ_CoppeliaSimZmqInterface> _get_interface_sptr() const;
    std::string _get_string_from_others(const COMPONENTS& model);

    void _load_model(const COMPONENTS& model,
                     const std::string& desired_model_name,
                     const bool& load_model_only_if_missing = true,
                     const bool& remove_child_script = true);
public:
    DQ_CoppeliaSimModels()=delete;
    DQ_CoppeliaSimModels(const std::shared_ptr<DQ_CoppeliaSimZmqInterface>& coppeliasim_interface_sptr);

    void load_model(const COMPONENTS& model,
                    const std::string& desired_model_name,
                    const DQ& pose = DQ(0),
                    const bool& load_model_only_if_missing = true,
                    const bool& remove_child_script = true);

    void load_reference_frame(const std::string& desired_model_name,
                              const DQ& pose = DQ(0),
                              const bool& load_model_only_if_missing = true,
                              const bool& remove_child_script = true);

    void load_reference_frames(const std::vector<std::string>& desired_model_names,
                               const std::vector<DQ>& poses);

    void load_reference_frames(const std::vector<std::string>& desired_model_names);

    void load_panda(const std::string& desired_model_name,
                    const DQ& pose = DQ(0));

    void load_primitive(const DQ_CoppeliaSimZmqInterface::PRIMITIVE& primitive,
                        const std::string& name = "shape",
                        const DQ& pose = DQ(1),
                        const std::vector<double> sizes = {0.2,0.2,0.2},
                        const std::vector<double> rgba_color = {1,0,0,1},
                        const bool& set_as_static = true,
                        const bool& set_as_respondable = false
                        );


};

}

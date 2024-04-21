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
*/

#pragma once
#include <dqrobotics/DQ.h>

using namespace DQ_robotics;
using namespace Eigen;

class DQ_CoppeliaSimInterface
{
private:
    bool _client_created = false;
public:
    DQ_CoppeliaSimInterface();
    void connect(const std::string& host = "localhost",
                 const int& rpcPort = 23000,
                 const int& cntPort = -1,
                 const int& verbose_ = -1);

    void start_simulation() const;
    void stop_simulation()  const;    
    void set_stepping_mode(const bool& flag);
    double get_simulation_time() const;
    void trigger_next_simulation_step() const;
    bool is_simulation_running() const;
    void set_status_bar_message(const std::string& message) const;

    //-----------Deprecated methods---------------------------//
    [[deprecated("This method is not required with ZeroMQ remote API.")]]
    void disconnect();
    [[deprecated("This method is not required with ZeroMQ remote API.")]]
    void disconnect_all();
    [[deprecated("The synchronous mode is now called stepping mode. Consider using set_stepping_mode(flag) instead.")]]
    void set_synchronous(const bool& flag);
    [[deprecated("This method is not required with ZeroMQ remote API.")]]
    int wait_for_simulation_step_to_end();
    //---------------------------------------------------------//





};



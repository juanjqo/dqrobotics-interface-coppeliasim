/**
(C) Copyright 2024 DQ Robotics Developers

This file is part of DQ Robotics.

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
#include <RemoteAPIClient.h>

using namespace DQ_robotics;
using namespace Eigen;

class DQ_CoppeliaSimInterface
{
public:
    DQ_CoppeliaSimInterface();

    bool connect(const std::string& host = "localhost",
                 const int& rpcPort = 23000,
                 const int& cntPort = -1,
                 const int& verbose_ = -1);

private:
    std::unique_ptr<RemoteAPIClient> client_;



    //[[deprecated]] bool connect(const int &port, const int& TIMEOUT_IN_MILISECONDS, const int& MAX_TRY_COUNT);
    //[[deprecated]] bool connect(const std::string& ip, const int& port, const int& TIMEOUT_IN_MILISECONDS, const int& MAX_TRY_COUNT);

};



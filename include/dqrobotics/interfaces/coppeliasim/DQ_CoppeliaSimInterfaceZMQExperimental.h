#pragma once
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>

class DQ_CoppeliaSimInterfaceZMQExperimental: public DQ_CoppeliaSimInterfaceZMQ
{
protected:
    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ::experimental> vi_exp_;
    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> vi_;
    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ::Impl> local_impl_;

public:
    DQ_CoppeliaSimInterfaceZMQExperimental();

    bool connect(const std::string& host = "localhost",
                 const int& port = 23000,
                 const int&TIMEOUT_IN_MILISECONDS = 300) override;

    void close_scene();

    void plot_reference_frame(const std::string& name,
                              const DQ& pose,
                              const double& scale = 1,
                              const std::vector<double>& thickness_and_length = {0.005, 0.1});
};


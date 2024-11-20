#include "_zmq_wrapper.h"

std::shared_ptr<RemoteAPIClient> _ZMQWrapper::client_;
std::shared_ptr<RemoteAPIObject::sim> _ZMQWrapper::sim_;

bool _ZMQWrapper::create_client(const std::string &host, const int &rpcPort, const int &cntPort, const int &verbose_)
{
    if(!client_)
        client_ = std::make_shared<RemoteAPIClient>(host, rpcPort, cntPort, verbose_);
    if(!sim_)
        sim_    = std::make_shared<RemoteAPIObject::sim>(client_->getObject().sim());

    return true;
}

std::shared_ptr<RemoteAPIObject::sim> _ZMQWrapper::get_sim()
{
    return sim_;
}

#pragma once
#include <RemoteAPIClient.h>

/**
 * @brief The _ZMQWrapper class
 * In this initial implementation, each process can only have one live connection with ZMQ.
 * More connections will require this singleton implementation to be changed to a map to accomodate it.
 *
 * Timeout will be added later
 */
class _ZMQWrapper
{
private:
    static std::shared_ptr<RemoteAPIClient> client_;
    static std::shared_ptr<RemoteAPIObject::sim> sim_;
public:

    static bool create_client(const std::string& host,
                              const int& rpcPort,
                              const int& cntPort = -1,
                              const int& verbose_ = -1);

    static std::shared_ptr<RemoteAPIObject::sim> get_sim();

};

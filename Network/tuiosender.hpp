#ifndef TUIOSENDER_H
#define TUIOSENDER_H

#include <vector>
#include <string>

//todo do not use boost
#include <boost/asio.hpp>

#include "Osc/OscOutboundPacketStream.h"
#include "hand.hpp"

class TUIOSender
{
public:
    TUIOSender(const std::string host, int port , const cv::Size resolution);
    ~TUIOSender();

    void send(const std::vector<std::shared_ptr<Hand> >& hands);
    void setResolution(const cv::Size& res);

    void setHost(const std::string host);


protected:
    /** \brief Boost network service*/
    boost::asio::io_service mIOService;
    /** \brief Socket used for data transmission*/
    boost::asio::ip::udp::socket * mSocket;
    /** \brief Stores network information of SoundDaemon*/
    boost::asio::ip::udp::endpoint mReceiverEndpoint;

    std::string mHost;
    int mPort;

    int mFrameID;
    char *mOSCBuffer;
    cv::Size mResolution;

};

#endif // TUIOSENDER_H

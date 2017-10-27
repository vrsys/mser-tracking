#include "tuiosender.hpp"

#define OUTPUT_BUFFER_SIZE 4096

TUIOSender::TUIOSender(const std::string host = "localhost", int port = 3333, const cv::Size resolution = cv::Size(0,0))
    : mHost(host),
      mPort(port),
      mFrameID(0),
      mResolution(resolution)
{
    try {
        mSocket = new boost::asio::ip::udp::socket(mIOService);
        mSocket->open(boost::asio::ip::udp::v4());

        boost::asio::ip::udp::resolver resolver(mIOService);
        boost::asio::ip::udp::resolver::query query(boost::asio::ip::udp::v4(), mHost.c_str(), "daytime");
        mReceiverEndpoint = *resolver.resolve(query);
        mReceiverEndpoint.port(mPort);
        std::cout << "TUIOSender: Successfully initialized network interface. " << mHost << "  " << mPort  << std::endl;
    } catch (std::exception& e)
    {
        mSocket = NULL;
        throw std::runtime_error(e.what());
    }

    mOSCBuffer = new char[ OUTPUT_BUFFER_SIZE ];
}

TUIOSender::~TUIOSender() {
    delete[] mOSCBuffer;
}

void TUIOSender::send(const std::vector<std::shared_ptr<Hand> > &hands)
{
    osc::OutboundPacketStream p( mOSCBuffer, OUTPUT_BUFFER_SIZE );

    try {
        // Alive Message for hands:
        p << osc::BeginBundleImmediate;

        p << osc::BeginMessage("/tuio/2Dcur") << "alive";
            for(const std::shared_ptr<Hand> h : hands) {
                for (const std::shared_ptr<Blob> b : h->mBlobs) {
                    p << (int)b->mID;
                }
            }
        p << osc::EndMessage;

        p << osc::BeginMessage("/tuiox/finger") << "alive";
            for(const std::shared_ptr<Hand> h : hands) {
                for (const std::shared_ptr<Blob> b : h->mBlobs) {
                    p << (int)b->mID;
                }
            }
        p << osc::EndMessage;

        p << osc::BeginMessage("/tuiox/hand") << "alive";
            for(const std::shared_ptr<Hand> h : hands) {
                if(h->mID >= 0 && h->mBlobs.size() > 0){
                    p << (int)h->mID;
                }
            }
        p << osc::EndMessage;

        for(const std::shared_ptr<Hand> h : hands) {
            for (const std::shared_ptr<Blob> b : h->mBlobs) {
                float tempX = (((float)b->mFilteredPosition.x + 2) / ((float)mResolution.width));
                float tempY = ((((float)b->mFilteredPosition.y + 20 )/ ((float)mResolution.height + 35)) );

                p << osc::BeginMessage("/tuio/2Dcur")
                  << "set"
                  << (int)b->mID
                  << tempX
                  << tempY
                  << (float)1.
                  << (float)1.
                  << (float)1.
                << osc::EndMessage;

                p << osc::BeginMessage("/tuiox/finger")
                  << "set"
                  << (int)b->mID
                  << tempX
                  << tempY
                  << (int)b->mHand->mID
                  << (int)b->mClass
                << osc::EndMessage;
            }
        }

        for(const std::shared_ptr<Hand> h : hands) {
            if(h->mID >= 0){
                p << osc::BeginMessage("/tuiox/hand")
                  << "set"
                  << (int)h->mID
                  << (float)h->mFilteredPosition.x / (float)mResolution.width
                  << (float)h->mFilteredPosition.y / (float)mResolution.height;

                // send associated blob id's for each hand
                for (unsigned int i = 0; i < 5; ++i) {
                    if (i < h->mBlobs.size()){
                        p << (int) h->mBlobs[i]->mID;
                    } else {
                        p << -1;
                    }
                }

                p << h->mClass;

                p << h->mArmEllipse.mPosition.x
                  << h->mArmEllipse.mPosition.y
                  << h->mArmEllipse.mMinor_axis
                  << h->mArmEllipse.mMajor_axis
                  << h->mArmEllipse.mInclination;

                p << osc::EndMessage;
            }
        }

        p << osc::BeginMessage("/tuio/2Dcur")
            << "fseq" << mFrameID
        << osc::EndMessage;

        p << osc::EndBundle;

        mSocket->send_to(boost::asio::buffer(p.Data(), p.Size()), mReceiverEndpoint);

    } catch( osc::Exception e ) {
        std::cerr << "too many objects" << e.what()<< std::endl;
    }

    ++mFrameID;
}

void TUIOSender::setResolution(const cv::Size& res)
{
    mResolution = res;
}

void TUIOSender::setHost(const std::string host)
{
    try {
        mHost = host;

        mSocket = new boost::asio::ip::udp::socket(mIOService);
        mSocket->open(boost::asio::ip::udp::v4());

        boost::asio::ip::udp::resolver resolver(mIOService);
        boost::asio::ip::udp::resolver::query query(boost::asio::ip::udp::v4(), mHost.c_str(), "daytime");
        mReceiverEndpoint = *resolver.resolve(query);
        mReceiverEndpoint.port(mPort);
        std::cout << "TUIOSender: Successfully initialized network interface. " << mHost << "  " << mPort  << std::endl;
    } catch (std::exception& e)
    {
        mSocket = NULL;
        throw std::runtime_error(e.what());
    }
}

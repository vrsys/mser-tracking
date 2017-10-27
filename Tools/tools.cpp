#include "Tools/tools.hpp"

int
getFileNames (std::string const& dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    //Unable to open dir
    if((dp = opendir(dir.c_str())) == NULL)
    {
        std::cout << "Error(" << errno << ") opening " << dir << std::endl;
        return errno;
    }
    //read files and push them to vector
    while ((dirp = readdir(dp)) != NULL)
    {
        std::string name = std::string(dirp->d_name);
        std::regex rex;
        rex = ".+(\\.jpg|\\.png|\\.JPG|\\.PNG)";
        //discard . and .. from list
        if(std::regex_match(name, rex))
        {
            files.push_back(std::string(dirp->d_name));
        }
    }
    closedir(dp);
    std::sort(files.begin(), files.end());
    return 0;
}


float clamp(float value, float min, float max)
{
    return std::max(min, std::min(value, max));
}

QImage matToQImage(const cv::Mat &inMat)
{
    switch ( inMat.type() )
    {
       // 8-bit, 4 channel
       case CV_8UC4:
       {
          QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_RGB32 );

          QImage copy(image);
          copy.bits(); //enforce deep copy
          return copy;
       }

       // 8-bit, 3 channel
       case CV_8UC3:
       {
          QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_RGB888 );

          return image.rgbSwapped();
       }

       // 8-bit, 1 channel
       case CV_8UC1:
       {
          static QVector<QRgb>  sColorTable;

          // only create our color table once
          if ( sColorTable.isEmpty() )
          {
             for ( int i = 0; i < 256; ++i )
                sColorTable.push_back( qRgb( i, i, i ) );
          }

          QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_Indexed8 );
          image.setColorTable( sColorTable );

          QImage copy(image);
          copy.bits(); //enforce deep copy
          return copy;
       }

       default:
          qWarning() << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type();
          break;
    }

    return QImage();
}

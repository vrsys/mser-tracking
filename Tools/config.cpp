#include "config.hpp"

Config::Config(std::string configFile)
    : mConfigFile(configFile),
      mFileIn(configFile)
{
}

std::string Config::getValue(std::string parameter, bool& success)
{
    mFileIn.seekg(std::ios::beg);
    std::string line;
    while (std::getline(mFileIn, line)) {
        if (line.find(parameter) != std::string::npos) {
            std::istringstream sin(line.substr(line.find(" ") + 1));
            return sin.str();
        }
    }
    success = false;
    return "";
}

void Config::setValue(std::string parameter, std::string value)
{
    std::ofstream tmp(mConfigFile+"Temp"); // create and write to file
    bool ok = tmp.good();
    if (tmp.is_open()) {
        tmp.close();
    }

    //bool ok = std::ofstream(mConfigFile+"Temp"); // create and write to file
    if(!ok) { std::perror("Error creating temp config"); return; }

    mFileIn.close();
    mFileIn.open(mConfigFile);
    mFileOut.open(mConfigFile+"Temp");

    std::string line;
    while(std::getline(mFileIn, line))
    {
        size_t pos = line.find(parameter);
        if(pos != std::string::npos){
            std::string newLine = parameter + " " + value ;
            line = newLine;
        }
        mFileOut << line << std::endl;
    }

    mFileIn.close();
    mFileOut.close();

    int rc = std::rename((mConfigFile+"Temp").c_str() , (mConfigFile).c_str());
    if(rc) { std::perror("Error renaming"); return; }
}

bool Config::successLoad()
{
    return mFileIn.good();
}

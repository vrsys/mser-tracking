#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

class Config
{
public:
    Config(std::string configFile);
    std::string getValue(std::string parameter, bool& success);
    void setValue(std::string parameter, std::string value);

    bool successLoad();

private:
    std::string mConfigFile;

    std::ifstream mFileIn;
    std::fstream mFileOut;
};

#endif // CONFIG_H

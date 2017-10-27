#ifndef CLASSIFICATION_H
#define CLASSIFICATION_H

#include <vector>
#include <memory>

#include "hand.hpp"

class Classification
{
public:
    Classification();
    void process(std::vector<std::shared_ptr<Hand> > &hands);
};

#endif // CLASSIFICATION_H

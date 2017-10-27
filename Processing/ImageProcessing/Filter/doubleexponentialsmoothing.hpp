#ifndef DOUBLEEXPONENTIALSMOOTHING_H
#define DOUBLEEXPONENTIALSMOOTHING_H

#define ALPHA 0.5f

#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>

typedef struct SmoothingStatistics {
    double Sp;
    double Sp2;
    unsigned char state;
    float alpha;
} SmoothingStatistics;

typedef struct SmoothedPoint2D {
    std::shared_ptr<SmoothingStatistics> x;
    std::shared_ptr<SmoothingStatistics> y;
} SmoothedPoint2D;



inline void updateModel (std::shared_ptr<SmoothingStatistics> s, const double value)
{
    if (s->state == 2)
    {
        s->Sp = s->alpha * value + (1 - s->alpha) * s->Sp;
        s->Sp2 = s->alpha * s->Sp + (1 - s->alpha) * s->Sp2;
    }
    else if (s->state == 1)
    {
        s->Sp = s->alpha * value + (1 - s->alpha) * s->Sp;
        s->Sp2 = s->alpha * s->Sp + (1 - s->alpha) * s->Sp2;
        //s->Sp2 = s->Sp;
        s->state++;
    }
    else if (s->state == 0)
    {
        s->Sp = value;
        s->Sp2 = value;
        s->state++;
    }

    //std::cout << "Update: " << s->Sp << " " << s->Sp2 << " " << s->state << std::endl;
}

inline int getPredictedValue(const std::shared_ptr<SmoothingStatistics> s, unsigned int delta = 1)
{
    if (s->state == 2)
    {
        return (2.0f + (float)delta * s->alpha / (1.0f - s->alpha)) * s->Sp - (1.0f + (float)delta * s->alpha / (1.0f - s->alpha)) * s->Sp2;
    }
    else if (s->state == 1)
    {
        return s->Sp;
    }

    return 0.;
}

inline void initSmoothingStatistics (std::shared_ptr<SmoothingStatistics> s) {
    s->state = 0;
    s->alpha = ALPHA;
}

inline void initSmoothedPoint2D (std::shared_ptr<SmoothedPoint2D> p)
{
    initSmoothingStatistics(p->x);
    initSmoothingStatistics(p->y);
}

inline void setSmoothingStatisticsAlpha (std::shared_ptr<SmoothingStatistics> s, float alpha) {
    s->alpha = alpha;
}

inline void setSmoothedPoint2DAlpha (const std::shared_ptr<SmoothedPoint2D> p, float alpha)
{
    setSmoothingStatisticsAlpha(p->x, alpha);
    setSmoothingStatisticsAlpha(p->y, alpha);
}

inline void updateSmoothedPoint2D (const std::shared_ptr<SmoothedPoint2D> p, const cv::Point2i& p1) {
    updateModel(p->x, p1.x);
    updateModel(p->y, p1.y);
}

inline bool predictSmoothedPoint2D (const std::shared_ptr<SmoothedPoint2D> p, cv::Point2i& p1, unsigned int delta = 1) {
    if ((p->x->state) && (p->y->state)) {
        p1.x = getPredictedValue(p->x, delta);
        p1.y = getPredictedValue(p->y, delta);

        return true;
    }

    return false;
}


#endif // DOUBLEEXPONENTIALSMOOTHING_H

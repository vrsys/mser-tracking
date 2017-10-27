//--------------------------------------------------------------------------------------------------
// Linear time Maximally Stable Extremal Regions implementation as described in D. Nistér and
// H. Stewénius. Linear Time Maximally Stable Extremal Regions. Proceedings of the European
// Conference on Computer Vision (ECCV), 2008.
// 
// Copyright (c) 2011 Idiap Research Institute, http://www.idiap.ch/.
// Written by Charles Dubout <charles.dubout@idiap.ch>.
// 
// MSER is free software: you can redistribute it and/or modify it under the terms of the GNU
// General Public License version 3 as published by the Free Software Foundation.
// 
// MSER is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
// 
// You should have received a copy of the GNU General Public License along with MSER. If not, see
// <http://www.gnu.org/licenses/>.
//--------------------------------------------------------------------------------------------------

#include "mser.hpp"

#include <algorithm>
#include <cassert>
#include <limits>

using namespace std;

MSER::Region::Region(int level, int pixel) : level_(level), pixel_(pixel), area_(0),
variation_(numeric_limits<double>::infinity()), parent_(), child_(nullptr), next_(nullptr)
{
	fill_n(moments_, 5, 0.0);
}

inline void MSER::Region::accumulate(int x, int y)
{
	++area_;
	moments_[0] += x;
	moments_[1] += y;
	moments_[2] += x * x;
	moments_[3] += x * y;
	moments_[4] += y * y;
}

void MSER::Region::merge(std::shared_ptr<Region> child)
{
    assert(!child->parent_.lock());
	assert(!child->next_);
	
	// Add the moments together
	area_ += child->area_;
	moments_[0] += child->moments_[0];
	moments_[1] += child->moments_[1];
	moments_[2] += child->moments_[2];
	moments_[3] += child->moments_[3];
	moments_[4] += child->moments_[4];
	
	child->next_ = child_;
	child_ = child;
    child->parent_ = shared_from_this();
}

MSER::MSER()
    : eight_(false),
      pool_(256), poolIndex_(0)
{
}

void MSER::process(const uint8_t * bits, int width, int height)
{
	// 1. Clear the accessible pixel mask, the heap of boundary pixels and the component stack. Push
	// a dummy-component onto the stack, with grey-level higher than any allowed in the image.
	vector<bool> accessible(width * height);
	vector<int> boundaryPixels[256];
	int priority = 256;
    vector<std::shared_ptr<Region>> regionStack;

    regionStack.push_back(std::make_shared<Region> ());
	
	// 2. Make the source pixel (with its first edge) the current pixel, mark it as accessible and
	// store the grey-level of it in the variable current level.
	int curPixel = 0;
	int curEdge = 0;
	int curLevel = bits[0];
	accessible[0] = true;
	
	// 3. Push an empty component with current level onto the component stack.
step_3:
    regionStack.push_back(std::make_shared<Region> (curLevel, curPixel));
	
	if (poolIndex_ == pool_.size())
		doublePool(regionStack);
	
	// 4. Explore the remaining edges to the neighbors of the current pixel, in order, as follows:
	// For each neighbor, check if the neighbor is already accessible. If it is not, mark it as
	// accessible and retrieve its grey-level. If the grey-level is not lower than the current one,
	// push it onto the heap of boundary pixels. If on the other hand the grey-level is lower than
	// the current one, enter the current pixel back into the queue of boundary pixels for later
	// processing (with the next edge number), consider the new pixel and its grey-level and go to 3.
	for (;;) {
		const int x = curPixel % width;
		const int y = curPixel / width;
		
        for (; curEdge < (eight_ ? 8 : 4); ++curEdge) {
			int neighborPixel = curPixel;
			
			if (eight_) {
				switch (curEdge) {
					case 0: if (x < width - 1) neighborPixel = curPixel + 1; break;
					case 1: if ((x < width - 1) && (y > 0)) neighborPixel = curPixel - width + 1; break;
					case 2: if (y > 0) neighborPixel = curPixel - width; break;
					case 3: if ((x > 0) && (y > 0)) neighborPixel = curPixel - width - 1; break;
					case 4: if (x > 0) neighborPixel = curPixel - 1; break;
					case 5: if ((x > 0) && (y < height - 1)) neighborPixel = curPixel + width - 1; break;
					case 6: if (y < height - 1) neighborPixel = curPixel + width; break;
					default: if ((x < width - 1) && (y < height - 1)) neighborPixel = curPixel + width + 1; break;
				}
			}
			else {
				switch (curEdge) {
					case 0: if (x < width - 1) neighborPixel = curPixel + 1; break;
					case 1: if (y < height - 1) neighborPixel = curPixel + width; break;
					case 2: if (x > 0) neighborPixel = curPixel - 1; break;
					default: if (y > 0) neighborPixel = curPixel - width; break;
				}
			}
			
			if (neighborPixel != curPixel && !accessible[neighborPixel]) {
				const int neighborLevel = bits[neighborPixel];
				accessible[neighborPixel] = true;
				
				if (neighborLevel >= curLevel) {
					boundaryPixels[neighborLevel].push_back(neighborPixel << 4);
					
					if (neighborLevel < priority)
						priority = neighborLevel;
				}
				else {
					boundaryPixels[curLevel].push_back((curPixel << 4) | (curEdge + 1));
					
					if (curLevel < priority)
						priority = curLevel;
					
					curPixel = neighborPixel;
					curEdge = 0;
					curLevel = neighborLevel;
					
					goto step_3;
				}
			}
		}
		
		// 5. Accumulate the current pixel to the component at the top of the stack (water
		// saturates the current pixel).
		regionStack.back()->accumulate(x, y);
		
		// 6. Pop the heap of boundary pixels. If the heap is empty, we are done. If the returned
		// pixel is at the same grey-level as the previous, go to 4.
		if (priority == 256) {
            unstableRoot_ = regionStack.back();
			poolIndex_ = 0;

			return;
		}
		
		curPixel = boundaryPixels[priority].back() >> 4;
		curEdge = boundaryPixels[priority].back() & 15;
		
		boundaryPixels[priority].pop_back();
		
		while (boundaryPixels[priority].empty() && (priority < 256))
			++priority;
		
		const int newPixelGreyLevel = bits[curPixel];
		
		if (newPixelGreyLevel != curLevel) {
			curLevel = newPixelGreyLevel;
			
			// 7. The returned pixel is at a higher grey-level, so we must now process
			// all components on the component stack until we reach the higher
			// grey-level. This is done with the processStack sub-routine, see below.
			// Then go to 4.
			processStack(newPixelGreyLevel, curPixel, regionStack);
		}
	}
}

void MSER::process(cv::Mat const& img)
{
    uint8_t* pxl = (uint8_t*)img.data;
    process(pxl, img.cols, img.rows);
}

std::shared_ptr<MSER::Region> MSER::getUnstableRoot(const cv::Mat& img)
{
    //regions_.clear();
    uint8_t* pxl = (uint8_t*)img.data;
    process(pxl, img.cols, img.rows);

    while(unstableRoot_->parent_.lock()){
        unstableRoot_ = unstableRoot_->parent_.lock();
    }

    return unstableRoot_;
}

void MSER::processStack(int newPixelGreyLevel, int pixel, vector<std::shared_ptr<Region>> & regionStack)
{
    // 1. Process component on the top of the stack. The next grey-level is the minimum of
	// newPixelGreyLevel and the grey-level for the second component on the stack.
	do {
        std::shared_ptr<Region> top = regionStack.back();
		
		regionStack.pop_back();
		
		// 2. If newPixelGreyLevel is smaller than the grey-level on the second component on the
		// stack, set the top of stack grey-level to newPixelGreyLevel and return from sub-routine
		// (This occurs when the new pixel is at a grey-level for which there is not yet a component
		// instantiated, so we let the top of stack be that level by just changing its grey-level.
		if (newPixelGreyLevel < regionStack.back()->level_) {
            //std::shared_ptr<MSER::Region>(new (&pool_[poolIndex_++]) Region(newPixelGreyLevel, pixel))
            regionStack.push_back(std::make_shared<Region> (newPixelGreyLevel, pixel));
			
            if (poolIndex_ == pool_.size())
                top = std::shared_ptr<MSER::Region>(reinterpret_cast<MSER::Region*>(reinterpret_cast<char *>(top.get()) +
                                                                 doublePool(regionStack)));
			
			regionStack.back()->merge(top);
			
			return;
		}
		
		// 3. Remove the top of stack and merge it into the second component on stack as follows:
		// Add the first and second moment accumulators together and/or join the pixel lists.
		// Either merge the histories of the components, or take the history from the winner. Note
		// here that the top of stack should be considered one ’time-step’ back, so its current
		// size is part of the history. Therefore the top of stack would be the winner if its
		// current size is larger than the previous size of second on stack.
		regionStack.back()->merge(top);
	}
	// 4. If(newPixelGreyLevel>top of stack grey-level) go to 1.
	while (newPixelGreyLevel > regionStack.back()->level_);
}

std::ptrdiff_t MSER::doublePool(vector<std::shared_ptr<MSER::Region>> & regionStack)
{
    assert(!pool_.empty()); // Cannot double the size of an empty pool

    vector<Region> newPool(pool_.size() * 2);
    copy(pool_.begin(), pool_.end(), newPool.begin());

    // Cast to char in case the two pointers do not share the same alignment
    const ptrdiff_t offset = reinterpret_cast<char *>(&newPool[0]) -
                                  reinterpret_cast<char *>(&pool_[0]);

    for (size_t i = 0; i < pool_.size(); ++i) {
        if (newPool[i].parent_.lock())
            newPool[i].parent_.lock() =
                std::shared_ptr<MSER::Region>(reinterpret_cast<MSER::Region*>(reinterpret_cast<char *>(newPool[i].parent_.lock().get()) + offset));

        if (newPool[i].child_)
            newPool[i].child_ =
                std::shared_ptr<MSER::Region>(reinterpret_cast<MSER::Region*>(reinterpret_cast<char *>(newPool[i].child_.get()) + offset));

        if (newPool[i].next_)
            newPool[i].next_ =
                std::shared_ptr<MSER::Region>(reinterpret_cast<MSER::Region*>(reinterpret_cast<char *>(newPool[i].next_.get()) + offset));
    }

    for (size_t i = 0; i < regionStack.size(); ++i)
        regionStack[i] =
            std::shared_ptr<MSER::Region>(reinterpret_cast<MSER::Region*>(reinterpret_cast<char *>(regionStack[i].get()) + offset));

    pool_.swap(newPool);

    return offset;
}

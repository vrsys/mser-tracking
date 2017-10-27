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

#ifndef MSER_H
#define MSER_H

#include <iostream>
#include <vector>
#include <memory>
#include <cstdint>

#include <opencv2/core/core.hpp>


/// The MSER class extracts maximally stable extremal regions from a grayscale (8 bits) image.
/// @note The MSER class is not reentrant, so if you want to extract regions in parallel, each
/// thread needs to have its own MSER class instance.
class MSER
{
public:
	/// A Maximally Stable Extremal Region.
	struct Region
        : public std::enable_shared_from_this<MSER::Region>
	{
		int level_; ///< Level at which the region is processed.
		int pixel_; ///< Index of the initial pixel (y * width + x).
		int area_; ///< Area of the region (moment zero).
		double moments_[5]; ///< First and second moments of the region (x, y, x^2, xy, y^2).
		double variation_; ///< MSER variation.
		
		/// Constructor.
		/// @param[in] level Level at which the region is processed.
		/// @param[in] pixel Index of the initial pixel (y * width + x).
		Region(int level = 256, int pixel = 0);
		
        // Implementation details (could be moved outside this header file)
        std::weak_ptr<Region> parent_; // Pointer to the parent region
        std::shared_ptr<Region> child_; // Pointer to the first child
        std::shared_ptr<Region> next_; // Pointer to the next (sister) region

    private:
		void accumulate(int x, int y);
        void merge(std::shared_ptr<Region> child);
		
		friend class MSER;
	};

	/// Constructor.
	/// @param[in] eight Use 8-connected pixels instead of 4-connected.
    MSER();

	
	/// Extracts maximally stable extremal regions from a grayscale (8 bits) image.
	/// @param[in] bits Pointer to the first scanline of the image.
	/// @param[in] width Width of the image.
	/// @param[in] height Height of the image.
    void process(const uint8_t * bits, int width, int height);

    /// Extracts maximally stable extremal regions from a grayscale (8 bits) image.
    /// @param[in] cv::Mat image.
    void process(cv::Mat const& img);

    /// Extracts all extremal regions from a grayscale (8 bits) image.
    /// @param[in] cv::Mat image.
    /// @param[out] (unstable) root of all regions.
    std::shared_ptr<Region> getUnstableRoot(cv::Mat const& img);

	
	// Implementation details (could be moved outside this header file)

private:
    // Helper method
    void processStack(int newPixelGreyLevel, int pixel, std::vector<std::shared_ptr<Region>> & regionStack);
	
	// Double the size of the memory pool
    std::ptrdiff_t doublePool(std::vector<std::shared_ptr<Region>> & regionStack);

    // parameter
    bool eight_;

    // ComponentTree
    std::shared_ptr<Region> unstableRoot_;

    // Memory pool of regions for faster allocation
    std::vector<Region> pool_;
    std::size_t poolIndex_;
};

#endif

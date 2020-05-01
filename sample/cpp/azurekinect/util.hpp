#ifndef __UTIL__
#define __UTIL__

#include <stdexcept>
#include <sstream>
#include <string>
#include <memory>

#include <cubemos/skeleton_tracking.h>

#define MULTIPLE 16

#define CHECK_SUCCESS( ret )                                                \
    if( ret != CM_ReturnCode::CM_SUCCESS ){                                 \
        std::stringstream ss;                                               \
        ss << "failed to " #ret " " << std::hex << ret << "!" << std::endl; \
        throw std::runtime_error( ss.str().c_str() );                       \
    }

using CUBEMOS_SKEL_Buffer_Ptr = std::unique_ptr<CM_SKEL_Buffer, void (*)( CM_SKEL_Buffer* )>;
CUBEMOS_SKEL_Buffer_Ptr create_skel_buffer();

/*
 This is utility to that provides converter to convert k4a::image to cv::Mat.

 cv::Mat mat = k4a::get_mat( image );

 Copyright (c) 2019 Tsukasa Sugiura <t.sugiura0204@gmail.com>
 Licensed under the MIT license.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
*/

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>

namespace k4a{
    cv::Mat get_mat( k4a::image& src, bool deep_copy = true );
}

cv::Mat k4a_get_mat( k4a_image_t& src, bool deep_copy = true );

#endif // __UTIL__
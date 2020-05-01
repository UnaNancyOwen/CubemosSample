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

using CUBEMOS_SKEL_Buffer_Ptr = std::unique_ptr<CM_SKEL_Buffer, void ( * )( CM_SKEL_Buffer* )>;
CUBEMOS_SKEL_Buffer_Ptr create_skel_buffer();

#endif // __UTIL__
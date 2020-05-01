#include "util.hpp"

CUBEMOS_SKEL_Buffer_Ptr create_skel_buffer()
{
    return CUBEMOS_SKEL_Buffer_Ptr( new CM_SKEL_Buffer(), []( CM_SKEL_Buffer* pb ){ cm_skel_release_buffer( pb ); delete pb; } );
}

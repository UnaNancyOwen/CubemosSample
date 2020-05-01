#include <iostream>
#include <sstream>

#include "realsense.hpp"

int main( int argc, char* argv[] )
{
    try{
        realsense realsense;
        realsense.run();
    }
    catch( const rs2::error& error ){
        std::cout << error.what() << std::endl;
        return -1;
    }
    catch( std::exception& ex ){
        std::cout << ex.what() << std::endl;
        return -1;
    }

    return 0;
}
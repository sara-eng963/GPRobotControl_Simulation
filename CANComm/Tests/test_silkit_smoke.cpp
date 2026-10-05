#include <silkit/SilKit.hpp>
#include <silkit/SilKitVersion.hpp>

#include <iostream>

int main()
{
    std::cout
        << "SIL Kit version: "
        << SilKit::Version::String()
        << '\n';

    return 0;
}
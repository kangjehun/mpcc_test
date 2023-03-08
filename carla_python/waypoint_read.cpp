#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

int main()
{
    std::ifstream infile("waypoints.txt");
    std::string line;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        std::string token;
        int index;
        double x, y, z;
        iss >> token >> index >> token >> x >> token >> y >> token >> z;
        std::cout << index << " " << x << " " << y << " " << z << " " << std::endl;
    }
    infile.close();
    return 0;
}
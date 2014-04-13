#include "ObjectGrowing.hpp"

using namespace envire;
using envire::Grid;

ENVIRONMENT_ITEM_DEF( ObjectGrowingUINT8 );


bool envire::ObjectGrowingUINT8::updateAll()
{
    return envire::Operator::updateAll();
}


struct RadialLUT
{
    int centerx, centery;
    unsigned int width, height;
    boost::multi_array<std::pair<int, int>, 2>  parents;
    boost::multi_array<bool, 2> in_distance;

    void precompute(double distance, double scalex, double scaley)
    {
        double const radius2 = distance * distance;

        width  = 2* ceil(distance / scalex) + 1;
        height = 2* ceil(distance / scaley) + 1;
        in_distance.resize(boost::extents[height][width]);
        std::fill(in_distance.data(), in_distance.data() + in_distance.num_elements(), false);
        parents.resize(boost::extents[height][width]);
        std::fill(parents.data(), parents.data() + parents.num_elements(), std::make_pair(-1, -1));

        centerx = width  / 2;
        centery = height / 2;
        parents[centery][centerx] = std::make_pair(-1, -1);

        for (unsigned int y = 0; y < height; ++y)
        {
            for (unsigned int x = 0; x < width; ++x)
            {
                int dx = (centerx - x);
                int dy = (centery - y);
                if (dx == 0 && dy == 0) continue;

                double d2 = dx * dx * scalex * scalex + dy * dy * scaley * scaley;
                in_distance[y][x] = (d2 < radius2);
                if (abs(dx) > abs(dy))
                {
                    int parentx = x + dx / abs(dx);
                    int parenty = y + rint(static_cast<double>(dy) / abs(dx));
                    parents[y][x] = std::make_pair(parentx, parenty);
                }
                else
                {
                    int parentx = x + rint(static_cast<double>(dx) / abs(dy));
                    int parenty = y + dy / abs(dy);
                    parents[y][x] = std::make_pair(parentx, parenty);
                }
            }
        }
    }

    void markAllRadius(boost::multi_array<uint8_t, 2>& result, int result_width, int result_height, int centerx, int centery, int value)
    {
        int base_x = centerx - this->centerx;
        int base_y = centery - this->centery;
        for (unsigned int y = 0; y < height; ++y)
        {
            int map_y = base_y + y;
            if (map_y < 0 || map_y >= result_height)
                continue;

            for (unsigned int x = 0; x < width; ++x)
            {
                int map_x = base_x + x;
                if (map_x < 0 || map_x >= result_width)
                    continue;
                if (in_distance[y][x] && result[map_y][map_x] == value)
                {
//                     LOG_DEBUG("  found cell with value %i (expected %i) at %i %i, marking radius", result[map_y][map_x], value, map_x, map_y);
                    markSingleRadius(result, centerx, centery, x, y, value, 255);
                }
            }
        }
    }

    void markSingleRadius(boost::multi_array<uint8_t, 2>& result, int centerx, int centery, int x, int y, int expected_value, int mark_value)
    {
        boost::tie(x, y) = parents[y][x];
        while (x != -1 && y != -1)
        {
            int map_x = centerx + x - this->centerx;
            int map_y = centery + y - this->centery;
            uint8_t& current = result[map_y][map_x];
            if (current != expected_value)
            {
                current = mark_value;
//              LOG_DEBUG("  marking %i %i", map_x, map_y);
            }
            boost::tie(x, y) = parents[y][x];
        }
    }
};

// void envire::ObjectGrowing::growObjects(Grid< uint8_t >& map, const std::string& band_name, int objectValue, double width)
// {
//     const double width_square = pow(width,2);
//     const int 
//         wx = width / map.getScaleX(), 
//         wy = width / map.getScaleY();
//     const double 
//         sx = map.getScaleX(),
//         sy = map.getScaleY();
// 
// 
//     OutputLayer::ArrayType& orig_data = band_name.empty() ?
//         map.getGridData() :
//         map.getGridData(output_band);
// 
//     OutputLayer::ArrayType data( orig_data );
// 
//     for (unsigned int y = 0; y < map.getCellSizeX(); ++y)
//     {
//         for (unsigned int x = 0; x < map.getCellSizeX(); ++x)
//         {
//             int value = orig_data[y][x];
//             if (value == objectValue)
//             {
//                 // make everything with radius width around the obstacle also
//                 // an obstacle
//                 for( int oy = -wy; oy <= wy; ++oy )
//                 {
//                     for( int ox = -wx; ox <= wx; ++ox )
//                     {
//                         const int tx = x+ox;
//                         const int ty = y+oy;
//                         if( (pow(ox*sx,2) + pow(oy*sy,2) < width_square )
//                                 && tx >= 0 && tx < (int)map.getWidth()
//                                 && ty >= 0 && ty < (int)map.getHeight() )
//                             data[ty][tx] = CLASS_OBSTACLE;
//                     }
//                 }
//             }
//         }
//     }
// 
//     std::swap( data, orig_data );
//     
// }


// void SimpleTraversability::growObstacles(OutputLayer& map, std::string const& band_name, double width)
// {
//     const double width_square = pow(width,2);
//     const int 
//         wx = width / map.getScaleX(), 
//         wy = width / map.getScaleY();
//     const double 
//         sx = map.getScaleX(),
//         sy = map.getScaleY();
// 
// 
//     OutputLayer::ArrayType& orig_data = band_name.empty() ?
//         map.getGridData() :
//         map.getGridData(output_band);
// 
//     OutputLayer::ArrayType data( orig_data );
// 
//     for (unsigned int y = 0; y < map.getHeight(); ++y)
//     {
//         for (unsigned int x = 0; x < map.getWidth(); ++x)
//         {
//             int value = orig_data[y][x];
//             if (value == CLASS_OBSTACLE)
//             {
//                 // make everything with radius width around the obstacle also
//                 // an obstacle
//                 for( int oy = -wy; oy <= wy; ++oy )
//                 {
//                     for( int ox = -wx; ox <= wx; ++ox )
//                     {
//                         const int tx = x+ox;
//                         const int ty = y+oy;
//                         if( (pow(ox*sx,2) + pow(oy*sy,2) < width_square )
//                                 && tx >= 0 && tx < (int)map.getWidth()
//                                 && ty >= 0 && ty < (int)map.getHeight() )
//                             data[ty][tx] = CLASS_OBSTACLE;
//                     }
//                 }
//             }
//         }
//     }
// 
//     std::swap( data, orig_data );
// }
// 
// void SimpleTraversability::closeNarrowPassages(SimpleTraversability::OutputLayer& map, std::string const& band_name, double min_width)
// {
//     RadialLUT lut;
//     lut.precompute(min_width, map.getScaleX(), map.getScaleY());
//     std::stringstream oss;
//     oss << std::endl;
//     for (unsigned int y = 0; y < lut.height; ++y)
//     {
//         for (unsigned int x = 0; x < lut.width; ++x) {
//             oss << "(" << lut.parents[y][x].first << " " << lut.parents[y][x].second << ") ";
//         }
//         oss << std::endl;
//     }
//     oss << std::endl;
//     for (unsigned int y = 0; y < lut.height; ++y)
//     {
//         for (unsigned int x = 0; x < lut.width; ++x) {
//             oss << "(" << lut.in_distance[y][x] << " ";
//         }
//         oss << std::endl;
//     }
//     LOG_DEBUG(oss.str().c_str());
// 
//     OutputLayer::ArrayType& data = band_name.empty() ?
//         map.getGridData() :
//         map.getGridData(output_band);
//     for (unsigned int y = 0; y < map.getHeight(); ++y)
//     {
//         for (unsigned int x = 0; x < map.getWidth(); ++x)
//         {
//             int value = data[y][x];
//             if (value == CLASS_OBSTACLE)
//             {
// //                 LOG_DEBUG("inspecting around obstacle cell %i %i", x, y);
//                 lut.markAllRadius(data, map.getWidth(), map.getHeight(), x, y, CLASS_OBSTACLE);
//             }
//         }
//     }
// 
//     for (size_t y = 0; y < map.getHeight(); ++y)
//     {
//         for (size_t x = 0; x < map.getWidth(); ++x)
//         {
//             if (data[y][x] == 255)
//                 data[y][x] = CLASS_OBSTACLE;
//         }
//     }
// }
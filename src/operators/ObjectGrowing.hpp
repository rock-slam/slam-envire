#ifndef OBJECTGROWING_H
#define OBJECTGROWING_H

#include <envire/core/Operator.hpp>
#include <envire/maps/TraversabilityGrid.hpp>

namespace envire {
    
template <typename X>
class GrowthPolicy {
public:
    GrowthPolicy(X maxValue)
    {
        policy.resize(boost::extents[maxValue][maxValue]);
    }
    
    void setBigger(X a, X b, bool isBigger)
    {
        policy[a][b] = isBigger;
    }
    
    bool isBigger(X a, X b)
    {
        return policy[a][b];
    }
    
private:
    boost::multi_array<bool, 2> policy;
};

template <typename Y>
class ObjectGrowing
{
public:
    void growObjects(GrowthPolicy<Y> &policy, Grid< Y >& mapIn, Grid< Y >& mapOut, const std::string& band_name, double newSize)
    {
        const double width_square = pow(newSize,2);
        const int 
            wx = newSize / mapIn.getScaleX() + 1, 
            wy = newSize / mapIn.getScaleY() + 1;
        const double 
            sx = mapIn.getScaleX(),
            sy = mapIn.getScaleY();


        typename Grid< Y >::ArrayType& orig_data = band_name.empty() ?
            mapIn.getGridData() :
            mapIn.getGridData(band_name);

        typename Grid< Y >::ArrayType &data( mapOut.getGridData(band_name) );

        if(orig_data.num_elements() != data.num_elements())
            throw std::runtime_error("ObjectGrowing, input and output data have differens sizes");
    
        memcpy(data.data(), orig_data.data(), sizeof(Y) * orig_data.num_elements());
        
        for (unsigned int y = 0; y < mapIn.getCellSizeX(); ++y)
        {
            for (unsigned int x = 0; x < mapIn.getCellSizeX(); ++x)
            {
                Y value = orig_data[y][x];
                //grow all object in the radius
                for( int oy = -wy; oy <= wy; ++oy )
                {
                    for( int ox = -wx; ox <= wx; ++ox )
                    {
                        const int tx = x+ox;
                        const int ty = y+oy;
                        if( (pow(ox*sx,2) + pow(oy*sy,2) < width_square )
                                && tx >= 0 && tx < (int)mapIn.getCellSizeX()
                                && ty >= 0 && ty < (int)mapIn.getCellSizeY() )
                        {
                            if(policy.isBigger(value, data[ty][tx]))
                            {
                                data[ty][tx] = value;
                            }
                        }
                    }
                }
            }
        }
    }
    
private:
    
};

class ObjectGrowingUINT8 : public ObjectGrowing<uint8_t>, public envire::Operator 
{
        ENVIRONMENT_ITEM( ObjectGrowingUINT8 );
public:
    virtual bool updateAll();
};


}
#endif // OBJECTGROWING_H

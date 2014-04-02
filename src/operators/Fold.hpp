#ifndef ENVIRE_OP_FOLD_HPP
#define ENVIRE_OP_FOLD_HPP

#include "../core/Operator.hpp"
#include "../maps/Grid.hpp"

namespace envire {

    
template <typename T>
class FoldOperator: public envire::Operator {
    


public:
    
    FoldOperator(): neighbourhood(0) {};
    
    void setNeightbourHoodSize(size_t size)
    {
        neighbourhood = size;
    }
    
    void fold(const typename Grid< T >::ArrayType &inputData, typename Grid< T >::ArrayType &outputData, size_t xs, size_t ys)
    {
        size_t cnt = 0;
        T sum = T();
        int nHalf = ceil(neighbourhood / 2.0);
        for(int y = -nHalf; y < nHalf ; y++)
        {
            for(int x = -nHalf;x < nHalf; x++)
            {
                size_t xr = xs + x;
                size_t yr = ys + y;
                
                //no negative check needed, will overflow if negative
                if(xr > maxX || yr > maxY)
                    continue;
                
                cnt++;
                
                sum += inputData[yr][xr];
            }
        }
        
        if(cnt)
            sum /= cnt;
        
        outputData[ys][xs] = sum;
    }
    
    virtual bool updateAll()
    {
        envire::Grid<T> *inputGrid = getInput<envire::Grid<T> *>();
        if(!inputGrid)
            throw std::runtime_error("FoldOperator: no input band set, or wrong type");
    
        envire::Grid<T> *outputGrid = getOutput< envire::Grid<T> *>();
        if (!outputGrid)
            throw std::runtime_error("FoldOperator: no output band set, or wrong type");

        if(inputGrid->getCellSizeX() != outputGrid->getCellSizeX() || inputGrid->getCellSizeY() != outputGrid->getCellSizeY())
            throw std::runtime_error("FoldOperator: Error, grids have different sizes");

        typename envire::Grid<T>::ArrayType &inputData(inputGrid->getGridData());
        typename envire::Grid<T>::ArrayType &outputData(outputGrid->getGridData());

        maxX = inputGrid->getCellSizeX();
        maxY = inputGrid->getCellSizeY();
        
        for(size_t y = 0;y < maxY; y++)
        {
            for(size_t x = 0;x < maxX; x++)
            {
                fold(inputData, outputData, x, y);
            }
        }
        
        return true;
    }
    
    
protected:
    size_t maxX;
    size_t maxY;

    size_t neighbourhood;
};


class DoubleFoldOperator: public FoldOperator<double> {
        ENVIRONMENT_ITEM( DoubleFoldOperator );
};

}

#endif

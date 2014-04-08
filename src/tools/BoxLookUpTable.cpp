#include "BoxLookUpTable.hpp"
#include <math.h>
#include <stdexcept>
#include <iostream>
#include <iomanip>

namespace envire
{

BoxLookUpTable::BoxLookUpTable() : scale(0), sizeX(0), sizeY(0), maxDistFromBox(0), distanceTable(NULL)
{

}
    
void BoxLookUpTable::recompute(double scalei, double sizeXi, double sizeYi, double maxDistFromBox)
{
    double epsilon = 0.00001;
    if((fabs(scale - scalei) < epsilon) 
        && (fabs(sizeXi - sizeX) < epsilon) 
        && (fabs(sizeYi - sizeY) < epsilon) 
        && ((this->maxDistFromBox + epsilon) > maxDistFromBox ))
        return;
    
    scale = scalei;
    sizeX = sizeXi;
    sizeY = sizeYi;
    this->maxDistFromBox = maxDistFromBox;
    
    
    sizeXHalf = sizeX / 2.0;
    sizeYHalf = sizeY / 2.0;
    
    //move cordinate frame into the center of a box
    offsetX = (scale / 2 - fmod(sizeXHalf + maxDistFromBox, scale));
    offsetY = (scale / 2 - fmod(sizeYHalf + maxDistFromBox, scale));
    if(offsetX < 0)
        offsetX += scale;
    if(offsetY < 0)
        offsetY += scale;
    
    cellSizeX = ceil((offsetX + sizeX + maxDistFromBox * 2) / scale);
    cellSizeY = ceil((offsetY + sizeY + maxDistFromBox * 2) / scale);
    
    centerX = sizeXHalf + maxDistFromBox + offsetX;
    centerY = sizeYHalf + maxDistFromBox + offsetY;

    computeDistances();
}
    
void BoxLookUpTable::computeDistances()
{
    if(distanceTable)
        delete[] distanceTable;
        
    distanceTable = new double[cellSizeX * cellSizeY];
    
    for(int y = 0; y < cellSizeY; y++)
    {
        for(int x = 0; x < cellSizeX;x++)
        {
            distanceTable[cellSizeX * y + x] = 0;//std::numeric_limits< double >::max();
        }
    }
    
    for(double x = -(sizeXHalf + maxDistFromBox); x < (sizeXHalf + maxDistFromBox); x+= scale / 10.0)
    {        
        int xi = (x + centerX) / scale;
        if(xi < 0 || xi >= cellSizeX)
            continue;
        
        for(double y = -(sizeYHalf + maxDistFromBox); y < (sizeYHalf + maxDistFromBox); y+= scale / 10.0)
        {
            int yi = (y + centerY) / scale;

            if(yi < 0 || yi >= cellSizeY)
                continue;
            
            double xp = x;
            double yp = y;
            double dist = 0;
            if(xp < 0)
                xp *= -1;
            if(yp < 0)
                yp *= -1;
            
            if(xp > sizeXHalf)
            {
                if(yp > sizeYHalf)
                {
                    //edge case 
                    dist = sqrt(pow(xp - sizeXHalf, 2) + pow(yp - sizeYHalf, 2));
                } 
                else
                {
                    dist = xp - sizeXHalf;
                }
            } 
            else
            {
                if(yp > sizeYHalf)
                {
                   dist = yp - sizeYHalf;
                }
                else
                {
                    //inside of box
                    dist = 0;
                }
            }

            distanceTable[cellSizeX * yi + xi] = std::max(dist, distanceTable[cellSizeX * yi + xi]);

        }
    }
}

const double& BoxLookUpTable::getDistanceToBox(double xp, double yp) const
{
    if(!distanceTable)
        throw std::runtime_error("BoxLookUpTable::Lookup table is not computed");    

    int x = (xp + centerX) / scale;
    int y = (yp + centerY) / scale;
    
    if(x < 0 || x > cellSizeX || y < 0 || y > cellSizeY)
    {
        std::cout << "xp " << xp << " yp " << yp << " x " << x << " y " << y << std::endl; 
        throw std::runtime_error("BoxLookUpTable:: Distance lookup out of bound");
    }
    
    return distanceTable[cellSizeX * y + x];
}

void BoxLookUpTable::printDebug()
{
    for(int y = 0; y < cellSizeY; y++)
    {
        for(int x = 0; x < cellSizeX;x++)
        {
            std::cout << std::fixed << std::setw(4) << std::setprecision(2) << distanceTable[cellSizeX * y + x] << " ";
        }
        std::cout << std::endl;
    }
}

    
}
#include "BoxLookUpTable.hpp"
#include <math.h>
#include <stdexcept>
#include <iostream>
#include <iomanip>

namespace envire
{

BoxLookUpTable::BoxLookUpTable() : scale(0), width(0), height(0), maxDistFromBox(0), distanceTable(NULL)
{

}
    
void BoxLookUpTable::recompute(double scale, double width, double height, double maxDistFromBox)
{
    double epsilon = 0.00001;
    if((fabs(scale - this->scale) < epsilon) 
        && (fabs(width - this->width) < epsilon) 
        && (fabs(height - this->height) < epsilon) 
        && ((this->maxDistFromBox + epsilon) > maxDistFromBox ))
        return;
    
    this->scale = scale;
    this->width = width;
    this->height = height;
    this->maxDistFromBox = maxDistFromBox;
    
    
    widthHalf = width / 2.0;
    heightHalf = height / 2.0;
    
    //move cordinate frame into the center of a box
    offsetX = (scale / 2 - fmod(widthHalf + maxDistFromBox, scale));
    offsetY = (scale / 2 - fmod(heightHalf + maxDistFromBox, scale));
    if(offsetX < 0)
        offsetX += scale;
    if(offsetY < 0)
        offsetY += scale;
    
    sizeX = ceil((offsetX + width + maxDistFromBox * 2) / scale);
    sizeY = ceil((offsetY + height + maxDistFromBox * 2) / scale);
    
    centerX = widthHalf + maxDistFromBox + offsetX;
    centerY = heightHalf + maxDistFromBox + offsetY;

    computeDistances();
}
    
void BoxLookUpTable::computeDistances()
{
    if(distanceTable)
        delete[] distanceTable;
        
    distanceTable = new double[sizeX * sizeY];
    
    for(int y = 0; y < sizeY; y++)
    {
        for(int x = 0; x < sizeX;x++)
        {
            distanceTable[sizeX * y + x] = 0;//std::numeric_limits< double >::max();
        }
    }
    
    for(double x = -(widthHalf + maxDistFromBox); x < (widthHalf + maxDistFromBox); x+= scale / 10.0)
    {        
        int xi = (x + centerX) / scale;
        if(xi < 0 || xi > sizeX)
            continue;
        
        for(double y = -(heightHalf + maxDistFromBox); y < (heightHalf + maxDistFromBox); y+= scale / 10.0)
        {
            int yi = (y + centerY) / scale;

            if(yi < 0 || yi > sizeY)
                continue;
            
            double xp = x;
            double yp = y;
            double dist = 0;
            if(xp < 0)
                xp *= -1;
            if(yp < 0)
                yp *= -1;
            
            if(xp > widthHalf)
            {
                if(yp > heightHalf)
                {
                    //edge case 
                    dist = sqrt(pow(xp - widthHalf, 2) + pow(yp - heightHalf, 2));
                } 
                else
                {
                    dist = xp - widthHalf;
                }
            } 
            else
            {
                if(yp > heightHalf)
                {
                   dist = yp - heightHalf;
                }
                else
                {
                    //inside of box
                    dist = 0;
                }
            }

            distanceTable[sizeX * yi + xi] = std::max(dist, distanceTable[sizeX * yi + xi]);

        }
    }
}

const double& BoxLookUpTable::getDistanceToBox(double xp, double yp) const
{
    if(!distanceTable)
        throw std::runtime_error("BoxLookUpTable::Lookup table is not computed");    

    int x = (xp + centerX) / scale;
    int y = (yp + centerY) / scale;
    
    if(x < 0 || x > sizeX || y < 0 || y > sizeY)
    {
        std::cout << "xp " << xp << " yp " << yp << " x " << x << " y " << y << std::endl; 
        throw std::runtime_error("BoxLookUpTable:: Distance lookup out of bound");
    }
    
    return distanceTable[sizeX * y + x];
}

void BoxLookUpTable::printDebug()
{
    for(int y = 0; y < sizeY; y++)
    {
        for(int x = 0; x < sizeX;x++)
        {
            std::cout << std::fixed << std::setw(4) << std::setprecision(2) << distanceTable[sizeX * y + x] << " ";
        }
        std::cout << std::endl;
    }
}

    
}
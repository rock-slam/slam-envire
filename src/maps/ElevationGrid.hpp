#ifndef __ENVIRE_ELEVATIONGRID_HPP__
#define __ENVIRE_ELEVATIONGRID_HPP__

#include <envire/maps/Grid.hpp>

namespace envire
{
    class ElevationGrid : public Grid<double>
    {
        ENVIRONMENT_ITEM( ElevationGrid )
    public:
        static const std::string ELEVATION;
        static const std::string ELEVATION_MIN;
        static const std::string ELEVATION_MAX;
        static const std::string ILLUMINATION;
        static const std::string VISIBILITY;
    private:
        const static std::vector<std::string> bands;

        ElevationGrid::ArrayType *elevationArray;
    public:
        ElevationGrid() : Grid<double>() {};
        ElevationGrid(size_t width, size_t height, double scalex, double scaley, double offsetx = 0.0, double offsety = 0.0):Grid<double>::Grid(width,height,scalex,scaley,offsetx,offsety){};
        ~ElevationGrid(){};
        virtual const std::vector<std::string>& getBands() const {return bands;};

        void convertToFrame(base::samples::frame::Frame &frame)
        {
            Grid<double>::convertToFrame(ElevationGrid::ELEVATION,frame);
        }

        double& get(double x, double y)
        { return Grid<double>::get(ELEVATION, x, y); }

        double get(double x, double y) const
        { return Grid<double>::get(ELEVATION, x, y); }

        double& get( const Position& pos )
        { return Grid<double>::get(ELEVATION, pos.x, pos.y); }

        double get( const Position& pos ) const
        { return Grid<double>::get(ELEVATION, pos.x, pos.y); }

        Eigen::Vector3d getNormal( const Position& pos ) const;

        /** @brief get the normal vector at the given position
        */
        Eigen::Vector3d getNormal( const Point2D& pos ) const;


        /** @brief get the elevation at the given point
        *
        * The underlying model assumes the height value to be at
        * the center of the cell, and a surface is approximated
        * using the getNormal. The Height value is the value of the
        * plane at that point.
        */
        double getElevation( const Point2D& pos ) const;

  };

}
#endif

#define BOOST_TEST_MODULE MLSTest 
#include <boost/test/included/unit_test.hpp>
#include <boost/scoped_ptr.hpp>

#include <envire/maps/Grids.hpp>
#include <envire/maps/ElevationGrid.hpp>
#include <envire/tools/VoxelTraversal.hpp>
#include <envire/tools/BoxLookUpTable.hpp>

using namespace envire;
using namespace Eigen;

BOOST_AUTO_TEST_CASE( test_elevationgrid ) 
{
    ElevationGrid grid( 3, 3, 0.5, 0.5 );

    grid.get( 0.75, 0.75 ) = 1.0;
    grid.get( 0, 0.75 ) = 0.0;
    grid.get( 1.25, 0.75 ) = 2.0;
    grid.get( 0.75, 0.25 ) = -1.0;
    grid.get( 0.75, 1.25 ) = 3.0;

    BOOST_CHECK( Eigen::Vector3d( -2.0, -4.0, 1.0 ).normalized().isApprox( grid.getNormal( Eigen::Vector2d( 0.75, 0.75 ) ) ) );
    BOOST_CHECK_CLOSE( 1.5, grid.getElevation( Eigen::Vector2d( 1.0 - 1e-9, 0.75 ) ), 1e-5 );

}

BOOST_AUTO_TEST_CASE( test_voxeltraversal )
{
    ElevationGrid grid( 3, 3, 0.5, 0.5 );

    VoxelTraversal vt( grid );
    vt.init( Eigen::Vector3d( 0,0,0 ), Eigen::Vector3d( 1,1,0 ) );
    ElevationGrid::Position pos;
    while( vt.step( pos ) )
    {
	std::cout << pos.x << " " << pos.y << std::endl;
    }
}

void makeX(size_t x, size_t y, uint8_t val, TraversabilityGrid::ArrayType &array)
{
//     int v = val;
//     std::cout << "Got CB " << x << " " << y << " val " << v << std::endl; 
    (array)[y][x] = val;
}

void printMap(const TraversabilityGrid &tr)
{
    const TraversabilityGrid::ArrayType &data(tr.getGridData(TraversabilityGrid::TRAVERSABILITY));

    std::cout << std::endl << "X  ";  
    for(size_t x = 0; x < tr.getCellSizeX(); x++)
    {
        std::cout << std::fixed << std::setw(2) << x << " ";          
    }
     std::cout << std::endl ;   
    
    for(int y = tr.getCellSizeY()- 1; y >= 0; y--)
    {
        std::cout << std::fixed << std::setw(2) << y << " ";
        for(size_t x = 0; x < tr.getCellSizeX(); x++)
        {
            int foo = data[y][x];
            std::cout << std::fixed << std::setw(2) << foo << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "X  ";  
    for(size_t x = 0; x < tr.getCellSizeX(); x++)
    {
        std::cout << std::fixed << std::setw(2) << x << " ";          
    }
     std::cout << std::endl ;   

}

void verifyRect(double sizeX, double sizeY, base::Pose2D p, TraversabilityGrid &tr, uint8_t expected, uint8_t overwrite, uint8_t unset, uint8_t ow_unset )
{
    TraversabilityGrid::ArrayType &data(tr.getGridData(TraversabilityGrid::TRAVERSABILITY));
    
    Rotation2D<double> rot(p.orientation);

    GridBase::Position lastP_g;
    
    const double sizeXHalf = sizeX / 2.0;
    const double sizeYHalf = sizeY / 2.0;

    for(double x = -sizeXHalf; x <= sizeXHalf + 0.000001; x += 0.001)
    {
        for(double y = -sizeYHalf; y <= sizeYHalf + 0.000001; y += 0.001)
        {
            Vector2d p_w = p.position + rot * Vector2d(x, y);
            GridBase::Position p_g;
            
            if(tr.toGrid(p_w, p_g))
            {
//                 if(lastP_g != p_g)
//                     std::cout << "X " << p_g.x << " y " << p_g.y << std::endl;

                if(data[p_g.y][p_g.x] == expected)
                    data[p_g.y][p_g.x] = overwrite;
                
                if(data[p_g.y][p_g.x] == unset)
                    data[p_g.y][p_g.x] = ow_unset;
                
                lastP_g = p_g;
            }
            else 
            {
                std::cout << "Error point not in grid " << std::endl;
            }
        }
    }
}

BOOST_AUTO_TEST_CASE( test_forEachInRect ) 
{
    TraversabilityGrid tr(40, 40, 0.12, 0.12);
    
//     std::cout << "XSize " << tr.getCellSizeX() << " YSize " << tr.getCellSizeY() << std::endl;
    
    base::Pose2D p(Eigen::Vector2d(3.0,1.0), -0* M_PI / 180.0 );
    
    TraversabilityGrid::ArrayType &data(tr.getGridData(TraversabilityGrid::TRAVERSABILITY));
    
    double sizeY = .6;
    double sizeX = 1.0;
    
    tr.forEachInRectangle(p, sizeX, sizeY, boost::bind(makeX, _1, _2, 1, boost::ref(data)));

    Rotation2D<double> rot(p.orientation);

    GridBase::Position lastP_g;

    std::cout << "Robot at " << p.position.transpose() << " angle " << p.orientation / M_PI * 180.0 << " width " << sizeY << " height " << sizeX << std::endl;

    verifyRect(sizeX, sizeY, p, tr, 1, 2, 0, 4);
    printMap(tr);

    for(size_t x = 0; x < tr.getCellSizeX(); x++)
    {
        for(size_t y = 0; y < tr.getCellSizeY(); y++)
        {
            int foo = data[y][x];
            if(foo == 1)
                std::cout << "Not Cleared " << x << " " << y << std::endl;

        }
    }

}

BOOST_AUTO_TEST_CASE( test_forEachInRects ) 
{
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    TraversabilityGrid tr(40, 40, 0.15, 0.15);
    
//     std::cout << "XSize " << tr.getCellSizeX() << " YSize " << tr.getCellSizeY() << std::endl;
    
    
    TraversabilityGrid::ArrayType &data(tr.getGridData(TraversabilityGrid::TRAVERSABILITY));
    
    double sizeX = 1.0;
    double sizeY = .5;
    double border = 0.3;

//     base::Pose2D p(Eigen::Vector2d(width / 2 + border + 0.1 , height / 2 + border + 0.1), 0* M_PI / 180.0 );
    base::Pose2D p(Eigen::Vector2d(2 , 2.15), 42* M_PI / 180.0 );

    std::cout << "Robot at " << p.position.transpose() << " angle " << p.orientation / M_PI * 180.0 << " width " << sizeY << " height " << sizeX << " border " << border << std::endl;
    
    tr.forEachInRectangles(p, sizeX, sizeY, boost::bind(makeX, _1, _2, 1, boost::ref(data)),
                            sizeX + border * 2, sizeY + border * 2, boost::bind(makeX, _1, _2, 5, boost::ref(data))
    );

    Rotation2D<double> rot(p.orientation);
    GridBase::Position lastP_g;

    Vector2d p_t((-sizeX / 2.0 - border), (-sizeY / 2.0 - border));
    Vector2d p_w = p.position + rot * Vector2d((-sizeX / 2.0 - border), (-sizeY / 2.0 - border));
    GridBase::Position p_g;
            
    if(tr.toGrid(p_w, p_g))
    {
        std::cout <<"MIN X " <<  p_g.x << " " << p_g.y << " " << p_t.transpose() << " " << (p_w).transpose()<<  std::endl;
        std::cout <<"MIN X " <<  p_g.x << " " << p_g.y << " " << p_w.x() / .1 << " " << p_w.y() / 0.1<<  std::endl;
    }

    verifyRect(sizeX, sizeY, p, tr, 1, 2, 0, 4);

    verifyRect(sizeX+ 2*border, sizeY + 2*border, p, tr, 5, 7, 0, 6);
    
    printMap(tr);  
}

class DistanceHelper
{
public:
    BoxLookUpTable *boxLut;
    Eigen::Rotation2D<double> inverseOrientation;
    double scaleX;
    double scaleY;
    int xCenter;
    int yCenter;
    Eigen::Vector2d pos;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DistanceHelper(const base::Pose2D &pose, const TraversabilityGrid &grid, double sizeX, double sizeY, double borderWidth) : inverseOrientation(Eigen::Rotation2D<double>(pose.orientation).inverse()),
    scaleX(grid.getScaleX()), scaleY(grid.getScaleY())
    {
        boxLut = new BoxLookUpTable();
        //note the scale should be higher than the grid scale because 
        //of the roation. Else we get aliasing problems.
        boxLut->recompute(scaleX, sizeX, sizeY, borderWidth);

        pos = pose.position;
    }

    void getDistance(size_t x, size_t y, double &distance)
    {
//         std::cout << "Distance for " << x << " " << y << " CenterX " << xCenter << " " << yCenter << " " << ((int)(x - xCenter)) * scaleX << " "<< scaleX << " " << 4*0.1 <<  std::endl;
        Vector2d posAligned = inverseOrientation * (Vector2d(x * scaleX, y * scaleY) - pos);

        
        distance = boxLut->getDistanceToBox(posAligned.x(), posAligned.y());
    }
    
};



BOOST_AUTO_TEST_CASE( test_BoxLookupTable ) 
{
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    TraversabilityGrid tr(40, 40, 0.15, 0.15);
    
//     std::cout << "XSize " << tr.getCellSizeX() << " YSize " << tr.getCellSizeY() << std::endl;
    
    base::Pose2D p(Eigen::Vector2d(2,2), 13* M_PI / 180.0 );
    
    TraversabilityGrid::ArrayType &data(tr.getGridData(TraversabilityGrid::TRAVERSABILITY));
    
    double sizeX = 1.0;
    double sizeY = .6;
    double border = 0.5;

    std::cout << "Robot at " << p.position.transpose() << " angle " << p.orientation / M_PI * 180.0 << " width " << sizeY << " height " << sizeX << " border " << border << std::endl;

    DistanceHelper helper(p, tr, sizeX, sizeY, border*2);

    std::cout << "FOOOOBAR" << std::endl;
    std::cout << "Robot at " << p.position.transpose() << " angle " << p.orientation / M_PI * 180.0 << " width " << sizeY << " height " << sizeX << " border " << border << std::endl;
    helper.boxLut->printDebug();

    
    tr.forEachInRectangles(p, sizeX, sizeY, boost::bind(makeX, _1, _2, 1, boost::ref(data)),
                            sizeX + border * 2, sizeY + border * 2, boost::bind(makeX, _1, _2, 2, boost::ref(data))
    );

    std::cout << "Border Map" << std::endl;
    printMap(tr);

    std::cout << "Distance Map " << std::endl;
    for(int y = tr.getCellSizeY()- 1; y >= 0; y--)
    {
        std::cout << std::fixed << std::setw(2) << y << " ";
        for(size_t x = 0; x < tr.getCellSizeX(); x++)
        {
            double dist = 0;
            if(data[y][x] == 2)
                helper.getDistance(x,y, dist);
            std::cout << std::setw(4) << std::setprecision(2) << dist << " ";
        }
        std::cout << std::endl;
    }  
}


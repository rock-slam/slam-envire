#include "MLSVisualization.hpp"
#include "ColorConversion.hpp"

#include <osg/Group>
#include <osg/Geode>
#include <osg/Point>
#include <osg/Geometry>
#include <envire/Core.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <osg/Drawable>
#include <osg/ShapeDrawable>
#include <osg/LineWidth>

using namespace envire;

MLSVisualization::MLSVisualization()
    : horizontalCellColor(osg::Vec4(0.1,0.5,0.9,1.0)), 
    verticalCellColor(osg::Vec4(0.8,0.9,0.5,1.0)), 
    negativeCellColor(osg::Vec4(0.1,0.5,0.9,0.2)), 
    uncertaintyColor(osg::Vec4(0.5,0.1,0.1,0.3)), 
    showUncertainty(false),
    showNegative(false),
    estimateNormals(false),
    cycleHeightColor(true),
    cycleColorInterval(1.0)
{
}

MLSVisualization::~MLSVisualization() {}

osg::Vec3 Vec3( const Eigen::Vector3d& v )
{
    return osg::Vec3( v.x(), v.y(), v.z() );
}

osg::Vec3 Vec3( const Eigen::Vector2d& v )
{
    return osg::Vec3( v.x(), v.y(), 0 );
}

class ExtentsRectangle : public osg::Geode
{
    osg::ref_ptr<osg::Geometry> geom;
    osg::ref_ptr<osg::Vec4Array> color;
    osg::ref_ptr<osg::Vec3Array> vertices;

public:
    ExtentsRectangle( const envire::GridBase::Extents& extents ) :
	geom( new osg::Geometry() ),
	color( new osg::Vec4Array() ), 
	vertices( new osg::Vec3Array() )
    {
	Eigen::Vector2d min = extents.min(), max = extents.max();
	vertices->push_back( osg::Vec3( min.x(), min.y(), 0 ));
	vertices->push_back( osg::Vec3( min.x(), max.y(), 0 ));
	vertices->push_back( osg::Vec3( max.x(), max.y(), 0 ));
	vertices->push_back( osg::Vec3( max.x(), min.y(), 0 ));

	geom->setVertexArray(vertices);
	osg::ref_ptr<osg::DrawArrays> drawArrays = new osg::DrawArrays( osg::PrimitiveSet::LINE_LOOP, 0, vertices->size() );
	geom->addPrimitiveSet(drawArrays.get());

	color->push_back( osg::Vec4( 0.0f, 0.9f, 0.1f, 0.8f ) );
	geom->setColorArray(color.get());
	geom->setColorBinding( osg::Geometry::BIND_OVERALL );

	addDrawable(geom.get());    

	osg::StateSet* ss = getOrCreateStateSet();
	ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
	ss->setAttribute( new osg::LineWidth( 2.0 ) );
    }
};

osg::Group* MLSVisualization::getNodeForItem(envire::EnvironmentItem* item) const
{
    osg::ref_ptr<osg::Group> group = new osg::Group();
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    
    group->addChild(geode.get());
    
    updateNode(item, group);
    
    return group.release();
}

bool MLSVisualization::handlesItem(envire::EnvironmentItem* item) const
{
    return dynamic_cast<envire::MultiLevelSurfaceGrid *>(item);
}

void MLSVisualization::highlightNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    // TODO handle highlight and unhighlight
}

void MLSVisualization::unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    // TODO handle highlight and unhighlight
}

void drawBox(
	osg::ref_ptr<osg::Vec3Array> vertices, 
	osg::ref_ptr<osg::Vec3Array> normals, 
	osg::ref_ptr<osg::Vec4Array> colors,  
	const osg::Vec3& position, 
	const osg::Vec4& heights,
	const osg::Vec3& extents, 
	const osg::Vec4& color, 
	const osg::Vec3& normal )
{
    const double xp = position.x();
    const double yp = position.y();
    const double zp = position.z();

    const double xs = extents.x();
    const double ys = extents.y();
    const double zs = extents.z();

    const osg::Vec4 h( heights + osg::Vec4(zp,zp,zp,zp) );

    vertices->push_back(osg::Vec3(xp-xs*0.5, yp-ys*0.5, h[0]+zs*0.5));
    vertices->push_back(osg::Vec3(xp+xs*0.5, yp-ys*0.5, h[1]+zs*0.5));
    vertices->push_back(osg::Vec3(xp+xs*0.5, yp+ys*0.5, h[2]+zs*0.5));
    vertices->push_back(osg::Vec3(xp-xs*0.5, yp+ys*0.5, h[3]+zs*0.5));
    for(size_t i=0;i<4;i++)
    {
	normals->push_back(normal);
	colors->push_back(color);
    }

    if( zs > 0.0 )
    {
	vertices->push_back(osg::Vec3(xp-xs*0.5, yp-ys*0.5, h[0]+zs*0.5));
	vertices->push_back(osg::Vec3(xp+xs*0.5, yp-ys*0.5, h[1]+zs*0.5));
	vertices->push_back(osg::Vec3(xp+xs*0.5, yp-ys*0.5, h[2]-zs*0.5));
	vertices->push_back(osg::Vec3(xp-xs*0.5, yp-ys*0.5, h[3]-zs*0.5));
	for(size_t i=0;i<4;i++)
	{
	    normals->push_back(osg::Vec3(0,-1.0,0));
	    colors->push_back(color);
	}

	vertices->push_back(osg::Vec3(xp+xs*0.5, yp-ys*0.5, h[0]+zs*0.5));
	vertices->push_back(osg::Vec3(xp+xs*0.5, yp+ys*0.5, h[1]+zs*0.5));
	vertices->push_back(osg::Vec3(xp+xs*0.5, yp+ys*0.5, h[2]-zs*0.5));
	vertices->push_back(osg::Vec3(xp+xs*0.5, yp-ys*0.5, h[3]-zs*0.5));
	for(size_t i=0;i<4;i++)
	{
	    normals->push_back(osg::Vec3(1.0,0,0));
	    colors->push_back(color);
	}

	vertices->push_back(osg::Vec3(xp+xs*0.5, yp+ys*0.5, h[0]+zs*0.5));
	vertices->push_back(osg::Vec3(xp-xs*0.5, yp+ys*0.5, h[1]+zs*0.5));
	vertices->push_back(osg::Vec3(xp-xs*0.5, yp+ys*0.5, h[2]-zs*0.5));
	vertices->push_back(osg::Vec3(xp+xs*0.5, yp+ys*0.5, h[3]-zs*0.5));
	for(size_t i=0;i<4;i++)
	{
	    normals->push_back(osg::Vec3(0,1.0,0));
	    colors->push_back(color);
	}

	vertices->push_back(osg::Vec3(xp-xs*0.5, yp+ys*0.5, h[0]+zs*0.5));
	vertices->push_back(osg::Vec3(xp-xs*0.5, yp-ys*0.5, h[1]+zs*0.5));
	vertices->push_back(osg::Vec3(xp-xs*0.5, yp-ys*0.5, h[2]-zs*0.5));
	vertices->push_back(osg::Vec3(xp-xs*0.5, yp+ys*0.5, h[3]-zs*0.5));
	for(size_t i=0;i<4;i++)
	{
	    normals->push_back(osg::Vec3(-1.0,0,0));
	    colors->push_back(color);
	}

	vertices->push_back(osg::Vec3(xp-xs*0.5, yp-ys*0.5, h[0]-zs*0.5));
	vertices->push_back(osg::Vec3(xp+xs*0.5, yp-ys*0.5, h[1]-zs*0.5));
	vertices->push_back(osg::Vec3(xp+xs*0.5, yp+ys*0.5, h[2]-zs*0.5));
	vertices->push_back(osg::Vec3(xp-xs*0.5, yp+ys*0.5, h[3]-zs*0.5));
	for(size_t i=0;i<4;i++)
	{
	    normals->push_back(osg::Vec3(0,0,-1.0));
	    colors->push_back(color);
	}
    }
}

osg::Vec3 estimateNormal( MultiLevelSurfaceGrid::SurfacePatch patch, MultiLevelSurfaceGrid::Position pos, MultiLevelSurfaceGrid* grid ) {
    patch.stdev = grid->getScaleX() * 2;

    Eigen::Vector3d center;
    center << grid->fromGrid( pos ), patch.mean;

    Eigen::Vector3d d[2] = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() };
    for(int n=0;n<2;n++)
    {
	for(int i=-1;i<2;i+=2)
	{
	    MultiLevelSurfaceGrid::Position p( pos.x + n*i, pos.y + (n-1)*i );
	    MultiLevelSurfaceGrid::SurfacePatch *res;
	    if( grid->contains( p ) && (res = grid->get( p, patch )) )
	    {
		Eigen::Vector3d v;
		v << grid->fromGrid( p ), res->mean;
		d[n] += (v-center)*i;
	    }
	}
    }

    Eigen::Vector3d n = d[0].cross( d[1] );
    if( n.norm() > 0.0 )
    {
	n.normalize();
	return osg::Vec3(n.x(), n.y(), n.z());
    }
    else
	return osg::Vec3(0,0,1.0);
}


void MLSVisualization::updateNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    osg::ref_ptr<osg::Geode> geode = group->getChild(0)->asGeode();
    //remove old drawables
    while(geode->removeDrawables(0));

    envire::MultiLevelSurfaceGrid *mls = dynamic_cast<envire::MultiLevelSurfaceGrid *>(item);
    assert(mls);

    //this leads to CullVisitor error
    //because it is not implemented for mls and the exeption is 
    //catched 
    // add extents
    //group->removeChild( extents );
   // extents = new ExtentsRectangle( mls->getExtents() );
   // group->addChild( extents );
    
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    
    const double xs = mls->getScaleX();
    const double ys = mls->getScaleY();

    const double xo = mls->getOffsetX();
    const double yo = mls->getOffsetY();

    osg::ref_ptr<osg::Vec3Array> var_vertices = new osg::Vec3Array;

    //std::cerr << "grid size: " << mls->getWidth() << " x " << mls->getHeight() << std::endl;
    int hor = 0;
    for(size_t x=0;x<mls->getWidth();x++)
    {
	for(size_t y=0;y<mls->getHeight();y++)
	{
	    for( envire::MultiLevelSurfaceGrid::iterator it = mls->beginCell( x, y ); it != mls->endCell(); it++ )
	    {
		const envire::MultiLevelSurfaceGrid::SurfacePatch &p(*it);
		double xp = (x+0.5) * xs + xo;
		double yp = (y+0.5) * ys + yo; 

		osg::Vec4 heights(0,0,0,0);
		if( mls->getConfig().updateModel == MLSConfiguration::SLOPE )
		{
		    heights[0] = p.getHeight( Eigen::Vector2f( 0, 0 ) ) - p.mean;
		    heights[1] = p.getHeight( Eigen::Vector2f( xs, 0 ) ) - p.mean;
		    heights[2] = p.getHeight( Eigen::Vector2f( xs, ys ) ) - p.mean;
		    heights[3] = p.getHeight( Eigen::Vector2f( 0, ys ) ) - p.mean;
		}

		if( p.isHorizontal() )
		{
		    osg::Vec4 col;
		    if( mls->getHasCellColor() )
		    {
			base::Vector3d c = p.getColor();
			col = osg::Vec4( c.x(), c.y(), c.z(), 1.0 );
		    }
		    else if( cycleHeightColor )
                    {
                       double hue = (p.mean - std::floor(p.mean)) / cycleColorInterval;
			ColorConversion::hslToRgb( hue - std::floor(hue), 1.0, 0.6 , col.x(), col.y(), col.z());
                       col.w() = 1.0;
                    }
		    else
			col = horizontalCellColor;
		    
		    drawBox( vertices, normals, color, osg::Vec3( xp, yp, p.mean ), heights, osg::Vec3( xs, ys, 0.0 ), 
			    col,
			    estimateNormals ? 
				estimateNormal( p, MultiLevelSurfaceGrid::Position(x,y), mls ) :
				osg::Vec3( 0, 0, 1.0 ) );
		    hor++;
		}
		else
		{
		    if( p.isVertical() || showNegative )
		    {	
			osg::Vec4 col = p.isVertical() ? verticalCellColor : negativeCellColor;
			drawBox( vertices, normals, color, osg::Vec3( xp, yp, p.mean-p.height*.5 ), heights, osg::Vec3( xs, ys, p.height ), col, osg::Vec3(0, 0, 1.0) );
		    }
		}

		if( showUncertainty )
		{
		    var_vertices->push_back( osg::Vec3( xp, yp, p.mean - p.height * 0.5 + (p.height * 0.5 + p.stdev) ) );
		    var_vertices->push_back( osg::Vec3( xp, yp, p.mean - p.height * 0.5 - (p.height * 0.5 + p.stdev) ) );
		}
	    }
	}
    }
    //std::cerr << "vertices: " << vertices->size() << " hor: " << hor << std::endl;

    geom->setVertexArray(vertices);
    osg::ref_ptr<osg::DrawArrays> drawArrays = new osg::DrawArrays( osg::PrimitiveSet::QUADS, 0, vertices->size() );
    geom->addPrimitiveSet(drawArrays.get());

    geom->setNormalArray(normals);
    geom->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

    geom->setColorArray(color.get());
    geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

    geode->addDrawable(geom.get());    

    if( showUncertainty )
    {
	osg::ref_ptr<osg::Geometry> var_geom = new osg::Geometry;
	var_geom->setVertexArray( var_vertices );
	osg::ref_ptr<osg::DrawArrays> drawArrays = new osg::DrawArrays( osg::PrimitiveSet::LINES, 0, var_vertices->size() );
	var_geom->addPrimitiveSet(drawArrays.get());

	osg::ref_ptr<osg::Vec4Array> var_color = new osg::Vec4Array;
	var_color->push_back( osg::Vec4( 0.5, 0.1, 0.8, 1.0 ) );
	var_geom->setColorArray( var_color.get() );
	var_geom->setColorBinding( osg::Geometry::BIND_OVERALL );

	geode->addDrawable( var_geom.get() );
    }
}

bool MLSVisualization::isUncertaintyShown() const
{
    return showUncertainty;
}

void MLSVisualization::setShowUncertainty(bool enabled)
{
    showUncertainty = enabled;
    emit propertyChanged("show_uncertainty");
}

bool MLSVisualization::isNegativeShown() const
{
    return showNegative;
}

void MLSVisualization::setShowNegative(bool enabled)
{
    showNegative = enabled;
    emit propertyChanged("show_negative");
}

bool MLSVisualization::areNormalsEstimated() const
{
    return estimateNormals;
}

void MLSVisualization::setEstimateNormals(bool enabled)
{
    estimateNormals = enabled;
    emit propertyChanged("estimate_normals");
}

bool MLSVisualization::isHeightColorCycled() const
{
    return cycleHeightColor;
}

void MLSVisualization::setCycleHeightColor(bool enabled)
{
    cycleHeightColor = enabled;
    emit propertyChanged("cycle_height_color");
}

double MLSVisualization::getCycleColorInterval() const
{
    return cycleColorInterval;
}

void MLSVisualization::setCycleColorInterval(double interval)
{
    if(interval == 0.0)
        cycleColorInterval = 1.0;
    else
        cycleColorInterval = interval;
    emit propertyChanged("cycle_color_interval");
}

QColor MLSVisualization::getHorizontalCellColor() const
{
    QColor color;
    color.setRgbF(horizontalCellColor.x(), horizontalCellColor.y(), horizontalCellColor.z(), horizontalCellColor.w());
    return color;
}

void MLSVisualization::setHorizontalCellColor(QColor color)
{
    horizontalCellColor.x() = color.redF();
    horizontalCellColor.y() = color.greenF();
    horizontalCellColor.z() = color.blueF();
    horizontalCellColor.w() = color.alphaF();
    emit propertyChanged("horizontal_cell_color");
}

QColor MLSVisualization::getVerticalCellColor() const
{
    QColor color;
    color.setRgbF(verticalCellColor.x(), verticalCellColor.y(), verticalCellColor.z(), verticalCellColor.w());
    return color;
}

void MLSVisualization::setVerticalCellColor(QColor color)
{
    verticalCellColor.x() = color.redF();
    verticalCellColor.y() = color.greenF();
    verticalCellColor.z() = color.blueF();
    verticalCellColor.w() = color.alphaF();
    emit propertyChanged("vertical_cell_color");
}

QColor MLSVisualization::getNegativeCellColor() const
{
    QColor color;
    color.setRgbF(negativeCellColor.x(), negativeCellColor.y(), negativeCellColor.z(), negativeCellColor.w());
    return color;
}

void MLSVisualization::setNegativeCellColor(QColor color)
{
    negativeCellColor.x() = color.redF();
    negativeCellColor.y() = color.greenF();
    negativeCellColor.z() = color.blueF();
    negativeCellColor.w() = color.alphaF();
    emit propertyChanged("negative_cell_color");
}

QColor MLSVisualization::getUncertaintyColor() const
{
    QColor color;
    color.setRgbF(uncertaintyColor.x(), uncertaintyColor.y(), uncertaintyColor.z(), uncertaintyColor.w());
    return color;
}

void MLSVisualization::setUncertaintyColor(QColor color)
{
    uncertaintyColor.x() = color.redF();
    uncertaintyColor.y() = color.greenF();
    uncertaintyColor.z() = color.blueF();
    uncertaintyColor.w() = color.alphaF();
    emit propertyChanged("uncertainty_color");
}

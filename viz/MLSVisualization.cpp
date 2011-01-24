#include "MLSVisualization.hpp"
#include <osg/Group>
#include <osg/Geode>
#include <osg/Point>
#include <osg/Geometry>
#include <envire/Core.hpp>
#include <envire/maps/MultiLevelSurfaceGrid.hpp>
#include <osg/Drawable>
#include <osg/ShapeDrawable>

MLSVisualization::MLSVisualization()
    : vertexColor(osg::Vec4(0.1,0.5,0.9,1.0)), showUncertainty(true)
{
}

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
    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    color->push_back(osg::Vec4(1,0,0,1));
    osg::Geometry *geom = group->getChild(0)->asGeode()->getDrawable(0)->asGeometry();
    geom->setColorArray(color.get());
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );
}

void MLSVisualization::unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    color->push_back(vertexColor);
    osg::Geometry *geom = group->getChild(0)->asGeode()->getDrawable(0)->asGeometry();
    geom->setColorArray(color.get());
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );
}

void drawBox(osg::ref_ptr<osg::Vec3Array> vertices, osg::ref_ptr<osg::Vec3Array> normals, osg::ref_ptr<osg::Vec4Array> colors,  const osg::Vec3& position, const osg::Vec3& extents, const osg::Vec4& color )
{
    const double xp = position.x();
    const double yp = position.y();
    const double zp = position.z();

    const double xs = extents.x();
    const double ys = extents.y();
    const double zs = extents.z();

    vertices->push_back(osg::Vec3(xp-xs*0.5, yp-ys*0.5, zp+zs*0.5));
    vertices->push_back(osg::Vec3(xp+xs*0.5, yp-ys*0.5, zp+zs*0.5));
    vertices->push_back(osg::Vec3(xp+xs*0.5, yp+ys*0.5, zp+zs*0.5));
    vertices->push_back(osg::Vec3(xp-xs*0.5, yp+ys*0.5, zp+zs*0.5));
    for(size_t i=0;i<4;i++)
    {
	normals->push_back(osg::Vec3(0,0,1.0));
	colors->push_back(color);
    }

    if( zs > 0.0 )
    {
	vertices->push_back(osg::Vec3(xp-xs*0.5, yp-ys*0.5, zp+zs*0.5));
	vertices->push_back(osg::Vec3(xp+xs*0.5, yp-ys*0.5, zp+zs*0.5));
	vertices->push_back(osg::Vec3(xp+xs*0.5, yp-ys*0.5, zp-zs*0.5));
	vertices->push_back(osg::Vec3(xp-xs*0.5, yp-ys*0.5, zp-zs*0.5));
	for(size_t i=0;i<4;i++)
	{
	    normals->push_back(osg::Vec3(0,-1.0,0));
	    colors->push_back(color);
	}

	vertices->push_back(osg::Vec3(xp+xs*0.5, yp-ys*0.5, zp+zs*0.5));
	vertices->push_back(osg::Vec3(xp+xs*0.5, yp+ys*0.5, zp+zs*0.5));
	vertices->push_back(osg::Vec3(xp+xs*0.5, yp+ys*0.5, zp-zs*0.5));
	vertices->push_back(osg::Vec3(xp+xs*0.5, yp-ys*0.5, zp-zs*0.5));
	for(size_t i=0;i<4;i++)
	{
	    normals->push_back(osg::Vec3(1.0,0,0));
	    colors->push_back(color);
	}

	vertices->push_back(osg::Vec3(xp+xs*0.5, yp+ys*0.5, zp+zs*0.5));
	vertices->push_back(osg::Vec3(xp-xs*0.5, yp+ys*0.5, zp+zs*0.5));
	vertices->push_back(osg::Vec3(xp-xs*0.5, yp+ys*0.5, zp-zs*0.5));
	vertices->push_back(osg::Vec3(xp+xs*0.5, yp+ys*0.5, zp-zs*0.5));
	for(size_t i=0;i<4;i++)
	{
	    normals->push_back(osg::Vec3(0,1.0,0));
	    colors->push_back(color);
	}

	vertices->push_back(osg::Vec3(xp-xs*0.5, yp+ys*0.5, zp+zs*0.5));
	vertices->push_back(osg::Vec3(xp-xs*0.5, yp-ys*0.5, zp+zs*0.5));
	vertices->push_back(osg::Vec3(xp-xs*0.5, yp-ys*0.5, zp-zs*0.5));
	vertices->push_back(osg::Vec3(xp-xs*0.5, yp+ys*0.5, zp-zs*0.5));
	for(size_t i=0;i<4;i++)
	{
	    normals->push_back(osg::Vec3(-1.0,0,0));
	    colors->push_back(color);
	}

	vertices->push_back(osg::Vec3(xp-xs*0.5, yp-ys*0.5, zp-zs*0.5));
	vertices->push_back(osg::Vec3(xp+xs*0.5, yp-ys*0.5, zp-zs*0.5));
	vertices->push_back(osg::Vec3(xp+xs*0.5, yp+ys*0.5, zp-zs*0.5));
	vertices->push_back(osg::Vec3(xp-xs*0.5, yp+ys*0.5, zp-zs*0.5));
	for(size_t i=0;i<4;i++)
	{
	    normals->push_back(osg::Vec3(0,0,-1.0));
	    colors->push_back(color);
	}
    }
}

void MLSVisualization::updateNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    osg::ref_ptr<osg::Geode> geode = group->getChild(0)->asGeode();
    //remove old drawables
    while(geode->removeDrawables(0));
    
    envire::MultiLevelSurfaceGrid *mls = dynamic_cast<envire::MultiLevelSurfaceGrid *>(item);
    assert(mls);

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    
    const double xs = mls->getScaleX();
    const double ys = mls->getScaleY();

    //std::cerr << "grid size: " << mls->getWidth() << " x " << mls->getHeight() << std::endl;
    int hor = 0;
    for(size_t x=0;x<mls->getWidth();x++)
    {
	for(size_t y=0;y<mls->getHeight();y++)
	{
	    for( envire::MultiLevelSurfaceGrid::const_iterator it = mls->beginCell_const( x, y ); it != mls->endCell_const(); it++ )
	    {
		const envire::MultiLevelSurfaceGrid::SurfacePatch &p(*it);
		double xp = (x+0.5) * xs;
		double yp = (y+0.5) * ys; 

		if( p.horizontal == true )
		{
		    drawBox( vertices, normals, color, osg::Vec3( xp, yp, p.mean ), osg::Vec3( xs, ys, 0.0 ), vertexColor );
		    hor++;
		}
		else
		{
		    drawBox( vertices, normals, color, osg::Vec3( xp, yp, p.mean-p.height*.5 ), osg::Vec3( xs, ys, p.height ), vertexColor );
		}

		if( showUncertainty )
		{
		    drawBox( vertices, normals, color, osg::Vec3( xp, yp, p.mean-p.height*.5 ), osg::Vec3( xs/2.0, ys/2.0, p.height+2.0*p.stdev ), osg::Vec4( 0.6, 0.1, 0.1, 0.5) );
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
}

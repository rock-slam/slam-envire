#include <Eigen/Geometry>
#include "icp.hpp"

#include "envire/Core.hpp"
#include "envire/TriMesh.hpp"
#include "envire/ScanMeshing.hpp"

#include "boost/scoped_ptr.hpp"

#include <algorithm>

using namespace envire;
using namespace std;

int main( int argc, char* argv[] )
{
    if( argc < 3 ) 
    {
	std::cout << "usage: env_icp input output" << std::endl;
	exit(0);
    }

    Serialization so;
    boost::scoped_ptr<Environment> env(so.unserialize( argv[1] ));
    
    std::vector<envire::TriMesh*> meshes = env->getItems<envire::TriMesh>();

    Eigen::MatrixXi graph = Eigen::MatrixXi::Zero(meshes.size(), meshes.size());

    for(int i=0;i<meshes.size();i++)
    {
	meshes[i]->setDirty();
	meshes[i]->updateFromOperator();
    }

    for(int i=0;i<meshes.size();i++)
    {
	for(int j=0;j<meshes.size();j++)
	{
	    // condition for strictly upper triangular
	    if( j>i )
	    {
		ICP icp;
		icp.updateTree( meshes[i], 0.01 );
		envire::FrameNode::TransformType t = meshes[j]->getFrameNode()->getTransform();
		icp.updateAlignment( meshes[j], 0.2, 0.01);
		meshes[j]->getFrameNode()->setTransform(t);
		graph(i,j) = icp.getX().size();
	    }
	}
    }
    graph = graph + graph.transpose();

    std::cout << graph << std::endl;

    std::vector<int> taken;
    // get mesh with highest total number of adjecencies
    // and make it the root of our tree
    int i, j;
    graph.rowwise().sum().maxCoeff(&i, &j);

    envire::FrameNode* root = new envire::FrameNode();
    env->addChild(env->getRootNode(), root );

    env->addChild(root, meshes[i]->getFrameNode() );
    taken.push_back(i);

    std::cout << "root node is: " << i << std::endl;

    struct highest
    {
	highest() : parent(-1), child(-1), adjecency(-1) {};
	int parent;
	int child;
	int adjecency;
    };

    // after that, always get the highest adjecency to any of the 
    // meshes already put into the tree
    while( taken.size() < meshes.size() ) 
    {
	highest h;
	for(int i=0;i<taken.size();i++)
	{
	    for(int j=0;j<meshes.size();j++)
	    {
		if( !std::count(taken.begin(),taken.end(),j) && graph(i,j) > h.adjecency ) 
		{
		    h.parent = taken[i];
		    h.child = j;
		    h.adjecency = graph(i,j);
		}
	    }
	}

	if( h.adjecency > 0 )
	{
	    envire::FrameNode::TransformType t = env->relativeTransform(
		    meshes[h.child]->getFrameNode(),
		    meshes[h.parent]->getFrameNode());
	    env->addChild(meshes[h.parent]->getFrameNode(), meshes[h.child]->getFrameNode());
	    meshes[h.child]->getFrameNode()->setTransform( t );

	    taken.push_back(h.child);
	    std::cout << "added parent: " << h.parent << " child: " << h.child << std::endl;
	}
	else 
	{
	    std::cout << "could not find any adjecent maps." << std::endl;
	    exit(0);
	}
    }
    /*
    ICP icp;
    icp.addToModel( mesh );
    icp.align( 5, 0.01 );
    */

    std::string path(argv[2]);
    so.serialize(env.get(), path);
} 

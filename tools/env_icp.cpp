#include <Eigen/Geometry>
#include "icp.hpp"

#include "envire/Core.hpp"
#include "envire/maps/TriMesh.hpp"
#include "envire/operators/ScanMeshing.hpp"

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

    Eigen::MatrixXi adjMat = Eigen::MatrixXi::Zero(meshes.size(), meshes.size());

    for(size_t i=0;i<meshes.size();i++)
    {
	meshes[i]->setDirty();
	meshes[i]->updateFromOperator();
    }

    //double density = 0.02;
    //double threshold = 0.2;
    for(size_t i=0;i<meshes.size();i++)
    {
	for(size_t j=0;j<meshes.size();j++)
	{
	    // condition for strictly upper triangular
	    if( j>i )
	    {
		/*
		ICP icp;
		icp.updateTree( meshes[i], density );
		envire::FrameNode::TransformType t = meshes[j]->getFrameNode()->getTransform();
		icp.updateAlignment( meshes[j], threshold, density);
		meshes[j]->getFrameNode()->setTransform(t);
		adjMat(i,j) = icp.getX().size();
		*/
	    }
	}
    }
    Eigen::MatrixXi t = adjMat + adjMat.transpose();
    adjMat = t;

    std::cout << adjMat << std::endl;

    std::vector<std::pair<int,int> > graph;
    std::vector<int> taken;
    // get mesh with highest total number of adjecencies
    // and make it the root of our tree
    int i, j;
    adjMat.rowwise().sum().maxCoeff(&i, &j);

    std::cout << adjMat.rowwise().sum() << std::endl;
    std::cout << i << " " << j << std::endl;

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
	for(size_t i=0;i<taken.size();i++)
	{
	    for(size_t j=0;j<meshes.size();j++)
	    {
		if( !std::count(taken.begin(),taken.end(),j) && (adjMat(taken[i],j) > h.adjecency) ) 
		{
		    h.parent = taken[i];
		    h.child = j;
		    h.adjecency = adjMat(taken[i],j);
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

	    graph.push_back( make_pair(h.child, h.parent));
	    taken.push_back(h.child);
	    std::cout << "added parent: " << h.parent << " child: " << h.child << " pairs: " << h.adjecency << std::endl;
	}
	else 
	{
	    std::cout << "could not find any adjecent maps." << std::endl;
	    exit(0);
	}
    }
    
    int iter = 5;
    // perform the icp 
    for(int i=0;i<iter;i++)
    {
	/*
	ICP icp;
	icp.getConfiguration().density = 0.1+i/10.0;
	icp.getConfiguration().minPairs = 50;

	for(std::vector<std::pair<int,int> >::iterator it=graph.begin();it!=graph.end();it++)
	{
	    icp.addToModel( meshes[ it->second ] );
	    std::cout << "i: " << i << " model: " << it->second << " meas: " << it->first << std::endl;
	    icp.align(meshes[it->first], 5, 0.0001);
	}
	*/
    }
    /*
    ICP icp;
    icp.addToModel( mesh );
    icp.align( 5, 0.01 );
    */

    std::string path(argv[2]);
    so.serialize(env.get(), path);
} 

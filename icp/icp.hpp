#ifndef __ICP_H__
#define __ICP_H__

#include<Eigen/Core>
#include<Eigen/Geometry>
#include<Eigen/StdVector>
#include<Eigen/QR>

#include<kdtree++/kdtree.hpp>

#include<envire/Core.hpp>
#include<envire/TriMesh.hpp>

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

class ICP {
    struct TreeNode
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef double value_type;

	TreeNode(const Eigen::Matrix<value_type,3,1>& point, const Eigen::Matrix<value_type,3,1>& normal, bool edge)
	    : point(point), edge(edge), normal(normal) {}

	Eigen::Matrix<value_type,3,1> point;
	Eigen::Matrix<value_type,3,1> normal;
	bool edge;

	inline value_type operator[](size_t n) const
	{
	    return point[n];
	}
    };

    typedef KDTree::KDTree<3, TreeNode> tree_type;

    struct Result
    {
	Result( int iterations, double avg_error )
	    : iterations(iterations), avg_error(avg_error) {};

	int iterations;
	double avg_error;
    };
    
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	ICP();

	/** performs a single alignment of the measurement to the model.
	 * The model needs to be added using addToModel before this call.
	 * 
	 * @param measurement - the mesh that needs to be matched
	 * @param max_iter - maximum number of iterations
	 * @param min_error - minimum average square distance between points after which to stop
	 */
	Result align( envire::TriMesh* measurement, int max_iter, double min_error);

	/** performs a global alignment of all the meshes in measurements. 
	 * This method does not require previous calls to updateModel.
	 * The difference to the single align is that all models will be updated 
	 * iteratively in a round robin fashion.
	 */
	Result align( int max_iter, double min_error);

	/** adds the @param model trimesh to the ICP model
	 * 
	 * @param model - model to be added
	 */
	void addToModel( envire::TriMesh* model );
	
	/** resets the model and clears the kdtree 
	 */
	void clearModel();

	/** update the alignment of the measurment with respect to the model
	 *
	 * @param measurement - measurement trimesh which is aligned to the model
	 * @param threshold - is the max distance to look for matching closest points
	 * @param density - how many points of the mesh to take into account. 
	 */ 
        double updateAlignment( envire::TriMesh* measurement, double threshold, double density );

        void updateTree( envire::TriMesh* model, double density );
	void clearTree();

        std::vector<Eigen::Vector3d>& getX() { return x; };
        std::vector<Eigen::Vector3d>& getP() { return p; };
        
    private:
	/** store an internal vector of models */
	std::vector<envire::TriMesh*> modelVec;

	/** store x and p for debug */
        std::vector<Eigen::Vector3d> x, p;

	/** internal instance of kdtree */
        tree_type kdtree;

	/** support stuff for random number generation */
	boost::minstd_rand rand_gen;
	boost::variate_generator<boost::minstd_rand&, boost::uniform_real<> > rand;
};

#endif

#ifndef __SCANMESHING_HPP__
#define __SCANMESHING_HPP__

#include <boost/smart_ptr.hpp>
#include "Core.hpp" 
#include "LaserScan.hpp" 
#include "TriMesh.hpp" 

namespace envire {
    class ScanMeshing;
    typedef boost::shared_ptr<ScanMeshing> ScanMeshing_Ptr;

    class ScanMeshing : public Operator
    {
            float maxEdgeLength;

        public:
            void addInput( LaserScan_Ptr scan ); 
            void addOutput( TriMesh_Ptr mesh ); 

            void setMaxEdgeLength( float value );
            bool updateAll();
    };
}

#endif

#ifndef __ENVIRE_TOOLS_NUMERIC_HPP__
#define __ENVIRE_TOOLS_NUMERIC_HPP__

// this class contains small numeric helpers 

template <class T> inline T sq( T a ) { return a * a; }

template <class T> inline void kalman_update( T& mean, T& stdev, T m_mean, T m_stdev )
{
    const double var = sq( stdev );
    const double m_var = sq( m_stdev );
    double gain = var / (var + m_var);
    if( gain != gain )
	gain = 0.5; // this happens when both stdevs are 0. 
    mean = mean + gain * (m_mean - mean);
    stdev = sqrt((1.0-gain)*var);
}

#endif

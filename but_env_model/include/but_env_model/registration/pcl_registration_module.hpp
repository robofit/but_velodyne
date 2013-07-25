/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 25/1/2012
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */
#define ROS_NOTE( s ) {std::sstream ss; ss << s; ROS_OUT("%s", ss.str().c_str() ); std::cerr << ss.str() << std::endl; }

//! Static member initialization
template <typename PointSource, typename PointTarget, typename Scalar>
const std::string but_env_model::CPclRegistration<PointSource, PointTarget, Scalar>::m_mode_names[4] = {"NONE", "ICP", "ICPNL", "SCA" };

//! Set used mode
template <typename PointSource, typename PointTarget, typename Scalar>
void but_env_model::CPclRegistration<PointSource, PointTarget, Scalar>::setMode( EPclRegistrationMode mode )
{
	m_mode = mode;

	switch( m_mode )
	{
	case PCL_REGISTRATION_MODE_NONE:
		m_registrationPtr = 0;
		break;
	case PCL_REGISTRATION_MODE_ICP:
		m_registrationPtr = & m_algIcp;
		break;
	case PCL_REGISTRATION_MODE_ICPNL:
		m_registrationPtr = & m_algIcpNl;
		break;
	case PCL_REGISTRATION_MODE_SCA:
		m_registrationPtr = & m_algSCA;
		break;
	default:
		m_registrationPtr = 0;
	}
}

//! Process data
//! @param source Source point cloud - this should be aligned to the target cloud
//! @param target Target point cloud - to this cloud should be source cloud aligned
//! @param output Output point cloud
template <typename PointSource, typename PointTarget, typename Scalar>
bool but_env_model::CPclRegistration<PointSource, PointTarget, Scalar>::process( PointSourcePtr & source, PointTargetPtr & target, PointSourcePtr & output )
{
	if( m_registrationPtr == 0 || m_mode == PCL_REGISTRATION_MODE_NONE )
		return false;

	m_registrationPtr->setInputCloud(source);
	m_registrationPtr->setInputTarget(target);
	m_registrationPtr->align( *output );

	return m_registrationPtr->hasConverged();
}

//! Convert string to the mode
//! @param name Mode name
template <typename PointSource, typename PointTarget, typename Scalar>
typename but_env_model::EPclRegistrationMode
but_env_model::CPclRegistration<PointSource, PointTarget, Scalar>::modeFromString( const std::string & name )
{
	std::string converted( name );
	std::transform( converted.begin(), converted.end(), converted.begin(), ::toupper);

	if( converted == m_mode_names[0] )
		return PCL_REGISTRATION_MODE_NONE;

	if( converted == m_mode_names[1] )
		return PCL_REGISTRATION_MODE_ICP;

	if( converted == m_mode_names[2] )
		return PCL_REGISTRATION_MODE_ICPNL;

	if( converted == m_mode_names[3] )
		return PCL_REGISTRATION_MODE_SCA;

	return PCL_REGISTRATION_MODE_NONE;
}

//! Initialize parameters from the parameter server
//! @param node_handle Node handle
template <typename PointSource, typename PointTarget, typename Scalar>
void but_env_model::CPclRegistration<PointSource, PointTarget, Scalar>::init( ros::NodeHandle & node_handle )
{
	std::cerr << "Setting registration parameters:" << std::endl;

	// Handling mode
	std::string strMode("NONE");
	node_handle.param("registration_mode", strMode, strMode );
	setMode( modeFromString(strMode) );

	std::cerr << "Used registration mode: " <<  getStrMode() << ", " << strMode << ", " << modeFromString(strMode) << std::endl;

	//------------------------------------------
	// Common parameters

	// Maximum iterations
	int iterations(6);
	node_handle.param("registration_maximum_iterations", iterations, iterations );
	m_maxIterations = iterations;

	std::cerr << "Number of iterations: " << m_maxIterations << std::endl;

	// RANSAC outlier threshold
	m_RANSACOutlierRejectionThreshold = 0.05;
	node_handle.param("registration_ransac_outlier_rejection_threshold", m_RANSACOutlierRejectionThreshold, m_RANSACOutlierRejectionThreshold );

	std::cerr << "RANSAC outlier rejection threshold: " << m_RANSACOutlierRejectionThreshold << std::endl;

	// Maximal correspondence distance
	m_maxCorrespondenceDistance = std::sqrt (std::numeric_limits<double>::max ());
	node_handle.param("registration_maximal_correspondence_distance", m_maxCorrespondenceDistance, m_maxCorrespondenceDistance );

	// Transformation epsilon
	m_transformationEpsilon = 0.0;
	node_handle.param("registration_transformation_epsilon", m_transformationEpsilon, m_transformationEpsilon );

	//-----------------------------------------
	// SCA parameters

	// Minimum distances between samples
	m_scaMinSampleDistance = 0.0;
	node_handle.param("registration_sca_min_sample_distance", m_scaMinSampleDistance, m_scaMinSampleDistance );

	// Number of samples to use during each iteration.
	m_scaNumOfSamples = 3;
	node_handle.param("registration_sca_num_of_samples", m_scaNumOfSamples, m_scaNumOfSamples );

	// Number of neighbors to use when selecting a random feature correspondence
	m_scaCorrespondenceRamdomness = 10;
	node_handle.param("registration_sca_correspondence_randomness", m_scaCorrespondenceRamdomness, m_scaCorrespondenceRamdomness );
}

//! Reinitialize registration parameters
template <typename PointSource, typename PointTarget, typename Scalar>
void but_env_model::CPclRegistration<PointSource, PointTarget, Scalar>::resetParameters()
{
	if( m_mode == PCL_REGISTRATION_MODE_NONE )
		return;

	// Set common parameters
	setRegistrationParameters();

	// Set specialized parameters
	switch( m_mode )
	{
	case PCL_REGISTRATION_MODE_ICP:
		break;
	case PCL_REGISTRATION_MODE_ICPNL:
		break;
	case PCL_REGISTRATION_MODE_SCA:
		setSCAParameters();
		break;
	default:
		;
	}
}

//! Set common registration parameters
template <typename PointSource, typename PointTarget, typename Scalar>
void but_env_model::CPclRegistration<PointSource, PointTarget, Scalar>::setRegistrationParameters()
{
	m_registrationPtr->setMaximumIterations( m_maxIterations );
	m_registrationPtr->setRANSACOutlierRejectionThreshold ( m_RANSACOutlierRejectionThreshold );
	m_registrationPtr->setMaxCorrespondenceDistance( m_maxCorrespondenceDistance );
	m_registrationPtr->setTransformationEpsilon(m_transformationEpsilon);
}

//! Set SCA parameters
template <typename PointSource, typename PointTarget, typename Scalar>
void but_env_model::CPclRegistration<PointSource, PointTarget, Scalar>::setSCAParameters()
{
	m_algSCA.setMinSampleDistance( m_scaMinSampleDistance);
	m_algSCA.setNumberOfSamples( m_scaNumOfSamples );
	m_algSCA.setCorrespondenceRandomness(m_scaCorrespondenceRamdomness);
}



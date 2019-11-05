/*
 * Copyright (c) 2019 Andrew Price
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "coterie_rviz_plugin/PolymorphicSetDisplay.h"
#include "coterie_rviz_plugin/PolymorphicSetVisual.h"

#include "coterie/QuaternionTraits.hpp"

#include "coterie/sampling/extents.hpp"
#include "coterie/sampling/uniform_sample.hpp"
#include "coterie/sampling/halton.hpp"

#include "coterie/serialization/PolymorphicSetMsg.hpp"

#include "coterie/visualization/aabb_set.hpp"
#include "coterie/visualization/point_set.hpp"
#include "coterie/visualization/polytope_set.hpp"
#include "coterie/visualization/ellipsoidal_set.hpp"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/frame_manager.h>

#include <random>


namespace coterie_rviz_plugin
{

template <typename SetT, typename RosterT>
bool getExemplars(const SetT& set, const SET_SAMPLE_STYLE style, const int numSamples, coterie::RNG& rng, RosterT& examples)
{
	switch (style)
	{
	case SET_SAMPLE_STYLE::EXTENTS:
	{
		coterie::getExtents(set, examples);
		examples.resize(std::min(static_cast<int>(examples.size()), numSamples));
		return true;
	}
	case SET_SAMPLE_STYLE::RANDOM:
	{
		coterie::sampleUniform(set, numSamples, examples, rng);
		break;
	}
	case SET_SAMPLE_STYLE::HALTON:
	{
		coterie::sampleHalton(set, numSamples, examples, rng);
		break;
	}
	}

	return false;
}

void visualizePosition(const coterie_msgs::PolymorphicSet& set, visualization_msgs::MarkerArray& ma)
{
	// TODO: if space == SE3
	// Get the set info
	switch (set.type)
	{
	case coterie_msgs::PolymorphicSet::TYPE_AABB_SET:
	{
		auto aabb = coterie::deserialize(set.aabb.front());
		auto m = coterie::visualizePosition(aabb, Eigen::VectorXd::Ones(aabb.dimension).eval());
		ma.markers.push_back(m);
		break;
	}
	case coterie_msgs::PolymorphicSet::TYPE_POINT_SET:
	{
		auto points = coterie::deserialize(set.point.front());
		auto m = coterie::visualizePosition(points, Eigen::VectorXd::Ones(points.dimension).eval());
		ma.markers.push_back(m);
		break;
	}
	case coterie_msgs::PolymorphicSet::TYPE_POLYTOPE_SET:
	{
		auto polytope = coterie::deserialize(set.polytope.front());
		auto m = coterie::visualizePosition(polytope, Eigen::VectorXd::Ones(polytope.dimension).eval());
		ma.markers.push_back(m);
		break;
	}
	case coterie_msgs::PolymorphicSet::TYPE_ELLIPSOIDAL_SET:
	{
		auto ellipsoid = coterie::deserialize(set.ellipsoid.front());
		auto m = coterie::visualizePosition(ellipsoid, Eigen::VectorXd::Ones(ellipsoid.dimension).eval());
		ma.markers.push_back(m);
		break;
	}
//	case coterie_msgs::PolymorphicSet::TYPE_RASTER_SET:
//	{
//		auto raster = coterie::deserialize(set.raster.front());
//		break;
//	}
	default:
		throw std::runtime_error("Set type '" + std::to_string(set.type) + "' unknown.");
	}
}

template <typename Collection>
std::vector<geometry_msgs::Transform> deltasToTransforms(const Collection& deltas, const uint8_t space)
{
	std::vector<geometry_msgs::Transform> tfs;

	geometry_msgs::Transform Identity;
	Identity.rotation.w = 1.0;

	for (const auto& delta : deltas)
	{
		geometry_msgs::Transform T = Identity;
		switch (space)
		{
		case coterie_msgs::PolymorphicSet::SPACE_R1:
		{
			T.translation.x += delta[0];
			break;
		}
		case coterie_msgs::PolymorphicSet::SPACE_R2:
		{
			T.translation.x += delta[0];
			T.translation.y += delta[1];
			break;
		}
		case coterie_msgs::PolymorphicSet::SPACE_R3:
		{
			T.translation.x += delta[0];
			T.translation.y += delta[1];
			T.translation.z += delta[2];
			break;
		}
		case coterie_msgs::PolymorphicSet::SPACE_SO2:
		{
			Eigen::Vector3d r(0, 0, delta[0]);
			Eigen::Quaterniond q = coterie::manifold_traits<Eigen::Quaterniond>::expmap(r);
			T.rotation.x = q.x();
			T.rotation.y = q.y();
			T.rotation.z = q.z();
			T.rotation.w = q.w();
			break;
		}
		case coterie_msgs::PolymorphicSet::SPACE_SE2:
		{
			T.translation.x += delta[0];
			T.translation.y += delta[1];

			Eigen::Vector3d r(0, 0, delta[2]);
			Eigen::Quaterniond q = coterie::manifold_traits<Eigen::Quaterniond>::expmap(r);
			T.rotation.x = q.x();
			T.rotation.y = q.y();
			T.rotation.z = q.z();
			T.rotation.w = q.w();
			break;
		}
		case coterie_msgs::PolymorphicSet::SPACE_SO3:
		{
			Eigen::Vector3d r(delta[0], delta[1], delta[2]);
			Eigen::Quaterniond q = coterie::manifold_traits<Eigen::Quaterniond>::expmap(r);
			T.rotation.x = q.x();
			T.rotation.y = q.y();
			T.rotation.z = q.z();
			T.rotation.w = q.w();
			break;
		}
		case coterie_msgs::PolymorphicSet::SPACE_SE3:
		{
			T.translation.x += delta[0];
			T.translation.y += delta[1];
			T.translation.z += delta[2];

			Eigen::Vector3d r(delta[3], delta[4], delta[5]);
			Eigen::Quaterniond q = coterie::manifold_traits<Eigen::Quaterniond>::expmap(r);
			T.rotation.x = q.x();
			T.rotation.y = q.y();
			T.rotation.z = q.z();
			T.rotation.w = q.w();
			break;
		}
		default:
			ROS_ERROR_STREAM("Space type '" << std::to_string(space) << "' unknown.");
		}

		tfs.push_back(T);
	}

	return tfs;
}

void getTransforms(const SET_SAMPLE_STYLE style, const int num_samples, const coterie_msgs::PolymorphicSet& set, coterie::RNG& rng, std::vector<geometry_msgs::Transform>& tfs)
{
	std::vector<Eigen::VectorXd> samples;
	switch (set.type)
	{
	case coterie_msgs::PolymorphicSet::TYPE_AABB_SET:
	{
		auto aabb = coterie::deserialize(set.aabb.front());
		getExemplars(aabb, style, num_samples, rng, samples);
		break;
	}
	case coterie_msgs::PolymorphicSet::TYPE_POINT_SET:
	{
		auto points = coterie::deserialize(set.point.front());
		getExemplars(points, style, num_samples, rng, samples);
		break;
	}
	case coterie_msgs::PolymorphicSet::TYPE_POLYTOPE_SET:
	{
		auto polytope = coterie::deserialize(set.polytope.front());
		getExemplars(polytope, style, num_samples, rng, samples);
		break;
	}
	case coterie_msgs::PolymorphicSet::TYPE_ELLIPSOIDAL_SET:
	{
		auto ellipsoid = coterie::deserialize(set.ellipsoid.front());
		getExemplars(ellipsoid, style, num_samples, rng, samples);
		break;
	}
//	case coterie_msgs::PolymorphicSet::TYPE_RASTER_SET:
//	{
//		auto raster = coterie::deserialize(set.raster.front());
//		break;
//	}
	default:
		throw std::runtime_error("Set type '" + std::to_string(set.type) + "' unknown.");
	}


	tfs = deltasToTransforms(samples, set.space);
}

PolymorphicSetDisplay::PolymorphicSetDisplay()
	: MarkerDisplay()
{
	marker_topic_property_->setMessageType( QString::fromStdString( ros::message_traits::datatype<MsgType>() ));
	marker_topic_property_->setValue( "set_test" );
	marker_topic_property_->setDescription( "coterie_msgs::PolymorphicSetVisualizationStamped topic to subscribe to." );

	style_property_ = new rviz::EnumProperty("Style", "Extents",
	                                         "Visualization style to use for displaying a finite selection of a set.",
	                                         this, SLOT(updateStyle()));
	style_property_->addOption("Extents", SET_SAMPLE_STYLE::EXTENTS);
	style_property_->addOption("Random", SET_SAMPLE_STYLE::RANDOM);
	style_property_->addOption("Halton", SET_SAMPLE_STYLE::HALTON);

	sample_count_property_ = new rviz::IntProperty("Number of Samples", 10,
	                                               "Number of sampled poses to display.",
	                                               this, SLOT(updateSampleCount()));

	color_property_ = new rviz::ColorProperty( "Color", QColor( 204, 51, 204 ),
	                                           "Color to draw the acceleration arrows.",
	                                           this, SLOT( updateColorAndAlpha() ));

	alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
	                                           "0 is fully transparent, 1.0 is fully opaque.",
	                                           this, SLOT( updateColorAndAlpha() ));

	history_length_property_ = new rviz::IntProperty( "History Length", 1,
	                                                  "Number of prior measurements to display.",
	                                                  this, SLOT( updateHistoryLength() ));
	history_length_property_->setMin( 1 );
	history_length_property_->setMax( 100000 );

	rng = std::make_shared<coterie::RNG>();
}

PolymorphicSetDisplay::~PolymorphicSetDisplay() = default;


void PolymorphicSetDisplay::subscribe()
{
	if ( !isEnabled() )
	{
		return;
	}

	std::string topic = marker_topic_property_->getTopicStd();
	if( !topic.empty() )
	{
		array_sub_.shutdown();

		try
		{
			array_sub_ = update_nh_.subscribe( topic, queue_size_property_->getInt(),
			                                   (void (PolymorphicSetDisplay::*)(const MsgType::ConstPtr&))
				                                   &PolymorphicSetDisplay::processMessage, this );
			setStatus( rviz::StatusProperty::Ok, "Topic", "OK" );
		}
		catch( ros::Exception& e )
		{
			setStatus( rviz::StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
		}
	}
}

void PolymorphicSetDisplay::unsubscribe()
{
	array_sub_.shutdown();
}

void PolymorphicSetDisplay::updateStyle()
{
	active_style_ = static_cast<SET_SAMPLE_STYLE>(style_property_->getOptionInt());
	sample_count_property_->setReadOnly(SET_SAMPLE_STYLE::EXTENTS == active_style_);

	regenerateDisplay();
}

void PolymorphicSetDisplay::updateSampleCount()
{
	active_sample_count_ = sample_count_property_->getInt();

	regenerateDisplay();
}

// Set the current color and alpha values for each visual.
void PolymorphicSetDisplay::updateColorAndAlpha()
{
	applyColorAndAlpha(*active_markers_);

	regenerateDisplay();
}

// Set the number of past visuals to show.
void PolymorphicSetDisplay::updateHistoryLength()
{
	visuals_.rset_capacity(history_length_property_->getInt());
}

void PolymorphicSetDisplay::applyColorAndAlpha(visualization_msgs::MarkerArray& ma) const
{
	Ogre::ColourValue color = color_property_->getOgreColor();
	color.a = alpha_property_->getFloat();
	for (auto& m : ma.markers)
	{
		m.color.r = color.r; m.color.g = color.g; m.color.b = color.b; m.color.a = color.a;
	}
}

geometry_msgs::Pose operator*(const geometry_msgs::Pose& P, const geometry_msgs::Transform& T)
{
	tf::Transform A;
	tf::poseMsgToTF(P, A);
	tf::Transform B;
	tf::transformMsgToTF(T, B);
	tf::Transform C = A * B;
	geometry_msgs::Pose D;
	tf::poseTFToMsg(C, D);
	return D;
}

// This is our callback to handle an incoming message.
void PolymorphicSetDisplay::processMessage( const MsgType::ConstPtr& msg )
{
	last_message_ = msg;
	regenerateDisplay();
}

void PolymorphicSetDisplay::regenerateDisplay()
{
	// Check if we have anything to display
	if (!last_message_) { return; }

	// Here we call the rviz::FrameManager to get the transform from the
	// fixed frame to the frame in the header of this Imu message.  If
	// it fails, we can't do anything else so we return.
	Ogre::Quaternion orientation;
	Ogre::Vector3 position;
	if( !context_->getFrameManager()->getTransform( last_message_->header.frame_id,
	                                                last_message_->header.stamp,
	                                                position, orientation ))
	{
		ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
		           last_message_->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
		return;
	}

	// Clean up old markers
	if (active_markers_)
	{
		for (auto& m : active_markers_->markers)
		{
			m.action = visualization_msgs::Marker::DELETEALL;
		}

		rviz::MarkerDisplay::incomingMarkerArray(active_markers_);
	}

	active_markers_ = boost::make_shared<visualization_msgs::MarkerArray>();

	if (last_message_->marker.markers.empty())
	{
		visualizePosition(last_message_->set, *active_markers_);
	}
	else
	{
		std::vector<geometry_msgs::Transform> tfs;
		getTransforms(active_style_, active_sample_count_, last_message_->set, *rng, tfs);
		ROS_WARN_STREAM("Got " << tfs.size() << " transforms.");
		int count = 0;
		for (const auto& T : tfs)
		{
			for (auto m : last_message_->marker.markers) // make a copy each time
			{
				m.pose = m.pose * T;
				m.id = ++count;
				active_markers_->markers.push_back(m);
			}
		}
	}

	for (auto& m : active_markers_->markers)
	{
		m.header = last_message_->header;
	}

	applyColorAndAlpha(*active_markers_);
	rviz::MarkerDisplay::incomingMarkerArray(active_markers_);

}

} // namespace coterie_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(coterie_rviz_plugin::PolymorphicSetDisplay,rviz::Display )

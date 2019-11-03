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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

namespace coterie_rviz_plugin
{

PolymorphicSetDisplay::PolymorphicSetDisplay()
{
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
}

void PolymorphicSetDisplay::onInitialize()
{
	MFDClass::onInitialize();
	updateHistoryLength();
}

PolymorphicSetDisplay::~PolymorphicSetDisplay() = default;

void PolymorphicSetDisplay::reset()
{
	MFDClass::reset();
	visuals_.clear();
}

// Set the current color and alpha values for each visual.
void PolymorphicSetDisplay::updateColorAndAlpha()
{
	float alpha = alpha_property_->getFloat();
	Ogre::ColourValue color = color_property_->getOgreColor();

	for( size_t i = 0; i < visuals_.size(); i++ )
	{
		visuals_[ i ]->setColor( color.r, color.g, color.b, alpha );
	}
}

// Set the number of past visuals to show.
void PolymorphicSetDisplay::updateHistoryLength()
{
	visuals_.rset_capacity(history_length_property_->getInt());
}

// This is our callback to handle an incoming message.
void PolymorphicSetDisplay::processMessage( const coterie_msgs::PolymorphicSetVisualizationStamped::ConstPtr& msg )
{
	// Here we call the rviz::FrameManager to get the transform from the
	// fixed frame to the frame in the header of this Imu message.  If
	// it fails, we can't do anything else so we return.
	Ogre::Quaternion orientation;
	Ogre::Vector3 position;
	if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
	                                                msg->header.stamp,
	                                                position, orientation ))
	{
		ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
		           msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
		return;
	}

	// We are keeping a circular buffer of visual pointers.  This gets
	// the next one, or creates and stores it if the buffer is not full
	boost::shared_ptr<PolymorphicSetVisual> visual;
	if( visuals_.full() )
	{
		visual = visuals_.front();
	}
	else
	{
		visual.reset(new PolymorphicSetVisual( context_->getSceneManager(), scene_node_ ));
	}

	// Now set or update the contents of the chosen visual.
	visual->setMessage( msg );
	visual->setFramePosition( position );
	visual->setFrameOrientation( orientation );

	float alpha = alpha_property_->getFloat();
	Ogre::ColourValue color = color_property_->getOgreColor();
	visual->setColor( color.r, color.g, color.b, alpha );

	// And send it to the end of the circular buffer
	visuals_.push_back(visual);
}

} // namespace coterie_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(coterie_rviz_plugin::PolymorphicSetDisplay,rviz::Display )

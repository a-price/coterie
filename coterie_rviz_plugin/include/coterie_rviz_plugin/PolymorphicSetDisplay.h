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

#ifndef SRC_POLYMORPHICSETDISPLAY_H
#define SRC_POLYMORPHICSETDISPLAY_H

#include <boost/circular_buffer.hpp>

#include <coterie_msgs/PolymorphicSetVisualizationStamped.h>
//#include <rviz/message_filter_display.h>
#include <rviz/default_plugin/marker_display.h>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
}

namespace coterie_rviz_plugin
{

enum SET_SAMPLE_STYLE
{
	EXTENTS,
	RANDOM
};

class PolymorphicSetVisual;

class PolymorphicSetDisplay : public rviz::MarkerDisplay
	// : public rviz::MessageFilterDisplay<coterie_msgs::PolymorphicSetVisualizationStamped>
{
Q_OBJECT
public:
	using MsgType = coterie_msgs::PolymorphicSetVisualizationStamped;
	// Constructor.  pluginlib::ClassLoader creates instances by calling
	// the default constructor, so make sure you have one.
	PolymorphicSetDisplay();
	~PolymorphicSetDisplay() override;

	// Overrides of protected virtual functions from Display.  As much
	// as possible, when Displays are not enabled, they should not be
	// subscribed to incoming data and should not show anything in the
	// 3D view.  These functions are where these connections are made
	// and broken.
protected:
	/** @brief Overridden from MarkerDisplay.  Subscribes to the marker
	 * array topic. */
	void subscribe() override;

	/** @brief Overridden from MarkerDisplay.  Unsubscribes to the
	 * marker array topic. */
	void unsubscribe() override;

	// These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:
	void updateStyle();
	void updateColorAndAlpha();
	void updateHistoryLength();

	// Function to handle an incoming ROS message.
private:
	void processMessage( const MsgType::ConstPtr& msg );

	// Storage for the list of visuals.  It is a circular buffer where
	// data gets popped from the front (oldest) and pushed to the back (newest)
	boost::circular_buffer<boost::shared_ptr<PolymorphicSetVisual> > visuals_;
	SET_SAMPLE_STYLE active_style_ = SET_SAMPLE_STYLE::EXTENTS;
	visualization_msgs::MarkerArray::Ptr active_markers_;

	// User-editable property variables.
	rviz::EnumProperty* style_property_;
	rviz::ColorProperty* color_property_;
	rviz::FloatProperty* alpha_property_;
	rviz::IntProperty* history_length_property_;
};
}

#endif //SRC_POLYMORPHICSETDISPLAY_H

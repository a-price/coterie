/**
 * \file visualize_se2.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 2016-6-22
 *
 * \copyright
 *
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <coterie_msgs/RasterSet.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Geometry>

std::string frame; // = "World";
std::string model; // = "package://grasp_capture/LEGO_24_Brick.stl";
ros::Publisher markerPub;
ros::Subscriber r2Sub;
ros::Subscriber r3Sub;
ros::Subscriber se2Sub;

enum SET_TYPE
{
	GOAL,
	CAPTURE,
	FRONTIER,
	DIFF
};

SET_TYPE setType = CAPTURE;
coterie_msgs::RasterSet prevSet;

visualization_msgs::Marker generateMarker(const size_t id)
{
	visualization_msgs::Marker m;
	m.action = visualization_msgs::Marker::ADD;
	m.type = visualization_msgs::Marker::MESH_RESOURCE;
	m.id = id;
	m.header.frame_id = frame;
	m.header.stamp = ros::Time::now();

	m.pose.position.x = 0;
	m.pose.position.y = 0;
	m.pose.position.z = 0;
	m.pose.orientation.w = 1;
	m.pose.orientation.x = 0;
	m.pose.orientation.y = 0;
	m.pose.orientation.z = 0;

	m.scale.x = 1;
	m.scale.y = 1;
	m.scale.z = 1;

	switch (setType)
	{
	case GOAL:
	{
		m.color.a = 1;
		m.color.r = 1;
		m.color.g = 1;
		m.color.b = 0;
		break;
	}
	case FRONTIER:
	{
		m.color.a = 0.1;
		m.color.r = 0;
		m.color.g = 0;
		m.color.b = 1;
		break;
	}
	case DIFF:
	{
		m.color.a = 0.7;
		m.color.r = 1;
		m.color.g = 0;
		m.color.b = 0;
		break;
	}
	default:
	{
		m.color.a = 0.7;
		m.color.r = 0;
		m.color.g = 1;
		m.color.b = 0;
		break;
	}
	}

	m.mesh_resource = model;
	m.frame_locked = true;

	return m;
}

void r2Callback(const coterie_msgs::RasterSetConstPtr ps)
{
	const std_msgs::MultiArrayLayout& layout = ps->byteArray.layout;
	visualization_msgs::MarkerArray ma;

	if (2 != layout.dim.size())
	{
		ROS_ERROR_STREAM("R2 Requires 2 dimensions: <x,y>. Message contains "
		                 << layout.dim.size() << ".");
		return;
	}

	assert(layout.dim[0].stride == ps->byteArray.data.size());

	std::array<Eigen::VectorXd, 2> axis;
	for (size_t d = 0; d < 2; ++d)
	{
		axis[d].setLinSpaced(layout.dim[d].size, ps->extents[d].min, ps->extents[d].max);
	}

	for (size_t i = 0; i < layout.dim[0].size; ++i)
	{
		for (size_t j = 0; j < layout.dim[1].size; ++j)
		{
			size_t idx = layout.data_offset +
			        layout.dim[1].stride * i +
			        j;
			bool isMember = ps->byteArray.data[idx];
			bool wasMember = prevSet.byteArray.data.size() > 0 && prevSet.byteArray.data[idx];
			if (!(isMember || wasMember)) { continue; }

			visualization_msgs::Marker m = generateMarker(idx);

			if ((!isMember) && wasMember)
			{
				// Membership has expired, remove.
				m.action = visualization_msgs::Marker::DELETE;
			}

			m.pose.position.x = axis[0][i];
			m.pose.position.y = axis[1][j];
			m.pose.position.z = 0.0;
			Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
			m.pose.orientation.w = q.w();
			m.pose.orientation.x = q.x();
			m.pose.orientation.y = q.y();
			m.pose.orientation.z = q.z();

			ma.markers.push_back(m);
		}
	}

	markerPub.publish(ma);
	prevSet = *ps;
}

void r3Callback(const coterie_msgs::RasterSetConstPtr ps)
{
	const std_msgs::MultiArrayLayout& layout = ps->byteArray.layout;
	visualization_msgs::MarkerArray ma;

	//layout.data_offset
	if (3 != layout.dim.size())
	{
		ROS_ERROR_STREAM("R3 Requires 3 dimensions: <x,y,z>. Message contains "
		                 << layout.dim.size() << ".");
		return;
	}

	assert(layout.dim[0].stride == ps->byteArray.data.size());

	std::array<Eigen::VectorXd, 3> axis;
	for (size_t d = 0; d < 3; ++d)
	{
		axis[d].setLinSpaced(layout.dim[d].size, ps->extents[d].min, ps->extents[d].max);
	}

	for (size_t i = 0; i < layout.dim[0].size; ++i)
	{
		for (size_t j = 0; j < layout.dim[1].size; ++j)
		{
			for (size_t k = 0; k < layout.dim[2].size; ++k)
			{
				size_t idx = layout.data_offset +
				        layout.dim[1].stride * i +
				        layout.dim[2].stride * j +
				        k;
				bool isMember = ps->byteArray.data[idx];
				bool wasMember = prevSet.byteArray.data.size() > 0 && prevSet.byteArray.data[idx];
				if (!(isMember || wasMember)) { continue; }

				visualization_msgs::Marker m = generateMarker(idx);

				if ((!isMember) && wasMember)
				{
					// Membership has expired, remove.
					m.action = visualization_msgs::Marker::DELETE;
				}

				m.pose.position.x = axis[0][i];
				m.pose.position.y = axis[1][j];
				m.pose.position.z = axis[2][k];
				Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
				m.pose.orientation.w = q.w();
				m.pose.orientation.x = q.x();
				m.pose.orientation.y = q.y();
				m.pose.orientation.z = q.z();

				ma.markers.push_back(m);
			}
		}
	}

	markerPub.publish(ma);
	prevSet = *ps;
}

void se2ModelsCallback(const coterie_msgs::RasterSetConstPtr ps)
{
	const std_msgs::MultiArrayLayout& layout = ps->byteArray.layout;
	visualization_msgs::MarkerArray ma;

	//layout.data_offset
	if (3 != layout.dim.size())
	{
		ROS_ERROR_STREAM("SE2 Requires 3 dimensions: <x,y,theta>. Message contains "
		                 << layout.dim.size() << ".");
		return;
	}

	assert(layout.dim[0].stride == ps->byteArray.data.size());

	std::array<Eigen::VectorXd, 3> axis;
	for (size_t d = 0; d < 3; ++d)
	{
		axis[d].setLinSpaced(layout.dim[d].size, ps->extents[d].min, ps->extents[d].max);
	}

	for (size_t i = 0; i < layout.dim[0].size; ++i)
	{
		for (size_t j = 0; j < layout.dim[1].size; ++j)
		{
			for (size_t k = 0; k < layout.dim[2].size; ++k)
			{
				size_t idx = layout.data_offset +
				        layout.dim[1].stride * i +
				        layout.dim[2].stride * j +
				        k;
				bool isMember = ps->byteArray.data[idx];
				bool wasMember = prevSet.byteArray.data.size() > 0 && prevSet.byteArray.data[idx];
				if (!(isMember || wasMember)) { continue; }

				visualization_msgs::Marker m = generateMarker(idx);

				if ((!isMember) && wasMember)
				{
					// Membership has expired, remove.
					m.action = visualization_msgs::Marker::DELETE;
				}

				m.pose.position.x = axis[0][i];
				m.pose.position.y = axis[1][j];
				m.pose.position.z = axis[2][k]*0.075;//0.0;
				Eigen::Quaterniond q(Eigen::AngleAxisd(axis[2][k], Eigen::Vector3d::UnitZ()));
				m.pose.orientation.w = q.w();
				m.pose.orientation.x = q.x();
				m.pose.orientation.y = q.y();
				m.pose.orientation.z = q.z();

				ma.markers.push_back(m);
			}
		}
	}

	markerPub.publish(ma);
	prevSet = *ps;
}

void se2PosesCallback(const coterie_msgs::RasterSetConstPtr ps)
{
	const std_msgs::MultiArrayLayout& layout = ps->byteArray.layout;
	visualization_msgs::MarkerArray ma;

	//layout.data_offset
	if (3 != layout.dim.size())
	{
		ROS_ERROR_STREAM("SE2 Requires 3 dimensions: <x,y,theta>. Message contains "
		                 << layout.dim.size() << ".");
		return;
	}

	assert(layout.dim[0].stride == ps->byteArray.data.size());
	assert(layout.dim[0].label == "theta");
	assert(layout.dim[1].label == "x");
	assert(layout.dim[2].label == "y");

	// Configure the Point and Line markers
	double spacing = (ps->extents[1].max-ps->extents[1].min)/(2*layout.dim[1].size);
	//double zscale = spacing/((ps->extents[0].max-ps->extents[0].min)/(2*layout.dim[0].size));
	double zscale = 0.0;

	visualization_msgs::Marker pointMarker = generateMarker(0);
	pointMarker.type = visualization_msgs::Marker::SPHERE_LIST;
	pointMarker.scale.x = spacing*5.0/(5.0+2.0*static_cast<double>(setType));
	pointMarker.scale.y = pointMarker.scale.x;
	pointMarker.scale.z = pointMarker.scale.x;
	pointMarker.color.a = 1;
	pointMarker.ns = "point";
	visualization_msgs::Marker lineMarker = generateMarker(1);
	lineMarker.type = visualization_msgs::Marker::LINE_LIST;
	lineMarker.scale.x = spacing/10.0;
	lineMarker.scale.y = 0;
	lineMarker.scale.z = 0;
	lineMarker.color.a = 1;
	lineMarker.ns = "line";

	std::array<Eigen::VectorXd, 3> axis;
	for (size_t d = 0; d < 3; ++d)
	{
		axis[d].setLinSpaced(layout.dim[d].size, ps->extents[d].min, ps->extents[d].max);
	}

	for (size_t i = 0; i < layout.dim[0].size; ++i)
	{
		for (size_t j = 0; j < layout.dim[1].size; ++j)
		{
			for (size_t k = 0; k < layout.dim[2].size; ++k)
			{
				size_t idx = layout.data_offset +
				        layout.dim[1].stride * i +
				        layout.dim[2].stride * j +
				        k;
				bool isMember = ps->byteArray.data[idx];
				if (!isMember) { continue; }

				geometry_msgs::Point p;
				p.z = axis[0][i] * zscale;
				p.x = axis[1][j];
				p.y = axis[2][k];

				pointMarker.points.push_back(p);
				lineMarker.points.push_back(p);

				Eigen::Quaterniond q(Eigen::AngleAxisd(axis[0][i], Eigen::Vector3d::UnitZ()));
				Eigen::Vector3d dir = q * (Eigen::Vector3d::UnitX() * spacing);
				p.x += dir.x();
				p.y += dir.y();
				p.z += 0.0;
				lineMarker.points.push_back(p);
			}
		}
	}

	ma.markers.push_back(pointMarker);
	ma.markers.push_back(lineMarker);
	markerPub.publish(ma);
	prevSet = *ps;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "visualize_set");
	ros::NodeHandle nh, pnh("~");

	if (!nh.getParam("target_model", model))
	{
		ROS_FATAL("No target model provided, exiting.");
		return -1;
	}

	if (!nh.getParam("support_frame", frame))
	{
		ROS_FATAL("Name of support surface reference frame not provided, exiting.");
		return -1;
	}

	bool posesOnly = pnh.param<bool>("poses_only", false);

	std::string setTypeStr = "";
	pnh.param("set_type", setTypeStr, std::string("capture"));
	if ("capture" == setTypeStr)
	{
		setType = CAPTURE;
	}
	else if ("goal" == setTypeStr)
	{
		setType = GOAL;
	}
	else if ("frontier" == setTypeStr)
	{
		setType = FRONTIER;
	}
	else if ("diff" == setTypeStr)
	{
		setType = DIFF;
	}
	else
	{
		ROS_FATAL("Optional 'set_type' parameter must be one of the following: { 'capture', 'goal', 'frontier', 'diff'}.");
		return -1;
	}

	markerPub = nh.advertise<visualization_msgs::MarkerArray>("set_markers", 1, true);
	r2Sub = nh.subscribe<coterie_msgs::RasterSet>("r2", 1, &r2Callback);
	r3Sub = nh.subscribe<coterie_msgs::RasterSet>("r3", 1, &r3Callback);
	if (posesOnly)
	{
		se2Sub = nh.subscribe<coterie_msgs::RasterSet>("se2", 1, &se2PosesCallback);
	}
	else
	{
		se2Sub = nh.subscribe<coterie_msgs::RasterSet>("se2", 1, &se2ModelsCallback);
	}

	ros::spin();
	return 0;
}

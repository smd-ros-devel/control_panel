/*
 * Copyright (c) 2011, 2012 SDSM&T RIAS.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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


/**
 * \file   qt_node.cpp
 * \date   Feb 3, 2012
 * \author Matt Richard
 */
#include "control_panel/nodes/qt_node.h"
#include "ros/network.h"
#include <string.h>
#include <stdio.h>

#include <iostream>

QtNode::QtNode(int argc, char **argv)
{
    char *span;

    // Store all command line arguments in a map
    for(int i = 1; i < argc; i++)
    {
        span = strstr(argv[i], ":=");
        if(span != NULL)
        {
            char *key = (char *)malloc(strlen(argv[i]));

            strncpy(key, argv[i], span - argv[i]);

            remappings[key] = span + 2;

            free(key);
        }
    }

    node_name = "qt_node";
    master_uri = "http://localhost:11311";
}

QtNode::~QtNode()
{
    stop();
}

std::string QtNode::getMasterURI() const
{
    return master_uri;
}

std::string QtNode::getHostIP() const
{
    return host_ip;
}

std::string QtNode::getNodeName() const
{
    return node_name;
}

bool QtNode::init(bool use_env_vars)
{
    if(use_env_vars)
    {
        remappings.erase("__master");
        remappings.erase("__ip");
    }
    else
    {
        if(master_uri != "")
            remappings["__master"] = master_uri;
        if(host_ip != "")
            remappings["__ip"] = host_ip;
    }

    std::map<std::string, std::string>::iterator it;
    it = remappings.find("__master");
    if(it != remappings.end())
        std::cout << "__master = " << it->second << std::endl;

    it = remappings.find("__ip");
    if(it != remappings.end())
        std::cout << "__ip = " << it->second << std::endl;

    printf("Initializing ROS node\n");
    ros::init(remappings, node_name);

    // Verify we have connection with the master
    if(!ros::master::check())
    {
        printf("Could not connect to master\n");
        return false;
    }

    printf("Starting global thread\n");
    ros::start();
    start(); // start thread

    return true;
}

void QtNode::run()
{
    ros::NodeHandle nh;

    ros::spin();
}

void QtNode::stop()
{
    // Shutdown the node if it's running
    if(ros::isStarted())
    {
        ROS_INFO("Shutting down node");
        ros::shutdown();
        ros::waitForShutdown();
        wait();
    }
}

void QtNode::setMasterURI(const std::string &uri)
{
    master_uri = uri;
}

void QtNode::setHostIP(const std::string &ip)
{
    host_ip = ip;
}

void QtNode::setNodeName(const std::string &name)
{
    if(name != "")
        node_name = name;
}

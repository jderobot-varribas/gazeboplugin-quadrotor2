/*
 *  Copyright (C) 1997-2015 JDE Developers Team
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/.
 *  Authors :
 *       Victor Arribas Raigadas <v.arribas.urjc@gmai.com>
 */

#include <quadrotor/interfaces/camerai.h>

using namespace quadrotor::interfaces;
using namespace jderobot;


CameraI::CameraI (const QuadRotorSensors *sensor):
    sensor(sensor)
{
    cameraSensorConnection = sensor->cam_ventral->ConnectUpdated(
                boost::bind(&CameraI::onCameraSensorBoostrap, this));
}

CameraI::~CameraI ()
{}


void
CameraI::onCameraSensorBoostrap(){
    if (sensor->img_ventral.empty())
        return;

    std::cout<<"CameraI::onCameraSensorBoostrap()"<<std::endl;

    sensor->cam_ventral->DisconnectUpdated(cameraSensorConnection);

    imgCached = sensor->img_ventral.clone();

    imageDescription = new ImageDescription();
    imageDescription->format = "RGB8";// colorspaces::ImageRGB8::FORMAT_RGB8->name();
    imageDescription->height = imgCached.rows;
    imageDescription->width  = imgCached.cols;
    imageDescription->size = imgCached.rows*imgCached.cols*3;

    imageData = new ImageData();
    imageData->description = imageDescription;
    imageData->pixelData.resize(imageDescription->size);
    memcpy(imageData->pixelData.data(), imgCached.data, imageData->pixelData.size());

    imageFormats.push_back(imageDescription->format);

    cameraSensorConnection = sensor->cam_ventral->ConnectUpdated(
                boost::bind(&CameraI::onCameraSensorUpdate, this));
}

void
CameraI::onCameraSensorUpdate(){
    // thread unsafe with gazebo (?)
    imgCached = sensor->img_ventral.clone();

    // thread unsafe with ice
    memcpy(imageData->pixelData.data(), imgCached.data, imageData->pixelData.size());
}

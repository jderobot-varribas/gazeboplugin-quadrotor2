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



#include "quadrotor/quadrotorplugin.hh"

GZ_REGISTER_MODEL_PLUGIN(quadrotor::QuadrotorPlugin)

using namespace quadrotor;
using namespace gazebo::physics;
using namespace gazebo::math;
using namespace gazebo::event;
using namespace gazebo::common;


QuadrotorPlugin::QuadrotorPlugin()
{

}


void
QuadrotorPlugin::Load(ModelPtr _model, sdf::ElementPtr _sdf){
    model = _model;
    sensors.Load(model);
    control.Load(model->GetLink(), _sdf);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnection = Events::ConnectWorldUpdateBegin(
        boost::bind(&QuadrotorPlugin::OnUpdate, this, _1));

//    this->InitializeIce(_sdf);
}


void
QuadrotorPlugin::Init(){
std::cout << "QuadrotorPlugin::Init()" << std::endl;
    sensors.debugInfo();
    sensors.Init();

    sensors.cam_frontal->SetActive(true);
    sensors.cam_ventral->SetActive(true);
    sensors.sonar->SetActive(true);
    sensors.imu->SetActive(true);

    control.Init();

//    icePlugin->start();
}


void
QuadrotorPlugin::OnUpdate(const UpdateInfo & _info){
//    std::cout << "QuadrotorPlugin::OnUpdate()" << std::endl;
//    std::cout << "\t" << _info.simTime << std::endl;
}


void
QuadrotorPlugin::Reset(){

}



void
QuadrotorPlugin::InitializeIce(sdf::ElementPtr _sdf){
    std::cout << "QuadrotorPlugin::InitializeIce()" << std::endl;
    std::string iceConfigFile = "quadrotorplugin.cfg";
    if(_sdf->HasElement("cfgFile"))
        iceConfigFile =  _sdf->GetElement("cfgFile")->GetValue()->GetAsString();
    std::cout << "\tconfig: "<< iceConfigFile << std::endl;
#if 1
    Ice::StringSeq args;
    args.push_back("--Ice.Config=" + iceConfigFile);
    Ice::CommunicatorPtr ic = Ice::initialize(args);
#else
    Ice::InitializationData id;
    id.properties->load(iceConfigFile);
    Ice::CommunicatorPtr ic = Ice::initialize(id);
#endif
    std::cout << "\tcreate Ice plugin..." << std::endl;
    icePlugin = QuadrotorIcePtr(new QuadrotorIce(ic, &sensors, &control));
}

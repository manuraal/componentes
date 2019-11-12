/*
 *    Copyright (C)2019 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

enum Estados
{
	encontrado,
	rotTo,
	goTo,
	obstacle
};

Estados estado = encontrado;
float rot = 0.6;

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	emit computetofinalize();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	//   THE FOLLOWING IS JUST AN EXAMPLE
	// To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innerModel = std::make_shared<InnerModel>(innermodel_path);
	}
	catch (std::exception e)
	{
		qFatal("Error reading config params");
	}

	//defaultMachine.start();

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(Period);
	emit this->initializetocompute();
}

void SpecificWorker::compute()
{
	differentialrobot_proxy->getBaseState(bState);
	ldata = laser_proxy->getLaserData();
	innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
	switch (estado)
	{
	case encontrado:
		if (target.picked)
			estado = rotTo;
		break;
	case rotTo:
		rotToTarget();
		break;
	case goTo:
		goToTarget();
		break;
	case obstacle:
		obstaculo();
		break;
	}
}

void SpecificWorker::rotToTarget()
{
	QVec rt = innerModel->transform("base", QVec::vec3(target.x, 0, target.z), "world");
	float dist = rt.norm2();
	float rot = atan2(rt.x(), rt.z());

	if (dist < 100)
	{
		estado = encontrado;
		target.picked = true;
		return;
	}

	if (fabs(rot) > 0.05)
		dist = 0;

	differentialrobot_proxy->setSpeedBase(0, rot);

	if (fabs(rot) <= 0.01)
	{
		differentialrobot_proxy->setSpeedBase(0, 0);
		estado = goTo;
	}
}

void SpecificWorker::goToTarget()
{
	RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
	std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });
	QVec rt = innerModel->transform("base", QVec::vec3(target.x, 0, target.z), "world");
	float dist = rt.norm2();
	if (dist < 100)
	{
		differentialrobot_proxy->setSpeedBase(0, 0);
		target.picked = false;
		estado = encontrado;
		return;
	}
	differentialrobot_proxy->setSpeedBase(300, 0);

	//Si encontramos un obstáculo
	if (ldata.front().dist < 200)
	{
		estado = obstacle;
	}
}

void SpecificWorker::obstaculo()
{
	try
	{
		float giro = 0.6;
		RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
		//std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });
		int giroIzq = 0;
		for (int i = 0; i < 49; i++)
		{
			giroIzq = giroIzq + ldata[i].dist;
		}

		int giroDer = 0;
		for (int i = 50; i < 100; i++)
		{
			giroDer = giroDer + ldata[i].dist;
		}

		differentialrobot_proxy->setSpeedBase(0, 0);
		if (giroIzq < giroDer)
		{

			differentialrobot_proxy->setSpeedBase(0, -giro);
		}
		else
		{
			differentialrobot_proxy->setSpeedBase(0, giro);
		}

		ldata = laser_proxy->getLaserData();
		std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });
		//si estamos lo suficientemente lejos del obstaculo
		if (ldata.front().dist >= 200)
		{
			differentialrobot_proxy->setSpeedBase(300, 0);
		}
	}
	catch (const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
}

void SpecificWorker::sm_compute()
{
	std::cout << "Entered state compute" << std::endl;
	compute();
}

void SpecificWorker::sm_initialize()
{
	std::cout << "Entered initial state initialize" << std::endl;
}

void SpecificWorker::sm_finalize()
{
	std::cout << "Entered final state finalize" << std::endl;
}

void SpecificWorker::RCISMousePicker_setPick(Pick myPick)
{
	target.x = myPick.x;
	target.z = myPick.z;
	target.picked = true;
}

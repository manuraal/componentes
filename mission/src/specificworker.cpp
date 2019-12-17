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
	IDLE,
	TURN,
	CHECKTAG,
	GOTO,
	WAIT
};

Estados estado = IDLE; //Estado inicial

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
	//       THE FOLLOWING IS JUST AN EXAMPLE
	//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
	//	try
	//	{
	//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
	//		std::string innermodel_path = par.value;
	//		innerModel = new InnerModel(innermodel_path);
	//	}
	//	catch(std::exception e) { qFatal("Error reading config params"); }

	defaultMachine.start();

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
	std::cout << "Estado =";
	std::cout << estado << std::endl;
	switch (estado)
	{
	case IDLE:
		idle();
		break;
	case TURN:
		turn();
		break;
	case CHECKTAG:
		checktag();
		break;
	case GOTO:
		gotoTarget();
		break;
	case WAIT:
		waiting();
		break;
	}
}

void SpecificWorker::idle()
{
	estado = TURN;
}

void SpecificWorker::turn()
{
	try
	{
		gotopoint_proxy->turn(0.3);
		if (t.read().empty() == false)
		{
			gotopoint_proxy->stop();
			std::cout << "Cambio a goto" << std::endl;
			estado = GOTO;
		}
	}
	catch (const std::exception &e)
	{
		std::cerr << e.what() << '\n';
	}
}

void SpecificWorker::gotoTarget()
{
	try
	{
		std::cout << "Entro en gototarget" << std::endl;
		auto [nodo, x, y, alpha] = t.read().front();
		gotopoint_proxy->go("", x, y, alpha);
		estado = WAIT;
	}
	catch (const std::exception &e)
	{
		std::cerr << e.what() << '\n';
	}
}

void SpecificWorker::waiting()
{
	try
	{
		if (gotopoint_proxy->atTarget())
		{
			gotopoint_proxy->stop();
			estado = IDLE;
		}
		else
		{
			std::cout << "ESPERANDO" << std::endl;
			usleep(1000000);
		}
	}
	catch (const std::exception &e)
	{
		std::cerr << e.what() << '\n';
	}
}
void SpecificWorker::checktag()
{
}

void SpecificWorker::sm_compute()
{
	//std::cout << "Entered state compute" << std::endl;
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

////////////////////////////////////////////
//  subscription
/////////////////////////////////////////

void SpecificWorker::AprilTags_newAprilTagAndPose(tagsList tags, RoboCompGenericBase::TBaseState bState, RoboCompJointMotor::MotorStateMap hState)
{
	//subscribesToCODE
}

void SpecificWorker::AprilTags_newAprilTag(tagsList tags)
{
	std::vector<Tp> tps;
	for (const auto &t : tags)
	{
		tps.push_back(std::make_tuple(t.id, t.tx, t.tz, t.ry));
		qDebug() << t.id;
	}
	t.write(tps);
}

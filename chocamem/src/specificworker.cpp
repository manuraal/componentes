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

const float threshold = 200; // millimeters
float rot = 0.6;			 // rads per second

enum Estados
{
	base,
	obstaculo,
	espiral
};
bool derecha = false;
bool izquierda = false;
Estados estado = espiral;
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx &mprx) : GenericWorker(mprx)
{
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
	innerModel = std::make_shared<InnerModel>(par.value);
	xmin = std::stoi(params.at("xmin").value);
	xmax = std::stoi(params.at("xmax").value);
	ymin = std::stoi(params.at("ymin").value);
	ymax = std::stoi(params.at("ymax").value);
	tilesize = std::stoi(params.at("tilesize").value);

	// Scene
	scene.setSceneRect(xmin, ymin, fabs(xmin) + fabs(xmax), fabs(ymin) + fabs(ymax));
	view.setScene(&scene);
	view.scale(1, -1);
	view.setParent(scrollArea);
	view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio);
	grid.initialize(TDim{tilesize, xmin, xmax, ymin, ymax}, TCell{true, false, nullptr});

	qDebug() << "Grid initialize ok";

	for (auto &[key, value] : grid)
	{
		value.rect = scene.addRect(-tilesize / 2, -tilesize / 2, 100, 100, QPen(Qt::NoPen));
		value.rect->setPos(key.x, key.z);
	}

	robot = scene.addRect(QRectF(-200, -200, 400, 400), QPen(), QBrush(Qt::blue));
	noserobot = new QGraphicsEllipseItem(-50, 100, 100, 100, robot);
	noserobot->setBrush(Qt::magenta);

	view.show();
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	this->Period = period;
	timer.start(Period);
	qDebug() << "End initialize";
}

void SpecificWorker::compute()
{
	readRobotState();

	//MAQUINA DE ESTADOS
	switch (estado)
	{

	//CASO BASE:
	case base:
		casoBase();
		break;

	//CASO OBSTACULO:
	case obstaculo:
		casoObstaculo();
		break;

	//CASO ESPIRAL:
	case espiral:
		casoEspiral();
		break;
	}
}

/**
 * Caso el cual hace que el robot 
 * se mueva en línea recta hasta
 * que encuentre un obstáculo
 **/
void SpecificWorker::casoBase()
{
	try
	{
		rot = 0.6;
		RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
		std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });
		if (ldata.front().dist < threshold)
		{
			estado = obstaculo;
		}
		else
		{
			differentialrobot_proxy->setSpeedBase(500, 0);
		}
	}
	catch (const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
}

/**
 * Caso que va a realizarse cuando 
 * la distancia del robor al obstáculo 
 * sea menor que el umbral establecido (threshold)  
 **/
void SpecificWorker::casoObstaculo()
{
	try
	{
		rot = 0.6;
		RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
		//std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });
		for (int i = 46; i < 99; i++)
		{
			if (ldata[i].dist < threshold)
			{
				izquierda = true;
			}
		}
		if (izquierda)
		{
			differentialrobot_proxy->setSpeedBase(0, 0.3);
			usleep(500000);
			izquierda = false;
			estado = espiral;
		}
		for (int i = 0; i < 45; i++)
		{
			if (ldata[i].dist < threshold)
			{
				derecha = true;
			}
		}

		if (derecha)
		{
			differentialrobot_proxy->setSpeedBase(0, !0.3);
			usleep(500000);
			derecha = false;
			estado = espiral;
		}
		//else
		// {
		// 	differentialrobot_proxy->setSpeedBase(0, 0.3);
		// 	estado = base;
		// }

		// if (ldata.front().dist > threshold)
		// {

		// }
	}
	catch (const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
}

/**
 * Caso que va a ejecutarse 
 * cuando el robot encuentre
 * un obstáculo
 **/
void SpecificWorker::casoEspiral()
{

	try
	{
		if (rot > 0)
			rot = rot - 0.05;
		else
		{
			//rot = 0.6;
			estado = base;
		}
		RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
		std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });
		if (ldata.front().dist < threshold)
		{
			// rot = 0.6;
			estado = obstaculo;
		}
		else
		{
			differentialrobot_proxy->setSpeedBase(300, rot);
			usleep(500000);
		}
	}
	catch (const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
}

void SpecificWorker::readRobotState()
{
	try
	{
		differentialrobot_proxy->getBaseState(bState);
		auto [valid, cell] = grid.getCell(bState.x, bState.z);
		//Compruba que la celda que pisa el robot es válida y la marca como visitada.
		if (valid)
		{
			// if (cell.visited)
			// {
			// 	std::cout << "CELDA YA VISITADA" << std::endl;
			// }
			cell.visited = true;
			cell.rect->setBrush(Qt::green);
		}

		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
		RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();

		//draw robot
		robot->setPos(bState.x, bState.z);
		robot->setRotation(-180. * bState.alpha / M_PI);

		//update  occupied cells
		updateOccupiedCells(bState, ldata);
	}
	catch (const Ice::Exception &e)
	{
		std::cout << "Error reading from Laser" << e << std::endl;
	}
	//Resize world widget if necessary, and render the world
	if (view.size() != scrollArea->size())
		view.setFixedSize(scrollArea->width(), scrollArea->height());
}

void SpecificWorker::updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata)
{
	InnerModelLaser *n = innerModel->getNode<InnerModelLaser>(QString("laser"));
	for (auto l : ldata)
	{
		auto r = n->laserTo(QString("world"), l.dist, l.angle); // r is in world reference system
		// we set the cell corresponding to r as occupied
		auto [valid, cell] = grid.getCell(r.x(), r.z());
		if (valid)
		{
			cell.free = false;
			cell.rect->setBrush(Qt::darkRed);
		}
	}
}

///////////////////////////////////////////////////////////////////77
////  SUBSCRIPTION
/////////////////////////////////////////////////////////////////////

void SpecificWorker::RCISMousePicker_setPick(const Pick &myPick)
{
	//subscribesToCODE
}

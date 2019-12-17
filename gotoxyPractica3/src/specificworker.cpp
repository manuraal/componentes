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
	located,
	rotTo,
	goTo,
	obstacle,
	surround,
	goToPoint
};

Estados estado = located; //Estado inicial
float rot = 0.4;		  //Velocidad de rotacion
int threshold = 290;	  //Limite de mm hasta un obstáculo
float a, b, c;			  //Coeficientes para la recta

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
	innerModel->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
	switch (estado)
	{
	case located:
		if (target.picked)
		{
			CalculoRecta(); //Calculamos la recta solo cuando pinchamos sobre el mundo
			estado = rotTo;
		}
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
	case surround:
		rodear();
		break;
	case goToPoint:
		irAlPunto();
		break;
	}
}

/**
 * Método para girar hacia el target
 * Primeramente se comprueba si estamos cerca de un obstaculo, si es asi pasamos al estado obstaculo
 * Despues comprobamos que no estemos en el punto seleccionado, si lo estamos, el robot se para, pasamos al estado located
 * Por ultimo si no estamos ni cerca de un obstaculo ni en el punto, vamos hacia el punto con el estado goTo
 **/
void SpecificWorker::rotToTarget()
{
	std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });
	QVec rt = innerModel->transform("robot", QVec::vec3(target.x, 0, target.z), "world");
	float dist = rt.norm2();
	float rot = atan2(rt.x(), rt.z());

	std::cout << "Estoy en el rotToTarget" << std::endl;

	if (ldata.front().dist < threshold)
	{
		estado = obstacle;
	}

	if (dist < 100)
	{
		estado = located;
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

/**
 * Método para it hacia el target seleccionado 
 * Comprobamos si estamos en el punto, si estamos, vamos al estado located
 * Si no, si encontramos un obstaculo pasamos al estado obstacle
 * Si no ocurre ninguna de las dos opciones anteriores simplemente vamos hacia delante
 **/
void SpecificWorker::goToTarget()
{
	RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
	std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });
	QVec rt = innerModel->transform("robot", QVec::vec3(target.x, 0, target.z), "world");
	float dist = rt.norm2();
	std::cout << "Estoy en el goToTarget" << std::endl;

	if (dist < 100)
	{
		differentialrobot_proxy->setSpeedBase(0, 0);
		target.picked = false;
		estado = located;
		return;
	}
	if (ldata.front().dist < threshold)
	{
		estado = obstacle;
	}
	differentialrobot_proxy->setSpeedBase(200, 0);
}

/**
 * Método para no chocar contra un obstáculo y rodearlo
 * Primeramente nos paramos frente al obstáculo y despues comprobamos la distancia a este,
 * si la distancia frontal del láser es mayor o igual a nuestro threshold pasamos al estado surround para rodear el obsatáculo
 **/
void SpecificWorker::obstaculo()
{
	std::cout << "Estoy en el obstaculo" << std::endl;
	try
	{
		differentialrobot_proxy->setSpeedBase(0, rot);
		ldata = laser_proxy->getLaserData();
		std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });
		if (ldata.front().dist >= threshold)
		{
			estado = surround;
		}
	}
	catch (const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
}

/**
 * Método 
 **/
void SpecificWorker::rodear()
{
	std::cout << "Estoy en el rodear" << std::endl;
	try
	{
		auto vector = std::min(ldata.begin(), ldata.end() - 1, [](auto &&a, auto &&b) { return (*a).dist < (*b).dist; });
		if ((*vector).dist <= threshold + 190)
		{
			std::cout << "No deberia de girar pero giro" << std::endl;

			differentialrobot_proxy->setSpeedBase(100, rot);
		}
		else if ((*vector).dist > threshold + 190)
		{
			std::cout << "No deberia de girar pero giro aqui" << std::endl;

			differentialrobot_proxy->setSpeedBase(100, -rot);
			if (Visible())
			{
				estado = goToPoint;
			}
		}

		comprobarRecta(a, b, c);
	}
	catch (const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
}

bool SpecificWorker::Visible()
{
	QPolygonF polygon;
	auto laser = innerModel->getNode<InnerModelLaser>(std::string("laser"));

	for (int i = 1; i < 80; i++)
	{
		QVec lr = laser->laserTo(std::string("world"), ldata[i].dist, ldata[i].angle);
		polygon << QPointF(lr.x(), lr.z());
	}

	QVec t = QVec::vec3(target.x, 0, target.z);
	return polygon.containsPoint(QPointF(t.x(), t.z()), Qt::WindingFill);
}

void SpecificWorker::irAlPunto()
{
	estado = rotTo;
}

//ax + by + c = 0
void SpecificWorker::CalculoRecta()
{
	/**
	 * y2 - y1       y - y1
	 * --------  =  --------
	 * x2 - x1       x - x1
	 */
	float num = bState.z - target.z;
	float den = bState.x - target.x;
	a = num;
	b = -den;
	c = -(num * bState.x) + (den * bState.z);
}

void SpecificWorker::comprobarRecta(float a, float b, float c)
{
	float res = a * bState.x + b * bState.z + c;
	float norm = res / (sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2)));
	if (norm >= 0 && norm <= 0.05)
	{
		estado = rotTo;
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

void SpecificWorker::GotoPoint_go(string nodo, float x, float y, float alpha)
{
	auto r = innerModel->transform("world", QVec::vec3(x, 0, y), "rgbd");
	target.x = r.x();
	target.z = r.z();
	std::cout << "Target x = ";
	std::cout << target.x << std::endl;
	std::cout << "Target z = ";
	std::cout << target.z << std::endl;
	target.picked = true;
}

void SpecificWorker::GotoPoint_turn(float speed)
{
	if (speed < 1 && speed > (-1))
	{
		differentialrobot_proxy->setSpeedBase(0, speed);
	}
	else
	{
		std::cout << "ERROR EN EL TURN" << std::endl;
	}
}

bool SpecificWorker::GotoPoint_atTarget()
{
	return !target.picked;
}

void SpecificWorker::GotoPoint_stop()
{
	try
	{
		differentialrobot_proxy->setSpeedBase(0, 0);
	}
	catch (const std::exception &e)
	{
		std::cerr << e.what() << '\n';
	}
}

void SpecificWorker::RCISMousePicker_setPick(Pick myPick)
{
	target.x = myPick.x;
	target.z = myPick.z;
	target.picked = true;
}

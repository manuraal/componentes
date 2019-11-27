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

/**
       \brief
       @author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <iostream>
#include <fstream>
#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <QPolygon>
#include <Qt>
#include <math.h>

class SpecificWorker : public GenericWorker
{
	Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void RCISMousePicker_setPick(Pick myPick);

	struct Target
	{
		bool picked;
		float x;
		float z;
	};

	Target target;

public slots:
	void compute();
	void initialize(int period);
	void rotToTarget();
	void goToTarget();
	void obstaculo();
	void rodear();
	void irAlPunto();
	bool Visible();
	void CalculoRecta();
	void comprobarRecta(float a, float b, float c);
	//Specification slot methods State Machine
	void sm_compute();
	void sm_initialize();
	void sm_finalize();

	//--------------------
private:
	std::shared_ptr<InnerModel> innerModel;
	RoboCompGenericBase::TBaseState bState;
	RoboCompLaser::TLaserData ldata;
};

#endif

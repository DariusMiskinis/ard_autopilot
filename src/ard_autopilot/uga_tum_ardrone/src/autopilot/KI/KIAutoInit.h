#pragma once
 /**
 *  This file is part of uga_tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  Portions Copyright 2015 Kenneth Bogert <kbogert@uga.edu> and Sina Solaimanpour <sina@uga.edu> (THINC Lab, University of Georgia)
 *  For more information see <https://vision.in.tum.de/data/software/uga_tum_ardrone>.
 *
 *  uga_tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  uga_tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with uga_tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __KIAUTOINIT_H
#define __KIAUTOINIT_H

#include "KIProcedure.h"

class KIAutoInit : public KIProcedure
{
private:
	enum {NONE, STARTED, WAIT_FOR_FIRST, TOOK_FIRST, WAIT_FOR_SECOND, DONE} stage;
	int stageStarted;
	bool nextUp;
	bool resetMap;
	int moveTimeMS;
	int waitTimeMS;
	int reachHeightMS;
	float controlCommandMultiplier;
public:
	KIAutoInit(bool resetMap = true, int imoveTimeMS=500, int iwaitTimeMS=800, int reachHeightMS=6000, float controlMult = 1.0, bool takeoff=true);
	~KIAutoInit(void);
	bool update(const uga_tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KIAUTOINIT_H */

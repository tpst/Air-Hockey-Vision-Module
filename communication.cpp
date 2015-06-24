#include "visModule.h"

#include <iostream>
#include <string>

string puckTracker::constructMessage(Puck puck)
{
	ostringstream puckMessage;

	puck.x_err = 0;
	puck.y_err = 0;
	puck.v_err = 0;

	if(puck.velocity < 0.015) puck.velocity = 0, puck.flag = 0;

	puckMessage << 0 << " "; 
	puckMessage << (int)(puck.position.x/mmratio) << " ";
	puckMessage << (int)(puck.position.y/mmratio) << " ";
	puckMessage << (int)puck.velocity << " ";
	puckMessage << (int)puck.x_err << " ";
	puckMessage << (int)puck.y_err << " ";
	puckMessage << (int)puck.v_err << " ";
	puckMessage << (int)puck.flag << endl;



	string puckMsg = puckMessage.str();

	//cout << puckMsg << endl;
	return puckMsg;
}

string puckTracker::constructMessage(Prediction p)
{
	ostringstream predictMessage;
	
	predictMessage << 1 << " ";
	predictMessage << (int)(p.distance/mmratio) << " ";
	predictMessage << p.angle << " ";
	predictMessage << (int)p.eta << " ";
	predictMessage << (int)p.velocity << " ";
	predictMessage << (int)p.d_err << " ";
	predictMessage << (int)p.a_err << " ";
	predictMessage << (int)p.t_err << " ";
	predictMessage << (int)p.v_err << endl;

	string predictMsg = predictMessage.str();
	
	//cout << predictMsg << endl;

	return predictMsg;
}



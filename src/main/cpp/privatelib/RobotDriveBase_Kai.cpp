/*---------------------------Modified Private Lib-----------------------------*/

#include "privatelib/RobotDriveBase_Kai.h"

double ApplyDeadband_Kai(double &tar, double deadband) {
	if((tar < deadband && tar > (-1) * deadband) ||
		(tar > deadband && tar < (-1) * deadband)) { tar = 0; }
	return tar;
}

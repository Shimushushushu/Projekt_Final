/*---------------------------Modified Private Lib-----------------------------*/

#include "privatelib/DualXtremeControl.h"

double xCombine(double Lx, double Rx) { return (Lx + Rx)/2; }

double yCombine(double Ly, double Ry) { return (Ly + Ry)/2; }

double zCombine(double Lx, double Rx) { return (Lx - Rx)/2; }
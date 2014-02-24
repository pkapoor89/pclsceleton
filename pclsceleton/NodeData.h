/*	Author: Andrew Hajj
	Date: 1/17/14
	Name: NodeData.h
*/

#ifndef NODEDATA_H
#define NODEDATA_H

using namespace std;
#include "TherapyCenter.h"

class NodeData
{

private:
	//x,y,z coordinates
	double x;
	double y;
	double z;
	Side side;
	Limb limb;
	long time;

public:
	
	//Default Construct
	NodeData() : side(Right), limb(Hand)	{};
		
	//Constructor
	NodeData(double x_coord, double y_coord, double z_coord, Side inputSide, Limb inputLimb, long inputTime)	
	: x(x_coord), y(y_coord), z(z_coord), side(inputSide), limb(inputLimb), time(inputTime) {};

	//Accessors
	void SetParameters(const double x_cor, const double y_cor, const double z_cor, const Side inSide, const Limb inLimb, const long inTime)
	{x = x_cor; y = y_cor; z = z_cor; side = inSide; time = inTime;};
	double X(void) const {return x;};	//access x
	double Y(void) const {return y;};	//access y
	double Z(void) const {return z;};	//access z
	Side getSide(void) const {return side;};	//access side
	Limb getLimb(void) const {return limb;};	//access limb
	long Time(void) const {return time;};	//access time stamp


	//'Set' operator
	NodeData& operator=(const NodeData& rhs);
 };



#endif

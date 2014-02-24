/*	Author: Andrew Hajj
	Date: 12/30/13
	Name: TherapyCenter.h
	Purpose: The center module for scoring a user's movements during a therapy session.
*/

#ifndef THERAPYCENTER_H
#define THERAPYCENTER_H

#include <iostream>
using namespace std;

// enums for Therapy, Limb and Side
enum Therapy { Stroke };
enum Type { MotorArm, MotorLeg };
enum Limb { Head, Neck, Shoulder, Elbow, Hand, Hip, Knee, Arm, Leg, Ankle, Foot, Wrist, Torso, Waist };
enum Side { Right, Left, Neither };

//Function get the score
int getScore(Therapy therapy, Side side, Type therapyType);

//overloaded operators for the enums
istream& operator >> (istream& i, Limb& limb);
istream& operator >> (istream& i, Side& side);

#endif

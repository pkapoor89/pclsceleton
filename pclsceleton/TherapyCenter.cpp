/*	Author: Andrew Hajj
	Date: 12/30/13
	Name: TherapyCenter.cpp
	Purpose: The center module for scoring a user's movements during a therapy session.
*/

#include <iostream>
#include <string>
#include "TherapyCenter.h"
#include "Stroke.h"

using namespace std;



/***	g e t S c o r e ( ) 
	Purpose: Function that gets called in the front end to get a score based on the user's movements.
	Inputs: Type of Therapy, side, and limb  (maybe use a data structure that contains all three?)
			(ex: getScore(Stroke, right, arm)
	Outputs: Score
	Requires: Interesting points.  Stored in prakhar's part as nodes.
*/

/* 	At the moment this function focuses on stroke patients,
	but has room for future additions (hence the case statment). */
int getScore(Therapy therapy, Side side, Type therapyType)
{
	int score = 99;
	
	//at the moment, focus on stroke patients.  Add new therapy types here in the future.
	switch (therapy)
	{
		//send along the limb and side to the getStrokeScore.
		case Stroke: 
			score = getStrokeScore(side, therapyType);
			break;
		default: score = 99;  //if score is 99, something went wrong in the call
			break;
	}
		
	if (score == 99)
		cout << "Something went wrong! Invalid score!";
	
	return score;
}


/* ---- o p e r a t o r > > ----
Purpose:	Overloads the >> for the Limb enum
Inputs:		istream - input stream
			limb - the enum to be read inputted
Outputs:	i - the input stream

*/
istream& operator >> (istream& i, Limb& limb)
{
	limb = Head;
	string value;
	if (i >> value) {
			if ( value == "Shoulder") 
				limb = Shoulder;
			else if (value == "Neck")
				limb = Neck;
			else if (value == "Elbow")
				limb = Elbow;
			else if (value == "Hand")
				limb = Hand;
			else if (value == "Hip")
				limb = Hip;
			else if (value == "Knee")
				limb = Knee;
			else if (value == "Arm")
				limb = Arm;
			else if (value == "Leg")
				limb = Leg;
			else if (value == "Ankle")
				limb = Ankle;
			else if (value == "Foot")
				limb == Foot;
			else if (value == "Wrist")
				limb == Wrist;
			else if (value == "Torso")
				limb == Torso;
			else if (value == "Waist")
				limb == Waist;
	}
	return i;
}

/* ---- o p e r a t o r > > ----
Purpose:	Overloads the >> for the Side enum
Inputs:		istream - input stream
			side - the enum to be read inputted
Outputs:	i - the input stream

*/
istream& operator >> (istream& i, Side& side)
{
	side = Right;
	string value;
	if (i >> value) {
			if ( value == "Left") 
				side = Left;
	}
	return i;
}

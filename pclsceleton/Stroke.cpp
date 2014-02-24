/*	Author: Andrew Hajj
	Date: 12/30/13
	Name: Stroke.cpp
	Purpose:	Module for Stroke patients.  Contains the functions used for stroke therapy.
				Maybe combine this with TherapyCenter?
*/

#include "Stroke.h"
#include <fstream>
#include "NodeData.h"
#include <vector>
#include <math.h>
#include <iostream>

#define Pi  3.14159

using namespace std;


/*	g e t S t r o k e S c o r e ( ) 
	Purpose: Figures out the score of a stroke patient.
	Inputs:  Side and limb of the exercise being done
			(ex: getStrokeScore(right, arm)
	Outputs: Score
	Requires: Interesting points.  Stored in prakhar's part as nodes.
*/
int getStrokeScore(Side side, Type therapyType)
{
	//Read the nodes in somewhere here//
	vector<NodeData> capturedNodes = parseFile();
	long elapsedTime = capturedNodes[capturedNodes.size() - 1].Time() - capturedNodes[0].Time();
	
	int score = 99;
	vector<int> Angles; //create the vector that will store the angles

	//Get an array of the angles based on the side and limb.
	switch (therapyType) {
	case MotorArm:
		if (side == Right) 
			Angles = fillAngles(Right, Arm, capturedNodes);
		else if (side == Left) 
		{
			Angles = fillAngles(Left, Arm, capturedNodes);
		}

		score = ruleBasedArm(Angles, elapsedTime);
		
	//	break;
/*	case MotorLeg:
		if (side == Right) 
			Angles = fillAngles(Right, Leg, capturedNodes);
		else if (side == Left) 
		{
			Angles = fillAngles(Left, Leg, capturedNodes);
		}

		score = ruleBasedLeg(Angles, elapsedTime); */
	} 

	return score;
}

/*	r u l e B a s e d A r m ( ) 
	Purpose: Figures out the score for arm exercises.  The nodes passed in will differ for right and left arms.
	Outputs: Score
	Requires: Interesting points.  Stored in prakhar's part as nodes.
*/
int ruleBasedArm(vector<int> angles, int elapsedTime)
{
	int range0 = 80;
	int range3 = 35;
	int range3Low = 23;
	int baseLineCount = 3;
	int angleBuffer = 5, dropSeconds = 7;
	
	int range0Count = 0, range2Count = 0, range3Count = 0, range4Count = 0;;
	
	int size = angles.size();
	
	//cout << angles.size();
	//the baseline is the average of the first 20 angles in the array
	//supposedly baseline is arms at rest on the sides of chair.
	int baseLine = 0; 
	
	for(int counter = 0; counter < baseLineCount; counter++)  //made it three for now as my test file had the first 3 nodes
		baseLine+=angles[counter];
	baseLine=baseLine/baseLineCount; //take the average (change back to 20 later)
	
	for(int index = baseLineCount; index < size; index++)
	{
		if (angles[index] <= baseLine + angleBuffer)
		{
			++range4Count;
			break;
		}
		else if(angles[index] >= range0) 
			++range0Count;
		else if (range3 <= angles[index] < range0) 
			++range2Count;
		else if (range3Low <= angles[index] < range3)
			++range3Count;
		else {
			++range4Count;
			break;
		}
//		beginTime = ;
	}
	cout << endl << "Last angle is: " << angles[size - 1] << endl;
	cout <<  "baseLine with buffer = " << (baseLine + angleBuffer) << endl;
	//Check if the arm was held at 90 degrees for the full time.
	if(range0Count == size-baseLineCount)
		return 0;
	//Check if the arm hit 90, but drifted and did not hit chair
	else if ((range0Count > 0) && (range4Count == 0) )
		return 1;
	//Check if arm fell and hit baseline with in the time range specified by dropSeconds
	else if (((range2Count + range0Count) < dropSeconds) && (range4Count > 0) && (range2Count + range0Count) != 0 ) 
		return 3;
	//Check if arm drifted and hit chair
	else if ((range0Count > 0 || range2Count > 0) && (range4Count > 0))
		return 2;
	//Check if no movement was done
	else //if( (range3Count == 0) && (range4Count > 0) )
		return 4;
	
}

/*	r u l e B a s e d L e g ( ) 
	Purpose: Figures out the score for the left arm exercise
	Outputs: Score
	Requires: Interesting points.  Stored in prakhar's part as nodes.
*/
//int ruleBasedLeg(int angles[], int times[])
//{
	//Similiar to the arms, but different angles.
//}

/*	f i l l A n g l e s ( ) 
	Purpose: 	Returns an array containing the angles between passed in nodes over time.
				Almost everything in this function will probably have to change -
				waiting on more info from the other segments.
	Inputs:  Side and limb the exercises being done.
			(ex: fillAngles(right, arm)
	Outputs: An array of the angles
	Requires: Interesting points.  Stored in prakhar's part as nodes.
*/
vector<int> fillAngles(Side side, Limb limb, vector<NodeData>& nodeList)
{
	vector<int> angleList;
	NodeData nodeOne;
	NodeData nodeTwo;
	NodeData nodeThree;

	long timeStamp = nodeList[0].Time();
			
	//fill the nodes array with the nodes we are looking for
	//this part also depends on how many nodes are being used.
	//in this case, I am guessing 3 - one for the shoulder, one for the hand
	//and one for the hip.
	for (int counter = 0; counter < nodeList.size(); counter++)
	{
		if ( nodeList[counter].Time() == timeStamp)
		{
			if (nodeList[counter].getLimb() == Elbow)
				nodeOne = nodeList[counter];
		
			else if ( nodeList[counter].getLimb() == Shoulder )
				nodeTwo = nodeList[counter];
		
			else if ( nodeList[counter].getLimb() == Hip )
				nodeThree = nodeList[counter];
			
		}
		else  //if the timeStamp is different, then it is likely we have reached the end of the nodes captured at that time
		{
			timeStamp = nodeList[counter].Time();
		//	cout << endl << "Time: " << nodeList[counter].Time() << endl;
		//	cout << nodeOne.Limb() << " " << nodeTwo.Limb() <<  " " << nodeThree.Limb() << endl;
			// calculate vector from nodeOne to nodeTwo
		//	cout << endl << nodeOne.Y() << " " << nodeTwo.Y() << " " << nodeThree.Y() << endl << endl;
			double x1 = nodeOne.X() - nodeTwo.X();
			double y1 = nodeOne.Y() - nodeTwo.Y();
			double z1 = nodeOne.Z() - nodeTwo.Z();
	
			// calculate vector from nodeTwo to nodeThree
			double x2 = nodeThree.X() - nodeTwo.X();
			double y2 = nodeThree.Y() - nodeTwo.Y();
			double z2 = nodeThree.Z() - nodeTwo.Z();
	//		cout << "x1, , y1, z1 = " << x1 << " " << y1 << " " << z1 << endl;
	//		cout << "x2, y2, z2 = " << x2 << " " << y2 << " " << z2 << endl;
			//calculate angle between the two vectors
			 double angleInRadians = acos((x1 * x2 + y1 * y2 + z1 * z2) 
					/ (sqrt((x1 * x1 + y1 * y1 + z1 * z1) 
					* (x2 * x2 + y2 * y2 + z2 * z2))));
	//		 cout << "Angle in Radians  = " << angleInRadians << endl;
			 cout  << "Current Angle [" << counter << "] = " << angleInRadians * 180 / Pi << endl;
			 angleList.push_back(angleInRadians * 180 / ( atan(1.0)*4) );  //add the angle to the list
			 counter--;
		}
	}
	//cout << "Pi = " << Pi << endl;
	return angleList;
}


/*	p a r s e F i l e ( ) 
	Purpose: 	Returns a vector containing all the nodes in a specified file.
	Outputs: A vector of nodes

*/
vector<NodeData> parseFile()
{
	vector<NodeData> inputVector;
	ifstream file("input_score2.txt");

	if (!file.good()) {
		cout << "bad file!";
		exit(0);
	}

	double X_coord, Y_coord, Z_coord;
	Side tempSide;
	Limb tempLimb;
	long tempTime;
	
	while (file >> X_coord >> Y_coord >> Z_coord >> tempSide >> tempLimb >> tempTime)
	{
		NodeData node(X_coord, Y_coord, Z_coord, tempSide, tempLimb, tempTime);
//		cout << endl << "x = " << X_coord << " y = " << Y_coord << " z = " << Z_coord << " side = " << tempSide << " limb = " << tempLimb << " time = " << tempTime << endl;
		inputVector.push_back(node);
	}
	
	//Close the file after use
	file.close();
	
	return inputVector;
}

/*	Author: Andrew Hajj
	Date: 12/30/13
	Name: Stroke.h
	Purpose:	Module for Stroke patients.  Contains the functions used for stroke therapy.
				Maybe combine this with TherapyCenter?
*/


#ifndef STROKE_H
#define STROKE_H

#include "TherapyCenter.h"
#include <fstream>
#include "NodeData.h"
#include <vector>



//Function that gets called by TherapyCenter.
int getStrokeScore(Side side, Type therapyType);


//Functions called by getStrokeScore.
int ruleBasedArm(vector<int> angles, int elapsedTime);		//returns the score for an arm exercise based on the rule-based algorithm

//ruleBasedLeg needs to be done still
//int ruleBasedLeg(vector<int> angles, int elapsedTime);		//returns the score for a leg exercise also based on the rule-based algorithm

vector<int> fillAngles(Side side, Limb limb, vector<NodeData>& nodeList);			//creates angles that the nodes create.

//Function to parse the text file into Nodes
vector<NodeData> parseFile();

#endif
/* 
 * File:   findpath.h
 * Author: byrdie
 *
 * Created on April 4, 2015, 11:19 PM
 */

#ifndef FINDPATH_H
#define	FINDPATH_H

#include "stlastar.h" // See header for copyright and usage information
#include "main.h"

#include <iostream>
#include <stdio.h>
#include <math.h>

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0

#define MAP_WIDTH 1000
#define MAP_HEIGHT 320
using namespace std;

// Global data

// The world map



// map helper functions

int GetMap(int x, int y);
cv::Point * search(cv::Mat birds_eye, cv::Point, cv::Point);


// Definitions

class MapSearchNode {
public:
    int x; // the (x,y) positions of the node
    int y;

    MapSearchNode() {
        x = y = 0;
    }

    MapSearchNode(int px, int py) {
        x = px;
        y = py;
    }

    float GoalDistanceEstimate(MapSearchNode &nodeGoal);
    bool IsGoal(MapSearchNode &nodeGoal);
    bool GetSuccessors(AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node);
    float GetCost(MapSearchNode &successor);
    bool IsSameState(MapSearchNode &rhs);

    void PrintNodeInfo();


};

#endif	/* FINDPATH_H */


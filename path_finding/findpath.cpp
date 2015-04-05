////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// STL A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "findpath.h"

const int MAP_WIDTH = 1750;
const int MAP_HEIGHT = 320;

int world_map[1750][320];

int GetMap(int x, int y) {
    if (x < 0 ||
            x >= MAP_WIDTH ||
            y < 0 ||
            y >= MAP_HEIGHT
            ) {
        return 9;
    }

    return world_map[x][y];
}

bool MapSearchNode::IsSameState(MapSearchNode &rhs) {

    // same state in a maze search is simply when (x,y) are the same
    if ((x == rhs.x) &&
            (y == rhs.y)) {
        return true;
    } else {
        return false;
    }

}

void MapSearchNode::PrintNodeInfo() {
    char str[100];
    sprintf(str, "Node position : (%d,%d)\n", x, y);

    cout << str;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal. 

float MapSearchNode::GoalDistanceEstimate(MapSearchNode &nodeGoal) {
    return fabsf(x - nodeGoal.x) + fabsf(y - nodeGoal.y);
}

bool MapSearchNode::IsGoal(MapSearchNode &nodeGoal) {

    if ((x == nodeGoal.x) &&
            (y == nodeGoal.y)) {
        return true;
    }

    return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application

bool MapSearchNode::GetSuccessors(AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node) {

    int parent_x = -1;
    int parent_y = -1;

    if (parent_node) {
        parent_x = parent_node->x;
        parent_y = parent_node->y;
    }


    MapSearchNode NewNode;

    // push each possible move except allowing the search to go backwards

        if ((GetMap(x - 1, y) < 1)
//    if ((GetMap(x - 1, y) > 0)

            && !((parent_x == x - 1) && (parent_y == y))
            ) {
        NewNode = MapSearchNode(x - 1, y);
        astarsearch->AddSuccessor(NewNode);
    }

        if ((GetMap(x, y - 1) < 1)
//    if ((GetMap(x, y - 1) > 0)

            && !((parent_x == x) && (parent_y == y - 1))
            ) {
        NewNode = MapSearchNode(x, y - 1);
        astarsearch->AddSuccessor(NewNode);
    }

        if ((GetMap(x + 1, y) < 1)
//    if ((GetMap(x + 1, y) > 0)

            && !((parent_x == x + 1) && (parent_y == y))
            ) {
        NewNode = MapSearchNode(x + 1, y);
        astarsearch->AddSuccessor(NewNode);
    }


        if ((GetMap(x, y + 1) < 1)
//    if ((GetMap(x, y + 1) > 0)

            && !((parent_x == x) && (parent_y == y + 1))
            ) {
        NewNode = MapSearchNode(x, y + 1);
        astarsearch->AddSuccessor(NewNode);
    }

    return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving

float MapSearchNode::GetCost(MapSearchNode &successor) {
    return (float) GetMap(x, y);

}


// Main

Point * search(cv::Mat birds_eye) {

    Point * trajectory = (Point *) malloc(sizeof (Point) * 3000);

    //    cout << "STL A* Search implementation\n(C)2001 Justin Heyes-Jones\n";

    // Our sample problem defines the world as a 2d array representing a terrain
    // Each element contains an integer from 0 to 5 which indicates the cost 
    // of travel across the terrain. Zero means the least possible difficulty 
    // in travelling (think ice rink if you can skate) whilst 5 represents the 
    // most difficult. 9 indicates that we cannot pass.

    // Create an instance of the search class...

    for (int i = 0; i < MAP_WIDTH; i++) {
        for (int j = 0; j < MAP_HEIGHT; j++) {
            world_map[i][j] = birds_eye.at<Vec3b>(j, i)[0];
        }
    }

    AStarSearch<MapSearchNode> astarsearch;

    unsigned int SearchCount = 0;

    const unsigned int NumSearches = 1;

    while (SearchCount < NumSearches) {

        // Create a start state
        MapSearchNode nodeStart;
        //        nodeStart.x = rand() % MAP_WIDTH;
        //        nodeStart.y = rand() % MAP_HEIGHT;
        nodeStart.x = 6;
        nodeStart.y = MAP_HEIGHT / 2;

        // Define the goal state
        MapSearchNode nodeEnd;
        //        nodeEnd.x = rand() % MAP_WIDTH;
        //        nodeEnd.y = rand() % MAP_HEIGHT;
        nodeEnd.x = MAP_WIDTH - 5;
        nodeEnd.y = MAP_HEIGHT / 2;

        // Set Start and goal states

        astarsearch.SetStartAndGoalStates(nodeStart, nodeEnd);

        unsigned int SearchState;
        unsigned int SearchSteps = 0;

        do {
            SearchState = astarsearch.SearchStep();

            SearchSteps++;


        } while (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);

        if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED) {
            cout << "Search found goal state\n";

            MapSearchNode *node = astarsearch.GetSolutionStart();

#if DISPLAY_SOLUTION
            cout << "Displaying solution\n";
#endif
            int steps = 0;

            //            node->PrintNodeInfo();
            for (;;) {

                trajectory[steps].x = node->x;
                trajectory[steps].y = node->y;

                node = astarsearch.GetSolutionNext();

                if (!node) {
                    break;
                }

                //                node->PrintNodeInfo();
                steps++;

            };

            cout << "Solution steps " << steps << endl;

            // Once you're done with the solution you can free the nodes up
            astarsearch.FreeSolutionNodes();


        } else if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED) {
            cout << "Search terminated. Did not find goal state\n";

        }

        // Display the number of loops the search went through
        cout << "SearchSteps : " << SearchSteps << "\n";

        SearchCount++;

        astarsearch.EnsureMemoryFreed();
    }

    return trajectory;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Created by jordi on 15/05/16.
//

#ifndef REPO_CPP_GRAPH_H
#define REPO_CPP_GRAPH_H

#include <iostream>
#include <stdio.h>
#include <time.h>
#include <list>
#include <queue>
#include <map>
#include <GL/glut.h>
#include "Node.h"

using namespace std;

class Graph
{
public:
    static Graph* Instance();
    ~Graph() {}
    void addNode(Node);
    void deleteNode(Node);
    Node& getNode(long);
    bool existNode(long);
    list<Node> getTourismNodes();
    list<Node> getUndeletedNodes();
    list<Node> getDeletedNodes();
    map<long, Node> getAllNodes();
    void toString();

    //Getters
    double getXMax();
    double getYMax();
    double getXMin();
    double getYMin();
    double getMaxDistance();
    double getMinDistance();

    double getMaxScore();
    //Setters
    void setXMax(double);
    void setYMax(double);
    void setXMin(double);
    void setYMin(double);
    void setMaxDistance(double);
    void setMinDistance(double);

    void calculateMaxMinDistance();
private:
    static Graph* _instance;
    Graph() {}
    map<long, Node> _nodes;
    double _x_max;
    double _y_max;
    double _x_min;
    double _y_min;
    double _maxDistance;
    double _minDistance;
};
#endif //REPO_CPP_GRAPH_H

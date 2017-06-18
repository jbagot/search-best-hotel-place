//
// Created by jordi on 15/05/16.
//

#include "Graph.h"

using namespace std;

Graph* Graph::_instance = 0;
double MAX_DISTANCE = 0, MIN_DISTANCE = 999;

Graph* Graph::Instance()
{
    if (_instance == 0)
    {
        _instance = new Graph();
        _instance->setMaxDistance(MAX_DISTANCE);
        _instance->setMinDistance(MIN_DISTANCE);
    }
    return _instance;
}

void Graph::addNode(Node node) {
    _nodes[node.getId()] = node;
}

void Graph::deleteNode(Node node) {
    map<long, double> neighbors = node.getNeighbors();

    for (map<long, double>::iterator itNeigh = neighbors.begin(); itNeigh != neighbors.end(); ++itNeigh) {
        Node& neigh = this->getNode(itNeigh->first);
        neigh.deleteNeighbor(node.getId());
    }

    _nodes.erase(node.getId());
}

Node& Graph::getNode(long id) {
    map<long, Node>::iterator it = _nodes.find(id);
    if (it != _nodes.end())
        return it->second;
}

bool Graph::existNode(long id) {
    map<long, Node>::iterator it = _nodes.find(id);
    return it != _nodes.end();
}

list<Node> Graph::getTourismNodes() {
    list<Node> tourismNodes;
    for (map<long, Node>::iterator it=_nodes.begin(); it!=_nodes.end(); ++it) {
        if(it->second.getName()!="") {
            tourismNodes.push_back(it->second);
        }
    }
    return tourismNodes;
}

list<Node> Graph::getUndeletedNodes() {
    list<Node> undeletedNodes;
    for (map<long, Node>::iterator it=_nodes.begin(); it!=_nodes.end(); ++it) {
        if(it->second.getName()!="" || it->second.getNeighbors().size()!=2) {
            undeletedNodes.push_back(it->second);
        }
    }

    return undeletedNodes;
}

list<Node> Graph::getDeletedNodes() {
    list<Node> deletedNodes;
    for (map<long, Node>::iterator it=_nodes.begin(); it!=_nodes.end(); ++it) {
        if(it->second.getName()=="" && it->second.getNeighbors().size()==2) {
            deletedNodes.push_back(it->second);
        }
    }

    return deletedNodes;
}

map<long, Node> Graph::getAllNodes() {
    return _nodes;
}

void Graph::toString() {
    for (map<long, Node>::iterator it=_nodes.begin(); it!=_nodes.end(); ++it) {
        it->second.toString();
    }
}


//Getters
double Graph::getXMax() {
    return _x_max;
}

double Graph::getYMax() {
    return _y_max;
}

double Graph::getXMin() {
    return _x_min;
}

double Graph::getYMin() {
    return _y_min;
}

double Graph::getMaxDistance() {
    return _maxDistance;
}

double Graph::getMinDistance() {
    return _minDistance;
}

double Graph::getMaxScore() {
    double maxScore = 0, auxScore;
    for (map<long, Node>::iterator it=_nodes.begin(); it!=_nodes.end(); ++it) {
        auxScore = it->second.getPoint().getScore();
        if(auxScore > maxScore)
            maxScore = auxScore;
    }
    return maxScore;
}

void Graph::calculateMaxMinDistance() {
    double maxDistance = this->getMaxDistance(), minDistance = this->getMinDistance();
    if(maxDistance == MAX_DISTANCE && minDistance == MIN_DISTANCE) {
        map<long, double> neighbors;
        for (map<long, Node>::iterator it = _nodes.begin(); it != _nodes.end(); ++it) {
            neighbors = it->second.getNeighbors();
            for (map<long, double>::iterator itNeigh = neighbors.begin(); itNeigh != neighbors.end(); ++itNeigh) {
                if (itNeigh->second > maxDistance) {
                    maxDistance = itNeigh->second;
                }
                if (itNeigh->second < minDistance) {
                    minDistance = itNeigh->second;
                }
            }
        }
        this->setMaxDistance(maxDistance);
        this->setMinDistance(minDistance);
    }
}

//Setters
void Graph::setXMax(double x_max) {
    _x_max = x_max;
}
void Graph::setYMax(double y_max) {
    _y_max = y_max;
}
void Graph::setXMin(double x_min) {
    _x_min = x_min;
}
void Graph::setYMin(double y_min) {
    _y_min = y_min;
}
void Graph::setMaxDistance(double maxDistance) {
    _maxDistance = maxDistance;
}
void Graph::setMinDistance(double minDistance) {
    _minDistance = minDistance;
}
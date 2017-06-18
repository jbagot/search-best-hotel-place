//
// Created by jordi on 16/05/16.
//

#include "Node.h"

/**
 * CONSTRUCTORS
 */
Node::Node(long id, long prev, bool intersection, double lat, double lon, double x, double y) {
    _id = id;
    _prev = prev;
    _intersection = intersection;
    _lat = lat;
    _lon = lon;
    _point = Point(x, y, 0);
    _weight = 0;
}

Node::Node(long id, long prev, bool intersection, double lat, double lon,
                         double x, double y, string name, string type, double weight) {
    _id = id;
    _prev = prev;
    _intersection = intersection;
    _lat = lat;
    _lon = lon;
    _point = Point(x, y, 0);
    _name = name;
    _type = type;
    _weight = weight;
}

/**
 * OPERATORS
 */
bool Node::operator==(Node const &other) {
    return other._id == _id;
}

bool Node::operator<(const Node& nNew) const {
    return (nNew._auxDistance > _auxDistance);
}

bool Node::operator()(const Node& nOld, const Node& nNew) const {
    return (nNew._auxDistance < nOld._auxDistance);
}

/**
 * GETTERS
 */
long Node::getId() const{
    return _id;
}

long Node::getPrev() {
    return _prev;
}

bool Node::getIntersection() {
    return _intersection;
}

double Node::getLat() {
    return _lat;
}

double Node::getLon() {
    return _lon;
}

double Node::getAuxDistance() {
    return _auxDistance;
}

map<long, double> Node::getNeighbors() {
    return _neighbors;
}

string Node::getType() {
    return _type;
}

double Node::getWeight() {
    return _weight;
}

string Node::getName() {
    return _name;
}

Point Node::getPoint() {
    return _point;
}

map<long, pair<double, long>> Node::getTourismScope() {
    return _tourismScope;
}

/**
 * SETTERS
 */
void Node::setId(long id) {
    _id = id;
}

void Node::setPrev(long prev) {
    _prev = prev;
}

void Node::setIntersection(bool intersection) {
    _intersection = intersection;
}

void Node::setLat(double lat) {
    _lat = lat;
}

void Node::setLon(double lon) {
    _lon = lon;
}

void Node::setAuxDistance(double auxDistance) {
    _auxDistance = auxDistance;
}

void Node::setType(string type) {
    _type = type;
}

void Node::setWeight(double weight) {
    _weight = weight;
}

void Node::setName(string name) {
    _name = name;
}

void Node::setPoint(double x, double y, double score) {
    _point.setX(x);
    _point.setY(y);
    _point.setScore(score);
}

void Node::addTourismScope(long tourismNode, double distance, long prev) {
    _tourismScope[tourismNode] = pair<double, long> (distance, prev);
}

void Node::addNeighbor(long neighbor, double distance) {
    _neighbors[neighbor] = distance;
}

void Node::toString() {
    cout << "Node " << _id << ": ";
    cout << "    name: " << _name;
    cout << ",    intersection: " << _intersection;
    cout << ",    lat: " << _lat;
    cout << ",    lon: " << _lon;
    cout << ",    x: " << this->getX();
    cout << ",    y: " << this->getY();
    cout << ",    type: " << _type;
    cout << ",    weight: " << _weight;
    cout << ",    score: " << this->getScore() << endl;
    cout << "Neighbors: {";

    for (map<long, double>::iterator it=_neighbors.begin(); it!=_neighbors.end(); ++it) {
        cout << endl;
        cout << "    id: " << it->first;
        cout << "    distance: " << it->second;
    }
    cout << endl << "}" << endl;
    cout << "TourismScope: {";

    for (map<long, pair<double, long>>::iterator it=_tourismScope.begin(); it!=_tourismScope.end(); ++it) {
        cout << endl;
        cout << "    id: " << it->first;
        cout << "    distance: " << it->second.first;
        cout << "    previous: " << it->second.second;
    }
    cout << endl << "}" << endl;
}

bool Node::isTourismNode() {
    return _type!="";
}

void Node::deleteNeighbor(long nodeId) {
    _neighbors.erase(nodeId);
}
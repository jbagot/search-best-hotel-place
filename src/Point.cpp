//
// Created by jordi on 09/07/16.
//

#include "Point.h"

//Constructors
Point::Point(double x, double y) {
    _x = x;
    _y = y;
    _score = 0;
}

Point::Point(double x, double y, double score) {
    _x = x;
    _y = y;
    _score = score;
}

//Getters
double Point::getX() {
    return _x;
}

double Point::getY() {
    return _y;
}

double Point::getScore() {
    return _score;
}

//Setters
void Point::setX(double x) {
    _x = x;
}

void Point::setY(double y) {
    _y = y;
}

void Point::setScore(double score) {
    _score = score;
}

void Point::toString() {
    cout << "POINT = [" << _x << ", " << _y << "] = " << _score << endl;
}
//
// Created by jordi on 09/07/16.
//

#ifndef REPO_CPP_POINT_H
#define REPO_CPP_POINT_H

#include <iostream>

using namespace std;

class Point {
    public:
        Point() {};
        Point(double, double);
        Point(double, double, double);

        //Getters
        double getX();
        double getY();
        double getScore();

        //Setters
        void setX(double);
        void setY(double);
        void setScore(double);

        void toString();
    private:
        double _x;
        double _y;
        double _score;
};

#endif //REPO_CPP_POINT_H
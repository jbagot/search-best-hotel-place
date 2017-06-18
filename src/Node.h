//
// Created by jordi on 16/05/16.
//

#ifndef REPO_CPP_NODE_H
#define REPO_CPP_NODE_H

#include <iostream>
#include <map>
#include "Point.h"

using namespace std;

class Node : Point {
    public:
        //Constructors

        Node() {};
        Node(long, long, bool, double, double, double, double);
        Node(long, long, bool, double, double, double, double, string, string, double);
        //Operators
        bool operator==(Node const&);
        bool operator<(const Node&)const;
        bool operator()(const Node&, const Node&) const;

        //Getters
        long getId() const ;
        long getPrev();
        bool getIntersection();
        double getLat();
        double getLon();
        Point getPoint();
        double getAuxDistance();
        string getType();
        double getWeight();
        string getName();
        map<long, double> getNeighbors();
        map<long, pair<double, long>> getTourismScope();

        //Setters
        void setId(long);
        void setPrev(long);
        void setIntersection(bool);
        void setLat(double);
        void setLon(double);
        void setAuxDistance(double);
        void setType(string);
        void setWeight(double);
        void setName(string);
        void setPoint(double, double, double);

        void addTourismScope(long, double, long);
        void addNeighbor(long, double);
        void toString();
        bool isTourismNode();
        void deleteNeighbor(long);
    private:

        long _id;
        long _prev;
        bool _intersection;
        double _lat;
        double _lon;
        double _auxDistance;
        string _type;
        double _weight;
        string _name;
        Point _point;

        map<long, pair<double, long>> _tourismScope;
        map<long, double> _neighbors;
};


#endif //REPO_CPP_NODE_H

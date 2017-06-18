//
// Created by jordi on 26/04/16.
//

#include <iostream>
#include <sstream>
#include <GL/glew.h> // Include the GLEW header file
#include <GL/glut.h> // Include the GLUT header file
#include <getopt.h>
#include <cstring>
#include "Graph.h"
#include "pugiXML/pugixml.hpp"
#include <math.h>
#include <Framework.h>
#include <set>

using namespace std;

Graph* graphMap;
int WIDTH = 1000, HEIGHT = 1000, MIN_COLOR = 50, MAX_COLOR = 255;
double MIN_WEIGHT = 1, MAX_WEIGHT = 10, minScore = 999, maxScore = 0, SIMPLIFY_TOLERANCE = 0.0001;
//scoreMaxPoints es on guardarem els scores de tots els punts on hi hagi un moviment de scope.
//Es un map on es guarden tots els scores i coordenades de cada aresta en una llista. Cada valor del map es una aresta,
//La clau del map es el id del node i el neighbor, per distingir les arestes
map<pair<long, long>, std::list<Point>> scoreMaxPoints;
//Is the point where the score is higgest
Point maxScorePoint;

int mainWindow = 0;

bool lbuttonDown=false;
bool rbuttonDown=false;
Vector2 lastPos;
Camera camera;

#define LEFT_PLANE                  0
#define RIGTH_PLANE                 1
#define TOP_PLANE                   1
#define BOTTOM_PLANE                0

#define NEAR_PLANE                  0
#define FAR_PLANE                   1

#define EYE                         Vector3(0.0f, 0.0f, 0.0f)
#define CENTER                      Vector3(0.0f, 0.0f, -1.0f)
#define UP                          Vector3(0.0f, 1.0f, 0.0f)


//he main loop (we dont need a loop inside, GLUT calls this function constanly)
void repaint()
{
    glutSetWindow(mainWindow);
    glutPostRedisplay();
}

//------------------------------------------------------------------------------------
//--------------------------- KEYBOARD FUNCTIONS -------------------------------------
//------------------------------------------------------------------------------------
void onKeyEvent(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 27: exit(0);
        default:
            break;
    }
    repaint();
}

void processSpecialKeys(int key, int x, int y)
{
    switch (key)
    {
        case GLUT_KEY_F4: break;
        default: break;
    }

    repaint();
}

//------------------------------------------------------------------------------------
//----------------------------- MOUSE FUNCTIONS --------------------------------------
//------------------------------------------------------------------------------------

void mouseActiveMotionEvent(int x, int y)
{
    if(lbuttonDown)
    {

        float speed = (float) 0.003;
        float proportion=camera.right-camera.left;
        speed*=proportion;
        float delta_x = lastPos.x-x;
        float delta_y = lastPos.y-y;
        lastPos.set((float)x,(float)y);

        camera.move(Vector3((-delta_x)*(camera.right*speed), (delta_y)*(camera.right*speed), 0));
        repaint();
    }

    if(rbuttonDown) {

    }

}


void mouseWheelEvent(int wheel, int direction, int x, int y)
{
    int numDegrees = 100*direction;
    float proportion=camera.right-camera.left;
    float numSteps = (float) ((numDegrees / 15.0)*0.008)*proportion;

    if(camera.right-numSteps>camera.left+numSteps)
    {
        camera.left      += numSteps;
        camera.right    -= numSteps;
        camera.top         -= numSteps;
        camera.bottom     += numSteps;
        camera.updateProjectionMatrix();
    }

    repaint();
}

void mousePressEvent(int button, int state, int x, int y)
{
    lbuttonDown=false;
    rbuttonDown=false;

    if(state == GLUT_DOWN)
    {
        if(button == GLUT_LEFT_BUTTON) lbuttonDown = true;
        else if(button == GLUT_RIGHT_BUTTON) rbuttonDown = true;

        lastPos.set((float)x,(float)y);
    }
    /*
    if(rbuttonDown){
        Vector2 sp=c.getSpaceVector((x/(float)SCREEN_RES_X),(SCREEN_RES_Y-y)/(float)SCREEN_RES_Y);
        mc.centerPaintedCircle(sp.x,sp.y);
        //float width=c.right-c.left, height = c.top-c.bottom;
        ////float v_width=2*(c.up.x-c.center.x), v_height=2*(c.up.y-c.center.y);
        ////float xV=c.up.x+v_width*x/float(SCREEN_RES_X), yV=c.up.y+v_height*y/float(SCREEN_RES_Y);
        ////mc.centerPaintedCircle(c.top,c.bottom);
        //mc.centerPaintedCircle(c.center.x+c.left+*width,c.center.y+c.bottom + height*;
    }
    */
    repaint();
}

//http://stackoverflow.com/a/236803/3540693
vector<string> &split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

//http://stackoverflow.com/a/236803/3540693
vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}

double cartesian_distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double getX(double lon) {
    return (WIDTH*(180+lon)/360);
}

double getY(double lat) {
    //height and width are map height and width
    double lat_rad = lat*M_PI/180;

    //get y value
    double merc_n = log(tan((M_PI/4)+(lat_rad/2)));
    return (WIDTH*merc_n/(2*M_PI));
}

double get_node_distance(pugi::xml_document &doc, long relation_node_id, double nodeX, double nodeY) {
    string xpath_query_relation_node = string("//node[@id=\"") + to_string(relation_node_id) + string("\"]");
    pugi::xpath_node relation_node = doc.select_node(xpath_query_relation_node.c_str());

    double x = 0, y = 0;
    if(relation_node) {
        //Get latitude of node
        double lat = relation_node.node().attribute("lat").as_double();
        if(lat) {
            //Get Y of node
            y = getY(lat);
        }
        //Get longitude on node
        double lon = relation_node.node().attribute("lon").as_double();
        if(lon) {
            //Get X of node
            x = getX(lon);
        }
    }

    return  cartesian_distance(nodeX, nodeY, x, y);
}

double getNormalizedWeight(double weight) {
    double result = 0;
    double maxDistance = graphMap->getMaxDistance();
    double minDistance = graphMap->getMinDistance();

    if(weight > MAX_WEIGHT) weight = MAX_WEIGHT;
    if(weight < MIN_WEIGHT) weight = MIN_WEIGHT;

    maxDistance *= 2;
    result = ((weight-MIN_WEIGHT)/(MAX_WEIGHT-MIN_WEIGHT))*(maxDistance-minDistance)+minDistance;

    return result;
}

void processFile(string cleanfile) {
    pugi::xml_document doc;
    if (!doc.load_file(cleanfile.c_str()))
        cout << "Error al carregar el cleanfile.osm en el pugixml" << endl;

    pugi::xpath_node_set ways = doc.select_nodes("//way");

    long prev_node, next_node;
    int i;
    string xpath_query, xpath_query_intersection, name, tourism;
    double lat, lon, x, y, distance, weight, x_min = 9999, x_max = -9999, y_min = 9999, y_max = -9999;
    bool intersection;
    for (pugi::xpath_node_set::const_iterator ways_it = ways.begin(); ways_it != ways.end(); ++ways_it) {
        pugi::xpath_node way = *ways_it;
        pugi::xpath_node_set nodesId = way.node().select_nodes("nd");
        //Get all nodes ids of current way
        vector<long> nodesIds;
        for (pugi::xpath_node_set::const_iterator nodesId_it = nodesId.begin(); nodesId_it != nodesId.end(); ++nodesId_it) {
            pugi::xpath_node nd = *nodesId_it;
            nodesIds.push_back(stol(nd.node().attribute("ref").value()));
        }

        for(i=0; i<nodesIds.size(); i++) {
            // Get previous node
            if (i!=0)
                prev_node = nodesIds[i-1];
            else
                prev_node = NULL;
            //Get next node
            if(i!=nodesIds.size()-1)
                next_node = nodesIds[i+1];
            else
                next_node = NULL;

            //Not exist node in graph yet
            if(!graphMap->existNode(nodesIds[i])) {
                Node newNode;
                //Get node element with ref == i
                xpath_query = string("//node[@id=\"") + to_string(nodesIds[i]) + string("\"]");
                pugi::xpath_node current_node = doc.select_node(xpath_query.c_str());
                //Get latitude of node
                lat = current_node.node().attribute("lat").as_double();
                //Get longitude on node
                lon = current_node.node().attribute("lon").as_double();
                //Get Y of node
                y = getY(lat);
                //Get X of node
                x = getX(lon);
                //Check if node is intersection or not
                xpath_query_intersection = string("//way/nd[@ref=\"") + to_string(nodesIds[i]) + string("\"]");
                pugi::xpath_node_set waysWithNode = doc.select_nodes(xpath_query_intersection.c_str());
                intersection = waysWithNode.size()>1;

                tourism = current_node.node().select_node("tag[@k=\"tourism\"]").node().attribute("v").as_string();
                if (tourism == "hotel" || tourism == "attraction") {
                    name = current_node.node().select_node("tag[@k=\"name\"]").node().attribute("v").as_string();
                    weight = current_node.node().select_node("tag[@k=\"weight\"]").node().attribute("v").as_double();
                    newNode = Node(nodesIds[i], NULL, intersection, lat, lon, x, y, name, tourism, weight);
                }
                else {
                    newNode = Node(nodesIds[i], NULL, intersection, lat, lon, x, y);
                }

                //Add node into graph
                graphMap->addNode(newNode);

                if(x > x_max)
                    x_max = x;
                if(y > y_max)
                    y_max = y;
                if(x < x_min)
                    x_min = x;
                if(y < y_min)
                    y_min = y;
            }
            Node& curNode = graphMap->getNode(nodesIds[i]);
            Point curPoint = curNode.getPoint();

            if(prev_node) {
                distance = get_node_distance(doc, prev_node, curPoint.getX(), curPoint.getY());
                curNode.addNeighbor(prev_node, distance);
            }

            if(next_node) {
                distance = get_node_distance(doc, next_node, curPoint.getX(), curPoint.getY());
                curNode.addNeighbor(next_node, distance);
            }
        }
    }
    graphMap->setXMax(x_max);
    graphMap->setYMax(y_max);
    graphMap->setXMin(x_min);
    graphMap->setYMin(y_min);

    //graphMap->toString();
}

double colorNormalize(double color255) {
    return color255/255;
}

double scoreToColor(double score) {
    double maxScoreAux = maxScore;
    double minScoreAux = minScore;

    if(score > 0) //Green
        minScoreAux = (minScore < 0) ? 0 : minScore;
    else //Red
        maxScoreAux = (maxScore > 0) ? 0 : maxScore;

    return colorNormalize(((score - minScoreAux) / (maxScoreAux - minScoreAux)) * (MAX_COLOR - MIN_COLOR) + MIN_COLOR);
}

void printPointByScore(double x, double y, double score) {
    if(score == 0) //White
        glColor3d(255.0, 255.0, 255.0);
    else {
        double color = scoreToColor(score);
        if(score > 0) //Green
            glColor3d(0.0, color, 0.0);
        else if(score < 0) //Red
            glColor3d(color, 0.0, 0.0);
    }

    glVertex2d(x, y);
}

void printPointByType(double x, double y, string type) {
    if(type == "hotel") //Blue
        glColor3d(0.0, 0.0, 255.0);
    else if(type == "attraction") //Yellow
        glColor3d(255.0, 255.0, 0.0);
    else
        glColor3d(255.0, 255.0, 255.0);

    glVertex2d(x, y);
}


Point getTransformedCoords(Point point, double graphXMax, double graphXMin, double graphYMax, double graphYMin) {
    double newX = (point.getX() - graphXMin) / (graphXMax - graphXMin);
    double newY = (point.getY() - graphYMin) / (graphYMax - graphYMin);
    return Point(newX, newY);
}

void display(void) {
    camera.set();
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f); //Black background
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    double graphXMax = graphMap->getXMax();
    double graphXMin = graphMap->getXMin();
    double graphYMax = graphMap->getYMax();
    double graphYMin = graphMap->getYMin();
    Point newPoint, nodePoint, neighPoint;
    int i=0;
    map<long, double> neighbors;
    map<long, Node> nodes = graphMap->getAllNodes(), printedNodes;
    //Draw all map in white
    glBegin(GL_LINES);
    for(map<long, Node>::iterator it = nodes.begin(); it != nodes.end(); ++it) {
        nodePoint = getTransformedCoords(it->second.getPoint(), graphXMax, graphXMin, graphYMax, graphYMin);
        neighbors = it->second.getNeighbors();
        for(map<long, double>::iterator itNeigh = neighbors.begin(); itNeigh != neighbors.end(); ++itNeigh) {
            if (printedNodes.find(itNeigh->first) == printedNodes.end()) {
                neighPoint = getTransformedCoords(graphMap->getNode(itNeigh->first).getPoint(), graphXMax, graphXMin, graphYMax, graphYMin);
                printPointByScore(nodePoint.getX(), nodePoint.getY(), 0);
                printPointByScore(neighPoint.getX(), neighPoint.getY(), 0);
            }
        }
        printedNodes[it->first] = it->second;
    }
    glEnd();

    //Draw color lines with scores
    for(map<pair<long, long>, std::list<Point>>::iterator it2 = scoreMaxPoints.begin(); it2 != scoreMaxPoints.end(); ++it2) {
        glBegin(GL_LINE_STRIP);
        cout << "[" << it2->first.first << " , " << it2->first.second << "] :" << endl;
        //Print node, start strip line with node
        i=0;
        for(list<Point>::iterator p = it2->second.begin(); p != it2->second.end(); ++p) {
            cout << "(" << p->getX() << ", " << p->getY() << ") = " << p->getScore() << endl;
            newPoint = getTransformedCoords(*p, graphXMax, graphXMin, graphYMax, graphYMin);
            printPointByScore(newPoint.getX(), newPoint.getY(), p->getScore());
            i++;
        }
        glEnd();
    }

    //Draw only node points
    glPointSize(5);
    glBegin(GL_POINTS);
    for (map<long, Node>::iterator it=nodes.begin(); it!=nodes.end(); ++it) {
        //Print node, start strip line with node
        newPoint = getTransformedCoords(it->second.getPoint(), graphXMax, graphXMin, graphYMax, graphYMin);
        printPointByType(newPoint.getX(), newPoint.getY(), it->second.getType());
    }
    glEnd();

    //Draw the best point of map
    glPointSize(10);
    glBegin(GL_POINTS);
    glColor3d(0.0, 255.0, 0.0);
    maxScorePoint.toString();
    newPoint = getTransformedCoords(maxScorePoint, graphXMax, graphXMin, graphYMax, graphYMin);
    glVertex2d(newPoint.getX(), newPoint.getY());
    glEnd();

    glutSwapBuffers();

    cout << "Distancia min:" << graphMap->getMinDistance() << ", Distancia max:" << graphMap->getMaxDistance() << endl;
}

void dijkstra(Node node, Node initialNode, double currentDistance,
              priority_queue<Node, vector<Node>, Node> pendingNodes, map<long, Node> processedNodes) {
    map<long, double> neighbors = node.getNeighbors();
    for(map<long, double>::iterator it=neighbors.begin(); it != neighbors.end(); ++it) {
        Node& neighbor = graphMap->getNode(it->first);
        double neighborDistance = it->second;

        map<long, Node>::const_iterator itProcessedNodes = processedNodes.find(neighbor.getId());
        //Check if node exist into map because we do not want processed nodes
        //FIXME: si esta a processedNodes haig de comprovar que tingui el tourismNode en el que estem ara, i si el te, comprovar que el valor sigui mes gran o no. Mhaig de quedar amb el que te el valor mes petit,
        // FIXME perque significa que esta mes a prop del tourismNode. Aqui perdo el node -20 amb el scope del -18
        if (itProcessedNodes == processedNodes.end()) {
            double acum_distance = currentDistance + neighborDistance;
            //Comprovem que en el neighbor actual encara hi arribi l abast del tourismNode, si l abast hi arriba el posarem a pendingNodes
            if (acum_distance <= initialNode.getWeight()) {
                neighbor.setAuxDistance(acum_distance);
                neighbor.setPrev(node.getId());
                pendingNodes.push(neighbor);
            }
        }
    }

    if(!pendingNodes.empty()) {
        Node auxNode = pendingNodes.top();

        //Delete first element from list
        pendingNodes.pop();

        if(!pendingNodes.empty()) {
            //FIXME aqui el mateix que a dalt, no pot ser que perdi nodes aqui. El node 17 el perdo aqui amb el tourism -18
            while (processedNodes.find(auxNode.getId()) != processedNodes.end()) {
                auxNode = pendingNodes.top();
                //Delete first element from list
                pendingNodes.pop();
            }
        }
        //Get with getNode method because we lost reference in pendingNodes vector
        Node& nextNode = graphMap->getNode(auxNode.getId());
        processedNodes[nextNode.getId()] = nextNode;
        nextNode.addTourismScope(initialNode.getId(), nextNode.getAuxDistance(), nextNode.getPrev());
        dijkstra(nextNode, initialNode, nextNode.getAuxDistance(), pendingNodes, processedNodes);
    }
}

void calculateScopeTourismNodes() {
    list<Node> tourismNodes = graphMap->getTourismNodes();
    for (list<Node>::iterator it=tourismNodes.begin(); it != tourismNodes.end(); ++it) {
        long tourismNodeId = it->getId();
        Node& tourismNode = graphMap->getNode(tourismNodeId);
        tourismNode.addTourismScope(tourismNodeId, 0, tourismNodeId);
        //Normalize weight
        tourismNode.setWeight(getNormalizedWeight(tourismNode.getWeight()));

        priority_queue<Node, vector<Node>, Node> pending_nodes;
        map<long, Node> processed_nodes;
        processed_nodes[tourismNode.getId()] = tourismNode;
        dijkstra(tourismNode, tourismNode, 0, pending_nodes, processed_nodes);
        //Afegim a ell mateix en el scope perque ell tambe es un tourism node
    }
    graphMap->toString();
}

struct TourismScopeDistance {
    long idTourismNode;
    long idPreviousNode;
    double weightRemain;
    bool isRepeated;
    //Repeated tourismNode in neigh
    long neighIdPreviousNode;
    double neighWeightRemain;

};

double calculateFirstScore(multimap<double, TourismScopeDistance> tourismScope, long neighborId,
                           map<long, int> &nodesIntOne, int &sumOnes, multiset<long> &repeatedTourismNodes) {
    double score  = 0, value;
    long tourismId, prevId;
    //Positive true, negative false
    bool isHotel, isPrevSameNeigh, isNegative;

    for(multimap<double, TourismScopeDistance>::iterator it = tourismScope.begin(); it != tourismScope.end(); ++it) {
        tourismId = it->second.idTourismNode;
        prevId = it->second.idPreviousNode;
        value = it->second.weightRemain;
        //If is hotel is negative (false), else true
        isHotel = (graphMap->getNode(tourismId).getType() == "hotel");
        isPrevSameNeigh = (prevId == neighborId);
        isNegative = (value < 0);

        if (isNegative) {
            /*
             * -H           => -1
             * -A           => +1
             */
            nodesIntOne[tourismId] = (isHotel) ? -1 : 1;
        }
        else {
            /* XOR
             * H && prev=neighbor  => -1
             * H && prev!=neighbor => +1
             * A && prev=neighbor  => +1
             * A && prev!=neighbor => -1
             * */
            nodesIntOne[tourismId] = (isHotel == isPrevSameNeigh) ? -1 : 1;
            sumOnes += nodesIntOne[tourismId];
            score += (value * ((isHotel) ? -1 : 1));
        }
    }
    return score;
}

double calculateEasyScore(double distance, double currentScore, long tourismNodeId, bool isNegative,
                          map<long, int> nodesIntOne, int &sumOnes, TourismScopeDistance tourismScopeDistance) {
    double score = 0;


    //Els nodes repetits
    score = currentScore + (sumOnes*distance);

    //Si es negatiu hem d afegir el node a la suma, sino l hem de treure
    if(isNegative)
        sumOnes += nodesIntOne[tourismNodeId];
    else {
        sumOnes += (nodesIntOne[tourismNodeId] * -1);
        //El repetit sempre sera positiu i li canviem el signe perque ara anira amb l altre
        if(tourismScopeDistance.isRepeated)
            sumOnes += (nodesIntOne[tourismNodeId] * -1);
    }

    return score;
}

double calculateScore(map<long, pair<double, long>> tourismScope) {
    double score = 0;

    for(map<long, pair<double, long>>::iterator it = tourismScope.begin(); it != tourismScope.end(); ++it) {
        Node tourismNode = graphMap->getNode(it->first);
        double normalizedWeight = tourismNode.getWeight();
        double remainWeight = normalizedWeight-it->second.first;
        if(tourismNode.getType() == "hotel")
            score -= remainWeight;
        else
            score += remainWeight;
    }

    return score;
}

void calculateMaxScorePoints() {
    //Get all nodes
    map<long, Node> nodes = graphMap->getAllNodes(), processedNodes;
    map<long, double> neighbors;
    map<long, pair<double, long>> nodeTourismScope, neighborTourismScope;
    //Need multimap because is posible that keys are repeated and need a pair because need weigh with absolute
    // values and normal.
    //distance absolute, distance, idNodeTourism, idNodePrevious
    multimap<double, TourismScopeDistance> noRepeatedTourismScope;
    multiset<long> repeatedTourismNodes;
    map<long, int> nodesIntOne;
    int sumOnes;
    pair<long, long> pairNodesIds;
    Point nodePoint, neighPoint, curPoint;

    double subsX = 0, subsY = 0, distance = 0, z = 0, pointX = 0, pointY = 0, scoreAtPoint = 0,
            weightRemainInNode = 0, weight = 0, weightRemainInNeigh = 0, partialDistance = 0;
    //Recorrem tots els nodes
    for (map<long, Node>::iterator it = nodes.begin(); it != nodes.end(); ++it) {
        Node &node = graphMap->getNode(it->first);
        nodePoint = node.getPoint();
        //Recuperem tots els tourismNodes que estiguin al seu abast
        nodeTourismScope = node.getTourismScope();
        //Recuperem tots els veins del node actual
        neighbors = node.getNeighbors();
        //Recorrem tots els veins del node actual
        for (map<long, double>::iterator itNeigh = neighbors.begin(); itNeigh != neighbors.end(); ++itNeigh) {
            //Comprovem si el vei ja ha sigut un node principal, si es aixi no el processem perque ja esta processat
            //if (processedNodes.find(itNeigh->first) == processedNodes.end()) {
                //Hem de netejar els maps que utilitzem per calcular el score
                //Aixo va per aresta, aixi que si es canvia de vei, sha de canviar
                nodesIntOne.clear();
                sumOnes = 0;
                noRepeatedTourismScope.clear();
                Node &neighbor = graphMap->getNode(itNeigh->first);
                neighPoint = neighbor.getPoint();
                //Es la distancia entre el node i el vei
                distance = itNeigh->second;
                //Es un pair amb els ids del node i el vei
                pairNodesIds = pair<long, long>(node.getId(), neighbor.getId());
                //Recuperem els tourismNodes que tenen abast a el vei
                neighborTourismScope = neighbor.getTourismScope();
                //1. Filtrar els nodes que estan en el node i en el neighbor, aquests no els volem recorrer perque no
                // acaben en aquesta aresta, pero si que els voldrem sumar al score
                //2. Ordenar els nodes que hem filtrat al pas1 per distancia minima.
                for(map<long, pair<double, long>>::iterator itNodeTourism = nodeTourismScope.begin(); itNodeTourism != nodeTourismScope.end(); ++itNodeTourism) {
                    //Put nodes that only appears in nodeTourismScope in another map because we only want use these, but
                    // change distance to distance remain of scope
                    weight = graphMap->getNode(itNodeTourism->first).getWeight();
                    weightRemainInNode = (weight - itNodeTourism->second.first);
                    partialDistance = weightRemainInNode;

                    TourismScopeDistance tourismScopeDistance;
                    tourismScopeDistance.idTourismNode=itNodeTourism->first;
                    tourismScopeDistance.idPreviousNode=itNodeTourism->second.second;
                    tourismScopeDistance.weightRemain=weightRemainInNode;
                    tourismScopeDistance.isRepeated=false;
                    tourismScopeDistance.neighIdPreviousNode=0;
                    tourismScopeDistance.neighWeightRemain=0;

                    if(neighborTourismScope.find(itNodeTourism->first) != neighborTourismScope.end()) {
                        //If node or neighbor are the previous node it will be deleted, else no
                        if(itNodeTourism->second.second == neighbor.getId() || neighborTourismScope[itNodeTourism->first].second == it->first) {
                            //Delete repeated node in neighborTourismScope map
                            neighborTourismScope.erase(itNodeTourism->first);
                        }
                        else {
                            //Special case, node and neighbor has the same tourism node but each one has different way
                            //P and Q node touch his neighbor
                            if((weightRemainInNode-distance) > 0 || (weightRemainInNeigh-distance) > 0) {
                                weightRemainInNeigh = (weight - neighborTourismScope[itNodeTourism->first].first);
                                //This distance is the point where Q is greater than P and we need to change the tourism
                                //Insert the neighbor
                                partialDistance = (distance+weightRemainInNode-weightRemainInNeigh)/2;
                                tourismScopeDistance.neighIdPreviousNode=neighborTourismScope[itNodeTourism->first].second;
                                //Always negative
                                if((weightRemainInNeigh-distance) > 0)
                                    tourismScopeDistance.neighWeightRemain = (weightRemainInNeigh-distance)*-1;
                                else
                                    tourismScopeDistance.neighWeightRemain = weightRemainInNeigh-distance;
                                //Delete tourismNode from neighbor
                                neighborTourismScope.erase(itNodeTourism->first);
                                tourismScopeDistance.isRepeated=true;
                            }
                        }
                    }
                    noRepeatedTourismScope.insert(pair<double, TourismScopeDistance>(abs(partialDistance), tourismScopeDistance));
                }

                //Add all nodes of neighborTourismScope to noRepeated because we erased repeated nodes of
                // neighborTourism before, but with substracted distance because we use node to generate the scores, not neighbor
                for(map<long, pair<double, long>>::iterator itNeighTourism = neighborTourismScope.begin(); itNeighTourism != neighborTourismScope.end(); ++itNeighTourism) {
                    weight = graphMap->getNode(itNeighTourism->first).getWeight();
                    weightRemainInNode = (weight - itNeighTourism->second.first) - distance;
                    TourismScopeDistance tourismScopeDistance;
                    tourismScopeDistance.idTourismNode=itNeighTourism->first;
                    tourismScopeDistance.idPreviousNode=itNeighTourism->second.second;
                    //Always negative because never touch de node P
                    tourismScopeDistance.weightRemain=weightRemainInNode;
                    tourismScopeDistance.isRepeated=0;
                    tourismScopeDistance.neighWeightRemain=0;
                    tourismScopeDistance.neighIdPreviousNode=0;
                    noRepeatedTourismScope.insert(pair<double, TourismScopeDistance>(abs(weightRemainInNode), tourismScopeDistance));
                }
                std::list<Point> lst;
                scoreMaxPoints[pairNodesIds] = lst;
                if(!noRepeatedTourismScope.empty()) {
                    //4. Guardem el score en un map on tinguem les coordenades. com el scorePoints, pero han destar ordenats per coordenades (distancia del node)
                    //   perque per pintarlos necessitem que esitguin en ordre pel degredat de color.

                    //Calculem el score del node i el posem en el atribut del Node i en el map final on guardarem tots els scores
                    scoreAtPoint = calculateFirstScore(noRepeatedTourismScope, itNeigh->first, nodesIntOne, sumOnes, repeatedTourismNodes);
                    node.setPoint(nodePoint.getX(), nodePoint.getY(), scoreAtPoint);
                    nodePoint = node.getPoint();
                    scoreMaxPoints[pairNodesIds].push_back(nodePoint);
                    if(scoreAtPoint > maxScore) {
                        maxScore = scoreAtPoint;
                        maxScorePoint = nodePoint;
                    }
                    if(scoreAtPoint < minScore) minScore = scoreAtPoint;

                    //Obtenim el vector QP
                    subsX = neighPoint.getX() - nodePoint.getX();
                    subsY = neighPoint.getY() - nodePoint.getY();
                    //3. Per cada node ordenat calcularem el seu score (mirant els nodes de node i de neighbor)
                    for (multimap<double, TourismScopeDistance>::iterator itNoRepeat = noRepeatedTourismScope.begin();
                         itNoRepeat != noRepeatedTourismScope.end(); ++itNoRepeat) {
                        double dist = itNoRepeat->first;
                        bool isNegative = (itNoRepeat->second.weightRemain < 0);
                        long tourismNodeId = itNoRepeat->second.idTourismNode;
                        //FIXME si no em guardo els tourismScope que estiguin als 2 nodes
                        if (dist < distance) {
                            //Ens movem del node al punt amb la distancia dist seguint el vector
                            z = dist / distance;
                            //Coordenades del nou punt on hem de calcular el score perque hi ha hagut un canvi de abast
                            pointX = nodePoint.getX() + z * subsX;
                            pointY = nodePoint.getY() + z * subsY;
                            //Calculem el score de manera senzilla gracies a la estructura que hem creat anteriorment
                            scoreAtPoint = calculateEasyScore(abs(itNoRepeat->second.weightRemain), scoreAtPoint, tourismNodeId, isNegative, nodesIntOne, sumOnes, itNoRepeat->second);
                            curPoint = Point(pointX, pointY, scoreAtPoint);
                            scoreMaxPoints[pairNodesIds].push_back(curPoint);
                            if(scoreAtPoint > maxScore) {
                                maxScore = scoreAtPoint;
                                maxScorePoint = curPoint;
                            }
                            if(scoreAtPoint < minScore) minScore = scoreAtPoint;
                        }
                    }

                    //Calculem el score del vei
                    scoreAtPoint = calculateScore(neighbor.getTourismScope());
                    neighbor.setPoint(neighPoint.getX(), neighPoint.getY(), scoreAtPoint);
                    neighPoint = neighbor.getPoint();
                    if(scoreAtPoint > maxScore) {
                        maxScore = scoreAtPoint;
                        maxScorePoint = neighPoint;
                    }
                    if(scoreAtPoint < minScore) minScore = scoreAtPoint;
                    scoreMaxPoints[pairNodesIds].push_back(neighPoint);
                }
            //}
        }
        //Posem el node a processat
        //processedNodes[node.getId()] = node;
    }
}

void simplifyNeighbors(Node previous, map<long, double> neighbors, double a, double b, double c, long prevPrev, std::list<Node> &deletedNodes, Node &lastNode) {
    map<long, double> neighNeighs;
    Node neighNeigh;
    Point testPoint;
    double dist;

    //Ara busquem els veins del vei per veure si estan a dins de la tolerancia o no
    for (map<long, double>::iterator itNeighNeigh = neighbors.begin(); itNeighNeigh != neighbors.end(); ++itNeighNeigh) {
        //Check if neighbor is different of node
        if (itNeighNeigh->first != prevPrev) {
            //Nomes pot haver-hi un vei que compleixi amb els requisits
            neighNeigh = graphMap->getNode(itNeighNeigh->first);
            testPoint = neighNeigh.getPoint();
            dist = abs(a * testPoint.getX() + b * testPoint.getY() + c);
            //Si aquest vei esta dins la tolerancia, simplificarem
            if (dist <= SIMPLIFY_TOLERANCE) {
                //Marquem el vei anterior com que es podra eliminar i marquem l actual perque es on haurem de fer la aresta amb el node inicial (per ara)
                deletedNodes.push_back(previous);
                lastNode = neighNeigh;
                //continue to other
                neighNeighs = neighNeigh.getNeighbors();
                if(!neighNeigh.isTourismNode() && neighNeighs.size()<= 2) {
                    simplifyNeighbors(neighNeigh, neighNeighs, a, b, c, previous.getId(), deletedNodes, lastNode);
                }
            }
        }
    }
}

void simplifyGraph(list<Node> nodes) {
    //Reuman-witkam
    map<long, double> neighbors, neighNeighs;
    Node node, neigh;
    Point nodePoint, neighPoint;
    long nodeId;
    double v1, v2, modul, a, b, c;
    //La clau es lultim node que entra a la tolerancia i la llista son els que s eliminaran
    std::list<Node> deletedNodes;
    std::list<Node> allDeletedNodes;
    Node lastNode;

    for (list<Node>::iterator it=nodes.begin(); it!=nodes.end(); ++it) {
        node = *it;
        nodeId = node.getId();
        if(graphMap->existNode(nodeId)) {
            //node = it->second;
            nodePoint = node.getPoint();
            neighbors = node.getNeighbors();
            for (map<long, double>::iterator itNeigh = neighbors.begin(); itNeigh != neighbors.end(); ++itNeigh) {
                if (graphMap->existNode(itNeigh->first)) {
                    neigh = graphMap->getNode(itNeigh->first);
                    //The neighbor not is a hotel or attraction
                    if (!neigh.isTourismNode()) {
                        neighNeighs = neigh.getNeighbors();
                        //Check if the nighbor is a simple node with 2 or less neighbors, because we can not delete nodes with more than one edge
                        if (neighNeighs.size() <= 2) {
                            //Ara es un candidat a ser destruit
                            //Construim la linia
                            neighPoint = neigh.getPoint();
                            v1 = neighPoint.getX() - nodePoint.getX();
                            v2 = neighPoint.getY() - nodePoint.getY();
                            modul = sqrt(v1 * v1 + v2 * v2);
                            //Recta ax+by+c=0
                            a = -v2 / modul;
                            b = v1 / modul;
                            c = -(a * nodePoint.getX() + b * nodePoint.getY());
                            lastNode = neigh;
                            simplifyNeighbors(neigh, neighNeighs, a, b, c, nodeId, deletedNodes, lastNode);
                        }
                    }
                    if (deletedNodes.size() > 0) {
                        //Calculem la distancia entre el node inicial i el final
                        Node &nodeMod = graphMap->getNode(nodeId);
                        Node &lastNodeMod = graphMap->getNode(lastNode.getId());
                        Point nodeModPoint = nodeMod.getPoint();
                        Point lastNodeModPoint = lastNodeMod.getPoint();
                        double distance = cartesian_distance(nodeModPoint.getX(), nodeModPoint.getY(),
                                                             lastNodeModPoint.getX(), lastNodeModPoint.getY());
                        //Afegim mutuament els veins
                        nodeMod.addNeighbor(lastNode.getId(), distance);
                        lastNodeMod.addNeighbor(nodeId, distance);

                        //Eliminem tots els nodes que hi havia entre mig i els eliminem del node inicial i final
                        for (std::list<Node>::iterator itDelete = deletedNodes.begin(); itDelete != deletedNodes.end(); ++itDelete) {
                            graphMap->deleteNode(*itDelete);
                        }
                        graphMap->toString();
                        allDeletedNodes.insert(allDeletedNodes.end(), deletedNodes.begin(), deletedNodes.end());
                        deletedNodes.clear();
                        lastNode = Node();
                    }
                }
            }
        }
    }

    //normalitzar recta: ax+by+c=0
    //Per trobar a i b ho fem aixi: v=(q1-p1,q2-p2). Llavors a = -v2/modul de v i b = v1/modul de v i c = (a*p1+b*p2)
    //El modul es: sqrt(v1*v1+v2*v2)
    //El punt es: R=(r1,r2)
    //Distancia es el valor absolut d avaluar el punt R a la recta: d=abs(a*r1+b*r2+c)



/*
    function reumannWitkam(PointList[], Tolerance)
        key=0
        while (key+3 > PointList.length)
            line= new Line(PointList[ key ], PointList[ key+1 ])
        test= key+2
        while ( perpendicularDistance(line, PointList[test]) < Tolerance )
            test++
        for (i=key+1; i<test-1; i++)
            PointList.remove( i )
        key++
    end
  */
}

void cleanFile(string inputfile, string outputfile) {
    char osmfiler[300];
    sprintf(osmfiler, "osmfilter %s --keep=\"highway= AND highway!=cycleway AND highway!=crossing AND highway!=traffic_signals AND highway!=bus_stop\" -o=%s", inputfile.c_str(), outputfile.c_str());
    system(osmfiler);
}

int main(int argc, char **argv) {
    int c;
    string inputfile, cleanfile;
    while((c = getopt(argc, argv, ":i:")) != EOF)
    {
        if(optarg) {
            cout << "Option: " << (char)c << endl;
            cout << "Argument: " << optarg << endl;
            if (c == 'i') {
                cout << optarg << endl;
                inputfile = optarg;
            }
            else {
                cerr << "Missing option." << endl;
                exit(1);
            }
        }
    }

    vector<string> inputname_list = split(inputfile, '.');
    cleanfile = inputname_list[0] + "_clean_cpp." + inputname_list[1];

    cleanFile(inputfile, cleanfile);

    graphMap = Graph::Instance();
    processFile(cleanfile);

    list<Node> undeletedNodes = graphMap->getUndeletedNodes();
    simplifyGraph(undeletedNodes);
    list<Node> deletedNodes = graphMap->getDeletedNodes();
    simplifyGraph(deletedNodes);

    graphMap->calculateMaxMinDistance();


    //Calculate distance of every touristNode to every node
    calculateScopeTourismNodes();

    calculateMaxScorePoints();

    glutInit(&argc, argv); // Initialize GLUT
    glEnable(GL_NORMALIZE);
    glutInitDisplayMode(GLUT_RGB);
    glutInitWindowSize(WIDTH, HEIGHT); // Set the width and height of the window
    glutInitWindowPosition(100, 100); // Set the position of the window
    glutCreateWindow("Graph Map"); // Set the title for the window

    camera.lookAt(EYE, CENTER, UP);
    camera.setOrthographic(LEFT_PLANE, RIGTH_PLANE, TOP_PLANE, BOTTOM_PLANE, NEAR_PLANE, FAR_PLANE);
    camera.resize(WIDTH, HEIGHT);

    glEnable(GL_TEXTURE_2D);
    glEnable(GL_DEPTH);
    glEnable(GL_POINT_SPRITE);
    //keybord
    glutKeyboardFunc(onKeyEvent);
    glutSpecialFunc(processSpecialKeys);

    //mouse
    glutMouseFunc(mousePressEvent);
    glutMotionFunc(mouseActiveMotionEvent); //when a key is pressed and mouse moves
    glutMouseWheelFunc(mouseWheelEvent);

    //Put de coordinates between 0,0 and 1,1
    //glScaled(0,0,0);
    glutDisplayFunc(display); // Tell GLUT to use the method "display" for rendering
    glutMainLoop(); // Enter GLUT's main loop

    cout << "TOT OK FI";
    return 0;
}
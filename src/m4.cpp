#include <vector>
#include <string>
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "m4.h"
#include <limits>
#include <list>
#include <queue>
#include "StreetsDatabaseAPI.h"
#include "mapDataStruct.h"
#include "LatLon.h"
#include <algorithm>
#include <unordered_set>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry.hpp>
#include <limits>


namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

//PSA this was the declaration that the boost library said I should use
typedef bg::model::point<double, 2, bg::cs::cartesian> point;
typedef std::pair<point, unsigned> value;

extern mapDataStruct* mapDataObject;

std::vector<unsigned> traveling_courier(const std::vector<DeliveryInfo>& deliveries, 
                                        const std::vector<unsigned>& depots, 
                                        const float turn_penalty){
    
    
    //vector of booleans to check if delivery has been picked up or dropped off
    std::vector<bool> pickedUp(deliveries.size(), false);
    std::vector<bool> droppedOff(deliveries.size(), false);
    
    //vector of booleans to check if depots is isolated
    std::vector<bool> isolated(depots.size(), false);
    
    //final path to return
    std::vector<unsigned> finalPath;
    
    //temporary paths
    std::vector<unsigned> currentPath;
    
     //variable for infinity
    double inf = std::numeric_limits<double>::infinity();

    //set initial minimum distance for pick up/drop off as infinity
    double minDistance = inf;
    
    double minDistanceDepots = inf;
    
    double minDistanceNext = inf;
    
    //starting intersection is any depots
    unsigned start = depots[0];

    //number of total deliveries
    unsigned int numOfDeliveries = deliveries.size();
    
    //counter for while
    unsigned int numOfDeliveriesCounter = deliveries.size();
    
    //go from first depot to first package pick up
    std::vector<unsigned> checkPath;
    //checkPath.resize(numOfDeliveries);
   
    checkPath = find_path_between_intersections(depots[0], deliveries[0].pickUp, turn_penalty);
    
    while (checkPath.size() == 0){
        checkPath = find_path_between_intersections(depots[1], deliveries[0].pickUp, turn_penalty);
        start = depots[1];
        isolated[0] = true;
    } 
    
    //ID of closest pick up 
    unsigned nearestDelivery;
    
    //vector of visited intersections
    std::vector<unsigned> alreadyVisited;
    
    //resize vector
    //alreadyVisited.resize(numOfDeliveries*2);
    
    alreadyVisited.push_back(start);
    
    LatLon currentPosition = mapDataObject->positionInter[alreadyVisited.back()];
    
    for(unsigned i = 0; i < numOfDeliveries; i++){
        
        //get intersection ID of pick up for current delivery
        unsigned nearestPickUp = deliveries[i].pickUp;
        
        //convert to latlon
        LatLon nearestPosition = mapDataObject->positionInter[nearestPickUp];
        
        unsigned currentDistance = find_distance_between_two_points(currentPosition, nearestPosition);
        
        if(currentDistance < minDistance){
            
            minDistance = currentDistance;
            nearestDelivery = i;
        }
  
    }
    
    //put visited intersection into path of intersections
    alreadyVisited.push_back(deliveries[nearestDelivery].pickUp);
    
    //update boolean vector for pickups and drop off
    pickedUp[nearestDelivery] = true; 
    
    //find path between the intersections
    currentPath = find_path_between_intersections(start, alreadyVisited.back(),
            turn_penalty);
    
    //add path from starting depots to closest pick up to final vector
    finalPath.insert(finalPath.end(), currentPath.begin(), currentPath.end());
    
    //do the rest of the pickups
    while(numOfDeliveriesCounter != 0){
        
        bool isPickedUp = false;
        
        minDistanceNext = inf;
       
         
        //drop off current package
        unsigned dropOff = deliveries[nearestDelivery].dropOff;
        
        //unsigned STARTINTER = alreadyVisited.back();
        //find shortest path to drop off and add to final vector
        currentPath = find_path_between_intersections(alreadyVisited.back(), dropOff, turn_penalty);
        
        //add to final path vector
        finalPath.insert(finalPath.end(), currentPath.begin(), currentPath.end());
        
        //update boolean vector for drop off
        droppedOff[nearestDelivery] = true;
        
        //add intersection into already visited
        alreadyVisited.push_back(dropOff);
        
        //find closest remaining pick up
        LatLon currentPositionNew = mapDataObject->positionInter[alreadyVisited.back()];
        
        for(unsigned i = 0; i < numOfDeliveries; i++){
            
            //std::cout<<i<<std::endl;
            //minDistance = inf;
            //get intersection ID of pick up for current delivery
            unsigned nextPickUp = deliveries[i].pickUp;
            unsigned nextDropOff = deliveries[i].dropOff;
            
            if(pickedUp[i] == false && droppedOff[i] == false){
            
             //convert to latlon
            LatLon nextPosition = mapDataObject->positionInter[nextPickUp];
            
            unsigned currentDistance = find_distance_between_two_points(currentPositionNew, nextPosition);
        
            if(currentDistance < minDistanceNext){
            
                minDistanceNext = currentDistance;
                nearestDelivery = i;
                isPickedUp = true;
            }
          }
            else if(pickedUp[i] == true && droppedOff[i] == false){
                
                LatLon nextDropOffPosition = mapDataObject->positionInter[nextDropOff];
                
                unsigned currentDistance = find_distance_between_two_points(currentPositionNew, 
                        nextDropOffPosition);
                
                if(currentDistance < minDistanceNext){
            
                minDistanceNext = currentDistance;
                nearestDelivery = i;
                isPickedUp = true;
                
                }
                
            }
            
        }
        
        if(isPickedUp){
            
            
         //put visited intersection into path of intersections
        alreadyVisited.push_back(deliveries[nearestDelivery].pickUp);
    
        //update boolean vector for pickups and drop off
        pickedUp[nearestDelivery] = true; 
    
        //find path between the intersections
        currentPath = find_path_between_intersections(alreadyVisited.rbegin()[1], 
                alreadyVisited.back(), turn_penalty);
    
        //add path from starting depots to closest pick up to final vector
        finalPath.insert(finalPath.end(), currentPath.begin(), currentPath.end());
        
        
        }
        
        numOfDeliveriesCounter--; 
        
    }
    
    //holds closest depots ID
    unsigned closestDepots;
    
    //get current position
    LatLon positionNow = mapDataObject->positionInter[alreadyVisited.back()];
    
    //find closest depot
    for(unsigned i = 0; i < depots.size(); i++){
        
        if(!isolated[i]){
        
        LatLon depotsPosition = mapDataObject->positionInter[depots[i]];
        
        unsigned currentDistanceDepots = find_distance_between_two_points(positionNow, depotsPosition);
        
        if(currentDistanceDepots < minDistanceDepots){
            
            minDistanceDepots = currentDistanceDepots;
            closestDepots = depots[i];
        }
       }   
    }
    
 
    //find path from current position to closest depots
    currentPath = find_path_between_intersections(alreadyVisited.back(), closestDepots, turn_penalty);
    
    //add to final vector
    finalPath.insert(finalPath.end(), currentPath.begin(), currentPath.end());
    
   return finalPath; 
 
}
        
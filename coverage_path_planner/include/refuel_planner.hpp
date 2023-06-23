#ifndef REFUEL_PLANNER_H
#define REFUEL_PLANNER_H

#include "../build/proto/grid.grpc.pb.h"
#include "ros_interface.h"
#include <polygon_coverage_geometry/cgal_definitions.h>


/*

TODO:

test the planning algrithem on grid test if possiable

*/ 
class RefuelPlanner{
    public:
        RefuelPlanner(std::vector<hecto::mission::common::Point> route,
                        hecto::mission::common::Point dockPos,
                        hecto::mission::common::Point delimiter,
                        double gridOptM){
            dockPos_ = Point_2(dockPos.latitudedeg(), dockPos.longitudedeg());
            delimiter_ = Point_2(delimiter.latitudedeg(), delimiter.longitudedeg());
            for(const auto p : route){
                auto temp = Point_2(p.latitudedeg(), p.longitudedeg());
                route_.push_back(temp);
            }
            gridOptM_ = gridOptM;

        }

        ~RefuelPlanner(){

        }

        std::vector<Point_2> getRefuelPlanGridlize(){      
            if(!computeGridRoute()){
                ROS_ERROR("Fail to grid-lize the route");
            }           
            if(!computeRefuelPlanDirect()){
                ROS_ERROR("Fail to compute refuel plan");
            }
            return plan_;
        }


    private:
        bool decomposeGridRoute(std::vector<Point_2> routeClean,std::vector<Point_2> gridRoute){
            return true;
        }

        bool computeRefuelPlanDirect(){
            bool endOfPlan = false;
            double dist = 0;
            gridPlan_ = gridRoute_;
            plan_ = routeClean_;
            std::vector<Point_2>::iterator it0 = gridPlan_.begin()+1;            
            std::vector<Point_2>::iterator it1;
            std::vector<Point_2>::iterator it2;
            double distTotal = getTotalTravelDistanceM(route_);
            int maxDockCnt = distTotal/max_distace_single_tank_+1;

            for(dockCount = 0;dockCount < maxDockCnt;dockCount++){
                for(it1 = it0;it1 != gridPlan_.end();++it1){
                    double rem_dis =  getRemainTravelDistanceM(dist);                   
                    if(max_distace_single_tank_< getDistanceLat2M(*(it1-1),*it1)+getDistanceLat2M(*it1,dockPos_)){
                        std::cout << "Exceed max distance for single tank of fuel. Abort\n";
                        return false;
                    }
                    if(rem_dis < getDistanceLat2M(*(it1-1),*it1)+getDistanceLat2M(*it1,dockPos_)){
                        it0 = it1;
                        dist = 0;
                        break;
                    }
                    dist += getDistanceLat2M(*(it1-1),*it1);
                    if(it1+1 == gridPlan_.end()){
                        if(rem_dis>getDistanceLat2M(*it1,*(it1+1))){
                            //end of plan
                            endOfPlan = true;
                        }else{
                            it0 = it1;
                        }
                    }
                }
                if(endOfPlan) break;


                for(it1 = plan_.begin();it1 != plan_.end();++it1){
                    if(*it1 == *(it0-1)){
                        plan_.insert(it1+1,dockPos_);
                        break;
                    }else if(*it1 == *it0){
                        plan_.insert(it1,dockPos_);
                        break;                       
                    }else{
                        Line_2 l = Line_2(*(it1-1),*it1);                       
                        if(l.has_on(*(it0-1)) && l.has_on(*it0)){
                            plan_.insert(it1,*it0);
                            plan_.insert(it1,dockPos_);
                            plan_.insert(it1,*(it0-1));
                            break;
                        }else{
                            std::cout<<"Docking route invalid. Abort\n";
                            return false;
                        }
                    }
                }
                gridPlan_.insert(it0,dockPos_);
            }
            return true;
        }

        bool computeGridRoute(){
            bool gridFlag = false;
            for(int i = 0;i<route_.size()-1;i++){
                if(route_[i] == delimiter_){
                    gridFlag = !gridFlag;
                }else{
                    if(gridFlag && route_[i+1] != delimiter_){
                        int cnt = (int)(getDistanceLat2M(route_[i],route_[i+1])/gridOptM_)+1;
                        double dx = (CGAL::to_double(route_[i].x())-CGAL::to_double(route_[i+1].x()))/(double)cnt;
                        double dy = (CGAL::to_double(route_[i].y())-CGAL::to_double(route_[i+1].y()))/(double)cnt;
                        for(int j = 0;j<cnt;j++){
                            gridRoute_.push_back(Point_2(CGAL::to_double(route_[i].x())+dx*j,CGAL::to_double(route_[i].y())+dy*j));
                        }
                    }else{
                        gridRoute_.push_back(route_[i]);
                    }
                    routeClean_.push_back(route_[i]);
                }
            }
            return true;
        }

        double getTotalTravelDistanceM(std::vector<Point_2> route){
            double dist;
            for(int i=0;i<route.size()-1;i++){
                dist += getDistanceLat2M(route[i],route[i+1]); 
            }
            return dist;
        }

        //Compute remaining travel distance with model of energy consumption
        inline double getRemainTravelDistanceM(double dist){
            
            return max_distace_single_tank_ - dist;
        }
        
        //Haversine formula
        //https://en.wikipedia.org/wiki/Haversine_formula
        inline double getDistanceLat2M(Point_2 p1,Point_2 p2){
            double R = 6378.137; // Radius of earth in KM
            double dLat = mDeg2rad(CGAL::to_double(p2.x()))- mDeg2rad(CGAL::to_double(p1.x()));
            double dLon = mDeg2rad(CGAL::to_double(p2.y())) - mDeg2rad(CGAL::to_double(p1.y()));
            double a = sin(dLat/2) * sin(dLat/2) +cos(mDeg2rad(CGAL::to_double(p1.x()))) *cos(mDeg2rad(CGAL::to_double(p2.x()))) *sin(dLon/2) * sin(dLon/2);
            double c = 2 * atan2(sqrt(a), sqrt(1-a));
            double d = R * c;
            return d * 1000; // meters
        }

        std::vector<Point_2> route_;        //route with delimiter
        std::vector<Point_2> routeClean_;   //route without delimiter
        std::vector<Point_2> gridRoute_;    //route grid-lized
        std::vector<Point_2> gridPlan_;     //plan grid-lized
        std::vector<Point_2> plan_;         //plan not grid-lized
        int dockCount;
        Point_2 dockPos_;
        const double max_distace_single_tank_ = 40;
        double gridOptM_;
        Point_2 delimiter_;
        

};


#endif
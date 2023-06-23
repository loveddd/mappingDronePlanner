#ifndef GRID_SERVICE_H
#define GRID_SERVICE_H

#include "../build/proto/grid.grpc.pb.h"

#include "coverage_planner.h"
#include "polygon_coverage_planners/planners/polygon_stripmap_planner_exact_preprocessed.h"
#include "refuel_planner.hpp"

using namespace grpc;
using namespace hecto::mission::common;
using namespace hecto::mission::grid;
using namespace polygon_coverage_planning;

class GridGrpcService final : public hecto::mission::grid::GridService::Service {
    public:
        GridGrpcService() {

        }

        inline Status grenerateRefuelPlan(ServerContext* context, const Mission* mission, RefuelPlanResponse* response){
            // Get route(vector of points)
            std::vector<Point> route;
            Point delimiter;
            delimiter.set_latitudedeg(200);
            delimiter.set_longitudedeg(200);
            Point dockingPos;
            double gridOptM = 5;
            for (const auto m : mission->actions()){
                if(m.actiontype() == Waypoint){
                    Point p;
                    p.set_latitudedeg(m.location().latitudedeg());
                    p.set_longitudedeg(m.location().longitudedeg());
                    route.push_back(p);
                }
                else if(m.actiontype() == Grid){
                    route.push_back(delimiter);
                    for(const auto i : m.grid().route()){
                        route.push_back(i);
                    }
                    route.push_back(delimiter);
                }else if(m.actiontype() == MissionStart){
                    //TODO: get docking pos
                    dockingPos.set_latitudedeg(0);
                    dockingPos.set_longitudedeg(0);
                }
            }

            
            RefuelPlanner planner(route,dockingPos,delimiter,gridOptM);
            std::vector<Point_2> route_2 = planner.getRefuelPlanGridlize(); 

            // TODO:

            for(const auto p : route_2){
                auto p1 = response->add_points();
                p1->set_latitudedeg(CGAL::to_double(p.x()));
                p1->set_longitudedeg(CGAL::to_double(p.y()));
            }

            return Status::OK;
        }


        inline Status calculateMissionTime(ServerContext* context, const Mission* mission, MissionTimeResponse* response)
        {
            double t = 0.0;
            const double a_max = mission->settings().maxflightaccms();
            const double v_max = mission->settings().maxflightspeedms();

            for(const auto p : mission->actions()){
                Point prev_point;
                double prev_height;
                switch(p.actiontype()){
                    case MissionStart:
                    {
                        ROS_DEBUG("Mission start at: %f s",t);
                        if(p.missionstart().startinglocation() == CurrentPosition){
                            prev_point.set_latitudedeg(p.location().latitudedeg());
                            prev_point.set_longitudedeg(p.location().longitudedeg());
                            prev_height = 0;
                        }else if(p.missionstart().startinglocation() == DockingPosition){
                            //prev point = DockingPosition
                        }
                        t +=  p.location().relativealtitudem(); //take off speed is 1 m/s
                        ROS_DEBUG("Takeoff finish at: %f s",t);
                        break;
                    }

                    case Grid: 
                    {
                        ROS_DEBUG("Grid action start at: %f s",t);
                        for(int i=0;i<p.grid().route().size()-1;i++){
                            double dis = dis_lat2meter(p.grid().route()[i],p.grid().route()[i+1]);
                            t+= computeTravelTime2D(dis,v_max,a_max);
                        }
                        ROS_DEBUG("Grid action finish at: %f s",t);
                        break;
                    } 

                    case Waypoint:
                    {
                        //First way point
                        ROS_DEBUG("Waypoint action start at: %f s", t);
                        if(prev_point.latitudedeg() == 0 && prev_point.longitudedeg() == 0){
                            ROS_DEBUG("First Waypoint not set");
                            break;
                        }

                        Point temp;
                        temp.set_latitudedeg(p.location().latitudedeg());
                        temp.set_longitudedeg(p.location().longitudedeg());
                        const double dis = dis_lat2meter(prev_point,temp);
                        const double heightDiff = p.location().absolutealtitudem()-prev_height;
                        // const double dis_3d = sqrt(dis*dis+heightDiff*heightDiff);
                        t += computeTravelTime2D(dis,v_max,a_max);

                        //new prev_point
                        prev_point.set_latitudedeg(p.location().latitudedeg());
                        prev_point.set_longitudedeg(p.location().longitudedeg());
                        prev_height = p.location().absolutealtitudem();

                        ROS_DEBUG("Waypoint action finished at: %f s", t);
                        break;
                    }  

                    case MissionEnd:
                    {
                        ROS_DEBUG("Ending action start at: %f s", t);
                        if(p.missionend().shouldland()){
                            t += p.location().relativealtitudem(); // assume landing speed 1 m/s
                        } else{
                            //hover
                            t += 0;
                        }
                        ROS_DEBUG("Ending action finished at: %f s", t);
                        break;
                    }

                }
            }
            response->set_seconds(t);
            return Status::OK;
        }


        inline Status generateGrid(ServerContext* context, const GridOptions* request, GenerateGridResponse* response) {
            // init
            std::cout << "Current lanedistance(m): "<<request->lanedistancem() << "\n";
            if(request->sweepwithpresetdirection()){
                std::cout << "Sweeping with pre-set direction"<<"\n";
                std::cout << "Current rotation deg: "<<request->rotationdeg() << "\n";
            }else{
                std::cout << "Sweeping with optimal direction"<<"\n";
            }

            std::cout << "Entry point: "<<request->entrypoint().latitudedeg() <<"  "<<request->entrypoint().longitudedeg() <<"\n";
            std::cout << "Goal point: "<<request->goalpoint().latitudedeg() <<"  "<<request->goalpoint().longitudedeg() <<"\n";
            const double scalar = (40075*cos(mDeg2rad(request->polygon()[0].latitudedeg()))/360*cos(mDeg2rad(request->rotationdeg()))+111.32*sin(mDeg2rad(request->rotationdeg())))*1000;
            std::cout << "latitude to meter: "<< 1.0/scalar << "\n";

            const Point_2 start(request->entrypoint().latitudedeg(),request->entrypoint().longitudedeg());
            const Point_2 goal(request->goalpoint().latitudedeg(),request->goalpoint().longitudedeg());

            polygon_coverage_planning::CoveragePlanner<PolygonStripmapPlannerExactPreprocessed> mPlanner(request);

            mPlanner.planPath(start,goal);
            std::vector<Point_2> mGrids;
            std::vector<Polygon_2> mBCD;
            mGrids = mPlanner.publishVisualization();
            // std::cout << "Grid size: " << mGrids.size() << "\n";
            mBCD = mPlanner.getDecomposition();
            // std::cout << "BCD size: " << mBCD.size() << "\n";

            for (const auto p : mGrids) {
                auto p1 = response->add_points();
                p1->set_latitudedeg(CGAL::to_double(p.x()));
                p1->set_longitudedeg(CGAL::to_double(p.y()));

            }

            std::cout << "Grid Generated\n";

            return Status::OK;
        }
    
    private:
        //Haversine formula
        //https://en.wikipedia.org/wiki/Haversine_formula
        inline double dis_lat2meter(Point p1,Point p2){
            double R = 6378.137; // Radius of earth in KM
            double dLat = mDeg2rad(p2.latitudedeg()) - mDeg2rad(p1.latitudedeg());
            double dLon = mDeg2rad(p2.longitudedeg()) - mDeg2rad(p1.longitudedeg());
            double a = sin(dLat/2) * sin(dLat/2) +cos(mDeg2rad(p1.latitudedeg())) *cos(mDeg2rad(p2.latitudedeg())) *sin(dLon/2) * sin(dLon/2);
            double c = 2 * atan2(sqrt(a), sqrt(1-a));
            double d = R * c;
            return d * 1000; // meters
        }

        inline double computeTravelTime2D(const double distance,double v_max, double a_max) {
            // Time to accelerate or decelerate to or from maximum velocity:
            const double acc_time = v_max / a_max;
            // Distance covered during complete acceleration or decelerate:
            const double acc_distance = 0.5 * v_max * acc_time;
            // Compute total segment time:
            if (distance < 2.0 * acc_distance) {
                // Case 1: Distance too small to accelerate to maximum velocity.
                return 2.0 * std::sqrt(distance / a_max);
            } else {
                // Case 2: Distance long enough to accelerate to maximum velocity.
                return 2.0 * acc_time + (distance - 2.0 * acc_distance) / v_max;
            }
        }


};

#endif
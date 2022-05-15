/*
#  This is the template for cpp Source Codes of different trajectory
#  planner in unstructured environments.The randomly generated obstacles are
#  oriented in different directions.
*/
#include <iostream>
#include <math.h>
#include <vector>
#include "vec2d.h"
#include <opencv2/opencv.hpp>
#include "mathstruct.h"
#include "TransformP2T.h"
#include "Visualize.h"
#include "PlanPRMPath.h"
#include "bezier.h"
using namespace cv;

/* run this program using the console pauser or add your own getch, system("pause") or input loop */
using namespace std;
struct Vehicle_geometrics_ vehicle_geometrics_;
struct Vehicle_kinematics_ vehicle_kinematics_;
struct Hybrid_astar_ hybrid_astar_;
struct Vehicle_TPBV_ vehicle_TPBV_;
struct Planning_scale_ planning_scale_;

int num_nodes_s = 60;
double margin_obs_ = 0.5;
int Nobs = 10;

vector < vector<math::Vec2d>> obstacles_;
vector<math::Vec2d> path;

int main(int argc, char** argv) {
    obstacles_ = GenerateStaticObstacles_unstructured();
    struct Trajectory trajectory = PlanPRMPath(obstacles_).first;
    
    //visual PRMpath
    //VisualizeStaticResults(trajectory);
    
    double path_length = PlanPRMPath(obstacles_).second;
    for (int i = 0; i < trajectory.x.size(); i++) {
        math::Vec2d node(trajectory.x[i], trajectory.y[i]);
        path.push_back(node);
    }
    Bezier::bezier B_curves(path, obstacles_);
    vector<Bezier::bounding_box> box_list = B_curves.get_box_list();
    double obj = 0;
    int res = B_curves.BezierPloyCoeffGeneration(obj);
    struct Trajectory bezier_path = B_curves.get_bezier_path(path_length);
    VisualizeStaticResults(bezier_path, box_list);
    VisualizeDynamicResults(bezier_path);
    return 0;
}
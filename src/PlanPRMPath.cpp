#include "PlanPRMPath.h"
#include "mathstruct.h"
#include <algorithm>
#include <cmath>
#include <queue>
#include <set>

vector<math::Vec2d> getvertex(unsigned int k, vector<vector<math::Vec2d>> obstacles_cell) {
    math::Vec2d source_vertex(vehicle_TPBV_.x0, vehicle_TPBV_.y0);
    math::Vec2d goal_vertex(vehicle_TPBV_.xtf, vehicle_TPBV_.ytf);
    vector<math::Vec2d> vertex;
    vertex.push_back(source_vertex);
    vertex.push_back(goal_vertex);
    try {
        std::string error = "";
        if (!Is3DNodeValid(vehicle_TPBV_.x0, vehicle_TPBV_.y0, vehicle_TPBV_.theta0, obstacles_cell)) {
            error = "vehicle can not start path correctly";
            throw error;
        }
        if (!Is3DNodeValid(vehicle_TPBV_.xtf, vehicle_TPBV_.ytf, vehicle_TPBV_.thetatf, obstacles_cell)) {
            error = "vehicle can not end path correctly";
            throw error;
        }
    }
    catch (std::string error) {
        std::cout << error << std::endl;
    }
    double lx = planning_scale_.xmin;
    double ux = planning_scale_.xmax;
    double ly = planning_scale_.ymin;
    double uy = planning_scale_.ymax;

    while (vertex.size() < k + 2) {
        double x = (ux - lx) * rand01() + lx;
        double y = (uy - ly) * rand01() + ly;
        math::Vec2d point2Dim(x, y);
        if (Is2DNodeValid(x, y, obstacles_cell)) {
            vertex.push_back(point2Dim);
        }
    }
    return vertex;
}

vector<vector<int>> getedges(vector<math::Vec2d> vertex, vector<vector<math::Vec2d>> obstacles_cell) {
    unsigned int NumofVertex = vertex.size();
    vector<vector<int>> edges(NumofVertex);
    for (unsigned int i = 0; i < NumofVertex; i++) {
        for (unsigned int j = i + 1; j < NumofVertex; j++) {
            math::Vec2d p1 = vertex[i];
            math::Vec2d p2 = vertex[j];
            double angle = (p2 - p1).Angle();
            double distance = (p2 - p1).Length();
            vector<math::Vec2d> v_fea = CreateVehiclePolygon(p1, p2, angle);
            
            bool Isfeasible_Line = 1;
            for (unsigned int i = 0; i < obstacles_cell.size(); i++) {
                if (IsCross(v_fea, obstacles_cell[i])) {
                    Isfeasible_Line = 0;
                    break;
                }
            }
            
            if (Isfeasible_Line)
            {
                edges[i].push_back(j);
                edges[j].push_back(i);
            }

        }
    }
    return edges;
}
pair <struct Trajectory, double> PlanPRMPath(vector<vector<math::Vec2d>> obstacles_cell) {
    unsigned int k = 20;
    vector<math::Vec2d> vertex = getvertex(k, obstacles_cell);
    vector<vector<int>> edges = getedges(vertex, obstacles_cell);

    priority_queue<AstarNode*> openlist_;
    AstarNode* init_node = new AstarNode(sizeof(AstarNode));
    init_node->g = 0;
    init_node->h = vertex[0].DistanceTo(vertex[1]);
    init_node->f = init_node->g + init_node->h;
    init_node->parent = nullptr;
    init_node->vertex_order = 0;
    openlist_.push(init_node);

    set<int> closed;

    bool completeness_flag = 0;
    double path_length = 0;
    int iter = 0;

    vector<math::Vec2d> path;
    AstarNode* prev = nullptr;
    const int max_iter = 500;
    int iternum = 0;
    while (!openlist_.empty() && (!completeness_flag) && iternum < max_iter) {
        AstarNode* cur_node = openlist_.top();
        openlist_.pop();
        closed.insert(cur_node->vertex_order);
        std::cout << cur_node->vertex_order << std::endl;
        for (unsigned int edge = 0; edge < edges[cur_node->vertex_order].size(); edge++) {
            int newVertex = edges[cur_node->vertex_order][edge];
            double child_g = cur_node->g + vertex[cur_node->vertex_order].DistanceTo(vertex[newVertex]);
            double child_h = vertex[1].DistanceTo(vertex[newVertex]);
            AstarNode* child_node = new AstarNode(newVertex,child_g,child_h,child_g+child_h);
            child_node->parent = cur_node;
            if (closed.find(newVertex) == closed.end() && !find(openlist_, newVertex)) {
                prev = child_node;
                if (child_node->vertex_order == 1) {
                    path_length = child_node->f;
                    completeness_flag = 1;
                    break;
                }
                openlist_.push(child_node);
            }
        }
        iternum += 1;
    }

    std::cout << "complete " << completeness_flag << std::endl;

    //getpath
    while (prev != nullptr) {
        math::Vec2d curp = vertex[prev->vertex_order];
        path.push_back(curp);
        prev = prev->parent;
    }
    for (int i = 0; i < path.size(); i++) {
        std::cout << path[i].x() << ' ' << path[i].y() << std::endl;
    }
    reverse(path.begin(), path.end());



    //resample
    vector<double> x;
    vector<double> y;
    vector<double> theta;
    if (completeness_flag) {
        path = Resample(path);
    }
    for (unsigned int i = 0; i < path.size() - 1; i++) {
        x.push_back(path[i].x());
        y.push_back(path[i].y());
        double angle = (path[i + 1] - path[i]).Angle();
        theta.push_back(angle);
    }
    x.push_back(path.back().x());
    y.push_back(path.back().y());
    theta.push_back(0);
    pair <struct Trajectory, double> ret = make_pair(TransformPathToTrajectory(x, y, theta, path_length, 1), path_length);
    return ret;
}

vector<math::Vec2d> Resample(vector<math::Vec2d> vec) {
    vector<math::Vec2d> res;
    for (int i = 0; i < vec.size() - 1; i++) {
        double distance = vec[i].DistanceTo(vec[i + 1]);
        double delta_x = (vec[i + 1].x() - vec[i].x()) / distance;
        double delta_y = (vec[i + 1].y() - vec[i].y()) / distance;
        for (int j = 0; j < distance; j++) {
            math::Vec2d a(j * delta_x, j * delta_y);
            res.push_back(vec[i] + a);
        }
    }
    res.push_back(vec.back());
    return res;
}



bool find(priority_queue<AstarNode*> openlist, int vertex_order) {
    while (!openlist.empty()) {
        AstarNode* cur = openlist.top();
        if (cur->vertex_order == vertex_order)
        {
            return true;
        }
        openlist.pop();
    }
    return false;
}

vector<double> linspace(double pos1, double pos2, int n) {
    vector<double> linvec;
    double d = (pos2 - pos1) / (n - 1);
    for (int i = 0; i < n; i++) {
        linvec.push_back(pos1 + i * d);
    }
    return linvec;
}

bool Is2DNodeValid(double x, double y, vector<vector<math::Vec2d>> obstacles_cell) {
    double x_inf = planning_scale_.xmin;
    double x_sup = planning_scale_.xmax;
    double y_inf = planning_scale_.ymin;
    double y_sup = planning_scale_.ymax;
    if (x > x_sup || x < x_inf ||
        y > y_sup || y < y_inf) {
        return false;
    }

    for (unsigned int i = 0; i < obstacles_cell.size(); i++) {
        if (PtInPolygon(x, y, obstacles_cell[i])) {
            return false;
        }
    }
    return true;
}

bool Is3DNodeValid(double x, double y, double theta, vector<vector<math::Vec2d>> obstacles_cell) {
    double xr = x + vehicle_geometrics_.r2x * cos(theta);
    double a = std::cos(theta);
    double yr = y + vehicle_geometrics_.r2x * sin(theta);
    double xf = x + vehicle_geometrics_.f2x * cos(theta);
    double yf = y + vehicle_geometrics_.f2x * sin(theta);
    double x_inf = planning_scale_.xmin + vehicle_geometrics_.radius * 1.01;
    double x_sup = planning_scale_.xmax - vehicle_geometrics_.radius * 1.01;
    double y_inf = planning_scale_.ymin + vehicle_geometrics_.radius * 1.01;
    double y_sup = planning_scale_.ymax - vehicle_geometrics_.radius * 1.01;
    if (xr > x_sup || xr < x_inf ||
        xf > x_sup || xf < x_inf ||
        yr > y_sup || yr < y_inf ||
        yf > y_sup || yf < y_inf) {
        return false;
    }
    vector<math::Vec2d> V_temp = CreateVehiclePolygon(x, y, theta);
    for (unsigned int i = 0; i < obstacles_cell.size(); i++) {
        if (IsCross(V_temp, obstacles_cell[i])) {
            return false;
        }
    }
    return true;
}

bool IsCross(vector<math::Vec2d> vehicle, vector<math::Vec2d> obstacle) {
    bool IsCross = false;
    for (unsigned int i = 0; i < vehicle.size() - 1; i++) {
        if (checkObj_linev(vehicle[i], vehicle[i + 1], obstacle)) {
            IsCross = true;
            break;
        }
    }
    return IsCross;
}

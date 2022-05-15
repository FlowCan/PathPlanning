#pragma once
#include "TransformP2T.h"
#include "vec2d.h"
#include <opencv2/opencv.hpp>

class AstarNode
{
public:
	int vertex_order;
	double g;
	double h;
	double f;
	AstarNode* parent;
public:
	AstarNode(int vertex_order_ = 0, double g_ = 0, double h_ = 0, double f_ = 0) :vertex_order(vertex_order_), g(g_), h(h_), f(f_), parent(nullptr) {

	}
	bool operator<(const AstarNode& a) {
		return f > a.f;
	}
};


vector<math::Vec2d> getvertex(unsigned int k, vector<vector<math::Vec2d>> obstacles_cell);
vector<vector<int>> getedges(vector<math::Vec2d> vertex, vector<vector<math::Vec2d>> obstacles_cell);
pair <struct Trajectory, double> PlanPRMPath(vector<vector<math::Vec2d>> obstacles_cell);
vector<math::Vec2d> Resample(vector<math::Vec2d> vec);
bool find(priority_queue<AstarNode*> openlist, int vertex_order);
vector<double> linspace(double pos1, double pos2, int n);
bool Is2DNodeValid(double x, double y, vector<vector<math::Vec2d>> obstacles_cell);
bool Is3DNodeValid(double x, double y, double theta, vector<vector<math::Vec2d>> obstacles_cell);
bool IsCross(vector<math::Vec2d> vehicle, vector<math::Vec2d> obstacle);
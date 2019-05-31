/*******************************
 *                             *
 *  Author: Rein Appeldoorn    *
 *  Date:   2013-03-06         *
 *                             *
 *******************************/

#ifndef cb_global_planner_ASTARPLANNER_H_
#define cb_global_planner_ASTARPLANNER_H_

#include <float.h> // for DBL_MAX
#include <math.h> // for sqrt

#include <list>
#include <vector>
#include <queue>
#include <stdio.h>
#include <iostream>

namespace cb_global_planner {

class AStarPlanner {

public:

	AStarPlanner(int width, int height);

	virtual ~AStarPlanner();

	//void setCost(int x, int y, double cost);

    void setCostmap(const unsigned char* char_cost_map);

	void resize(int nx, int ny);

    bool plan(std::vector<unsigned int> mx_start, std::vector<unsigned int> my_start, int mx_goal, int my_goal, std::vector<int>& plan_xs, std::vector<int>& plan_ys, bool best_heuristic = false);

protected:

	static constexpr double SQRT2 = 1.414213562;

	static long N_OBJECTS;

	//double** cost_map_;

	const unsigned char* char_cost_map_;

	double** visited_map_;
	unsigned int width_;
	unsigned int height_;

	void deleteMap();

	double getCost(int x, int y);

	struct CellInfo {

		static long N_OBJECTS;

		int x_;
		int y_;
		double f_;   // f = g + h
		double g_;   // g = cost so far
		double h_;   // h = predicted extra cost
		CellInfo* visited_from_; // pointer to cell from which this cell is visited

		CellInfo(double x, double y, double g, double h) : x_(x), y_(y), g_(g), h_(h), visited_from_(0) {
			f_ = g_ + h_;
			++N_OBJECTS;
		}

		~CellInfo() {
			--N_OBJECTS;
		}
	};

	struct compareCellInfos {
	    bool operator()(const CellInfo* c1, const CellInfo* c2) const {
	    	return c1->f_ > c2->f_;
	   }
	};

	void expandCell(CellInfo* c, int dx, int dy, double cost_factor, double** visited_map,
			int x_goal, int y_goal, double min_cell_cost,
			std::priority_queue<CellInfo*, std::vector<CellInfo*>, compareCellInfos>& Q);

	double calculateHeuristicCost(int x, int y, int x_goal, int y_goal, double min_cell_cost);

};

}

#endif /* ASTARPLANNER_H_ */

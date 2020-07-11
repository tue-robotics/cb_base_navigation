#include "a_star_planner.h"

#include <costmap_2d/cost_values.h>

namespace cb_global_planner {

long AStarPlanner::N_OBJECTS = 0;

long AStarPlanner::CellInfo::N_OBJECTS = 0;

AStarPlanner::AStarPlanner(int width, int height) : visited_map_(0) {
	resize(width, height);
	++N_OBJECTS;
}

AStarPlanner::~AStarPlanner() {
	deleteMap();
	--N_OBJECTS;
}

/*
void AStarPlanner::setCost(int x, int y, double cost) {
	cost_map_[x][y] = cost;
}
*/

void AStarPlanner::setCostmap(const unsigned char* char_cost_map) {
	char_cost_map_ = char_cost_map;
}

double AStarPlanner::getCost(int x, int y) {
	int k = width_ * y + x;

    unsigned char cost = char_cost_map_[k];

    if (cost == costmap_2d::NO_INFORMATION)
    //    cost = 0;
        return DBL_MAX; // Do not plan through unknown space

    double cell_pass_through_time = DBL_MAX;

    if (cost != costmap_2d::INSCRIBED_INFLATED_OBSTACLE && cost != costmap_2d::LETHAL_OBSTACLE)
    {
        double max_vel = (1 - (double) cost / 256) * 1.0; //max_velocity_;
        cell_pass_through_time = 1 / max_vel; // cell_size
    }

	//printf("k = %d, cost = %f\n", k, cell_pass_through_time);

    return cell_pass_through_time;
}

void AStarPlanner::resize(int width, int height) {
	if (visited_map_) {
		deleteMap();
	}

	width_ = width;
	height_ = height;

	visited_map_ = new double*[width_];
	for(unsigned int i = 0; i < width_; ++i) {
		visited_map_[i] = new double[height_];
	}

	// to make sure the algorithm won't expand off the map, mark all border cells as visited
	// todo: this limits the search space: border cells can now not be used in the solution. FIX
	for(unsigned int x = 0; x < width_; ++x) {
		visited_map_[x][0] = 0;
		visited_map_[x][height_ - 1] = 0;
	}

	for(unsigned int y = 0; y < height_; ++y) {
		visited_map_[0][y] = 0;
		visited_map_[width_ - 1][y] = 0;
	}
}

bool AStarPlanner::plan(std::vector<unsigned int> mx_start, std::vector<unsigned int> my_start, int mx_goal, int my_goal, std::vector<int>& plan_xs, std::vector<int>& plan_ys, bool best_heuristic) {
	// - initialize all cells in visited_map to false
	// - determine minimum cell cost in cost map
	double min_cell_cost = 1; //DBL_MAX;
	for(unsigned int x = 1; x < width_ - 1; ++x) {
		for(unsigned int y = 1; y < height_ - 1; ++y) {
			visited_map_[x][y] = DBL_MAX;
			//min_cell_cost = min(getCost(x, y), min_cell_cost);
		}
	}

    std::priority_queue<CellInfo*, std::vector<CellInfo*>, compareCellInfos> Q;

    std::list<CellInfo*> visited_cells;   // remember visited cells to be able to clear memory

    // add start points to priority queue and visited map
    for (unsigned int i = 0; i < mx_start.size(); ++i)
    {
        if (mx_start[i] > 0 && mx_start[i] < width_-1 && my_start[i] > 0 && my_start[i] < height_-1)
        {
            CellInfo* start = new CellInfo(mx_start[i], my_start[i], 0, calculateHeuristicCost(mx_start[i], my_start[i], mx_goal, my_goal, min_cell_cost));
            visited_map_[mx_start[i]][my_start[i]] = 0;
            Q.push(start);
        }
    }

	CellInfo* goal_cell = 0;
    CellInfo* best_cell = 0;

    double best_score = 1e9;

	while(!Q.empty() && !goal_cell) {
		CellInfo* c = Q.top();
		visited_cells.push_back(c);
		Q.pop();

		// check if goal is reached
		if (c->x_ == mx_goal && c->y_ == my_goal) {
			// goal reached!
			goal_cell = c;
        } else {

            if (c->h_ < best_score) {
                best_cell = c;
                best_score = c->h_;
            }

			// direct adjacent expansion
			expandCell(c, -1, 0, 1.0, visited_map_, mx_goal, my_goal, min_cell_cost, Q);
			expandCell(c, +1, 0, 1.0, visited_map_, mx_goal, my_goal, min_cell_cost, Q);
			expandCell(c, 0, -1, 1.0, visited_map_, mx_goal, my_goal, min_cell_cost, Q);
			expandCell(c, 0, +1, 1.0, visited_map_, mx_goal, my_goal, min_cell_cost, Q);

			// diagonal expansion
			expandCell(c, -1, -1, SQRT2, visited_map_, mx_goal, my_goal, min_cell_cost, Q);
			expandCell(c, +1, -1, SQRT2, visited_map_, mx_goal, my_goal, min_cell_cost, Q);
			expandCell(c, -1, +1, SQRT2, visited_map_, mx_goal, my_goal, min_cell_cost, Q);
			expandCell(c, +1, +1, SQRT2, visited_map_, mx_goal, my_goal, min_cell_cost, Q);
		}
	}

    // Path to goal found
	if (goal_cell) {
		while(goal_cell) {
			plan_xs.push_back(goal_cell->x_);
			plan_ys.push_back(goal_cell->y_);
			goal_cell = goal_cell->visited_from_;
		}

		// delete remaining CellInfos in Q
		while(!Q.empty()) {
			delete Q.top();
			Q.pop();
		}
    } else if (best_heuristic) {
        if (best_cell) {
            while(best_cell) {
                plan_xs.push_back(best_cell->x_);
                plan_ys.push_back(best_cell->y_);
                best_cell = best_cell->visited_from_;
            }

            // delete remaining CellInfos in Q
            while(!Q.empty()) {
                delete Q.top();
                Q.pop();
            }
        }
    }

    // delete visited CellInfos
    for (std::list<CellInfo*>::iterator it = visited_cells.begin(); it != visited_cells.end(); ++it)
        delete *it;

    return (goal_cell);
}

void AStarPlanner::expandCell(CellInfo* c, int dx, int dy, double cost_factor, double** visited_map,
                              int x_goal, int y_goal, double min_cell_cost,
                              std::priority_queue<CellInfo*, std::vector<CellInfo*>, compareCellInfos>& Q)
{

	int x = c->x_ + dx;
	int y = c->y_ + dy;

	double g_child = c->g_ + getCost(x, y) * cost_factor;
	if (g_child < visited_map[x][y]) {
		CellInfo* c_child = new CellInfo(x, y, g_child, calculateHeuristicCost(x, y, x_goal, y_goal, min_cell_cost));
		c_child->visited_from_ = c;
		Q.push(c_child);
		visited_map_[x][y] = g_child;
	}
}

double AStarPlanner::calculateHeuristicCost(int x, int y, int x_goal, int y_goal, double min_cell_cost) {
	double dx = (double)(x_goal - x);
	double dy = (double)(y_goal - y);
	return sqrt(dx * dx + dy * dy) * min_cell_cost;
}

void AStarPlanner::deleteMap() {
	for(unsigned int i = 0; i < width_; ++i) {
		delete[] visited_map_[i];
	}

	delete[] visited_map_;
    visited_map_ = nullptr;
}

}

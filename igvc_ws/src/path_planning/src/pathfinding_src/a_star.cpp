/*
Description: This is a script that implements the A* pathfinding algorithm 
Last Modified: 13 Feb 2019
Author: Isaac Draper
*/

#include<unordered_map>
#include<iostream>
#include<vector>
#include<limits>

#include "heuristics.cpp"
#include "structs.cpp"

static std::vector<pos*>* run_a_star(std::vector<pos*>* nodes, std::unordered_map<pos*, std::vector<double>*>* edges, pos* start, pos* goal, double h_func(const pos*,const pos*)) {

	std::unordered_map<pos*,double> f_cost;
	std::unordered_map<pos*,double> g_cost;
	std::unordered_map<pos*,double> h_cost;
	std::unordered_map<pos*,double> closedList;
	std::unordered_map<pos*,pos*> parents;
	for (auto p : *nodes) {
		f_cost.insert({p,std::numeric_limits<double>::max()});
		g_cost.insert({p,std::numeric_limits<double>::max()});
		h_cost.insert({p,std::numeric_limits<double>::max()});
		closedList.insert({p,false});
		parents.insert({p,NULL});
	}

	f_cost.at(start) = 0;
	g_cost.at(start) = 0;
	h_cost.at(start) = 0;
	parents.at(start) = start;

	std::unordered_map<pos*,double> openList;

	openList.insert({start,0});

	bool reached = false;
	pos* curr = NULL;
	double fNew, gNew, hNew;
	while (openList.size() > 0) {
		curr = (*(openList.begin())).first;
		openList.erase(curr);

		closedList.at(curr) = true;

		int i = 0;
		for (int w : *(edges->at(curr))) {
			if (w == 0) {
				i++;
				continue;
			}
			
			if (nodes->at(i) == goal) {
				parents.at(nodes->at(i)) = curr;
				reached = true;
				break;
			}
			else if (!closedList.at(nodes->at(i))) {
				gNew = g_cost.at(curr) + h_func(curr, nodes->at(i));
				hNew = h_func(goal, nodes->at(i));
				fNew = gNew + hNew;

				if (f_cost.at(nodes->at(i)) == std::numeric_limits<double>::max() || f_cost.at(nodes->at(i)) > fNew) {
					openList.insert({nodes->at(i),fNew});

					f_cost.at(nodes->at(i)) = fNew;
					h_cost.at(nodes->at(i)) = hNew;
					g_cost.at(nodes->at(i)) = gNew;
					parents.at(nodes->at(i)) = curr;
				}
			}
			i++;
		}
		if (reached) break;
	}

	std::vector<pos*>* path = new std::vector<pos*>();
	curr = goal;
	while (parents.at(curr) != curr) {
		path->push_back(curr);
		curr = parents.at(curr);
	}
	path->push_back(start);

	for (unsigned int i = 0; i < path->size()/2; i++) {
		pos* tmp = path->at(i);
		path->at(i) = path->at(path->size()-i-1);
		path->at(path->size()-i-1) = tmp;
	}

	return path;
}


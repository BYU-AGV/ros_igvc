/*
Description: This is a script that implements the breadth first pathfinding algorithm 
Last Modified: 13 Feb 2019
Author: Isaac Draper
*/

#include<unordered_map>
#include<vector>
#include<queue>
#include<limits>

#include "heuristics.cpp"
#include "structs.cpp"

static std::vector<pos*>* run_bfs(std::vector<pos*>* nodes, std::unordered_map<pos*, std::vector<double>*>* edges, pos* start, pos* goal, double h_func(const pos*,const pos*)) {
	std::queue<pos*> queue;
	queue.push(start);

	std::unordered_map<pos*,bool> visited;
	std::unordered_map<pos*,double> dist;
	std::unordered_map<pos*,pos*> pred;
	for (auto p : *nodes) {
		visited.insert({p,false});
		dist.insert({p,std::numeric_limits<double>::max()});
		pred.insert({p,NULL});
	}

	visited.at(start) = true;
	dist.at(start) = 0;

	bool reached = false;
	pos* curr = NULL;
	while (queue.size() > 0) {
		curr = queue.front();
		queue.pop();

		int i = 0;
		for (int w : *(edges->at(curr))) {
			if (w == 0) {
				i++;
				continue;
			}

			if (!visited.at(nodes->at(i))) {
				if (curr == goal) {
					reached = true;
					break;
				}

				visited.at(nodes->at(i)) = true;
				dist.at(nodes->at(i)) = dist.at(curr) + h_func(curr, nodes->at(i));
				pred.at(nodes->at(i)) = curr;
				queue.push(nodes->at(i));
			}

			i++;
		}
		if (reached) break;
	}


	std::vector<pos*>* path = new std::vector<pos*>();
	curr = goal;
	while (pred.at(curr) != NULL) {
		path->push_back(curr);
		curr = pred.at(curr);
	}
	path->push_back(start);

	for (unsigned int i = 0; i < path->size()/2; i++) {
		pos* tmp = path->at(i);
		path->at(i) = path->at(path->size()-i-1);
		path->at(path->size()-i-1) = tmp;
	}

	return path;
}


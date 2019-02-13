/*
Description: This is a script that implements the breadth first pathfinding algorithm 
Last Modified: 13 Feb 2019
Author: Isaac Draper
*/

#include<unordered_map>
#include<iostream>
#include<vector>
#include<queue>

#include "structs.cpp"

static void run_bfs(std::vector<pos*>* nodes, std::unordered_map<pos*, std::vector<double>*>* edges, pos* start, pos* goal) {
	std::queue<pos*> queue;
	queue.push(start);

	std::unordered_map<pos*,bool> visited;
	for (auto p : *nodes) visited.insert({p,false});
	visited.at(start) = true;

	/*
	for (auto p : *nodes) {
		print_pos(p);
		std::cout << "\t";
		for (auto i : *(edges->at(p)))
			std::cout << i << " ";
		std::cout << std::endl;
	}
	*/

	pos* curr = start;
	while (queue.size() > 0 && !equal_pos(goal, curr)) {
		curr = queue.front();
		queue.pop();

		std::cout << "Curr: ";
		print_pos(curr);
		std::cout << std::endl;

		int i = 0;
		for (int w : *(edges->at(curr))) {
			if (w == 0 || equal_pos(nodes->at(i), curr) || visited.at(nodes->at(i))) {
				i++;
				continue;
			}

			visited.at(nodes->at(i)) = true;
			queue.push(nodes->at(i));

			i++;
		}
	}

	return;
}


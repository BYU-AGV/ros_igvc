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

	bool* visited = new bool[nodes->size()];
	visited[0] = true;

	for (auto p : *nodes) {
		print_pos(p);
		std::cout << "\t";
		for (auto i : *(edges->at(p)))
			std::cout << i << " ";
		std::cout << std::endl;
	}
	
	pos* curr = NULL;
	while (queue.size() > 0 && !equal_pos(goal, curr)) {
		curr = queue.front();
		queue.pop();

	}

	return;
}


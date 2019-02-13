/*
Description: This is a common file to contain heuristic functions 
Last Modified: 13 Feb 2019
Author: Isaac Draper
*/

#include<cmath>

#include "structs.cpp"

#ifndef HEURISTICS_CPP
#define HEURISTICS_CPP

static double euclid_dist(const pos* p1, const pos* p2) {
	return std::sqrt(std::pow(p2->x - p1->x, 2) + std::pow(p2->y - p1->y, 2));
}

static double man_dist(const pos* p1, const pos* p2) {
	return std::abs(p2->x - p1->x) + std::abs(p2->y - p1->y);
}

#endif


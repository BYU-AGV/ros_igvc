/*
Description: This is a common file to contain heuristic functions 
Last Modified: 12 Mar 2019
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

static double gps_dist(const pos* p1, const pos* p2) {
	double R = 6378137;
	double PI = 3.1415926;
	
	double d_lat = (p2->x - p1->x) * PI/180;
	double d_lon = (p2->y - p1->y) * PI/180;

	double lat1 = (p1->x) * PI/180;
	double lat2 = (p2->x) * PI/180;

	double a = std::sin(d_lat/2) * std::sin(d_lat/2) +
		   std::sin(d_lon/2) * std::sin(d_lon/2) *
		   std::cos(lat1) * std::cos(lat2);
	return R * 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
}

#endif


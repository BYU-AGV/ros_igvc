/*
Description: This is a common file to contain structs
Last Modified: 13 Feb 2019
Author: Isaac Draper
*/

#include<iostream>

#ifndef STRUCTS_CPP
#define STRUCTS_CPP

struct pos {
	double x;
	double y;
};

bool equal_pos(const pos* p1, const pos* p2) {
	if (p1 == NULL || p2 == NULL) return false;
	if (p1->x != p2->x) return false;
	if (p1->y != p2->y) return false;
	return true;
}

void print_pos(const pos* p) {
	std::cout << "(" << p->x << "," << p->y << ")";
}

#endif


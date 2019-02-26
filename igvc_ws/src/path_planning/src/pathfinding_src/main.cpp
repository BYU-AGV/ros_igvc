/*
Description: This is the main entry point between python and c++
Last Modified: 25 Feb 2019
Author: Isaac Draper
*/

#include<Python.h>

#include<unordered_map>
#include<iostream>
#include<sstream>
#include<string>
#include<vector>

#include "structs.cpp"
#include "breadth_first_search.cpp"
#include "a_star.cpp"

// Reference of all functions to be exported
static PyObject* runAlgorithm(PyObject* self, PyObject* args, PyObject* kwargs);

// ------------------------------------------------------------------------------------
// All functions in this section are required functions for building
// ------------------------------------------------------------------------------------

static PyMethodDef Methods[] = {
	{"search",  (PyCFunction)runAlgorithm, METH_VARARGS | METH_KEYWORDS, "Run a pathing algorithm"},
	{NULL, NULL, 0, NULL}        /* Sentinel */
};

PyMODINIT_FUNC initpathfinding(void) {
	(void) Py_InitModule("pathfinding", Methods);
}

// ------------------------------------------------------------------------------------
// All functions in this section deal with converting between python and c++
// ------------------------------------------------------------------------------------

static std::vector<pos*>* parseNodes(PyObject* obj) {
	PyObject* seq;
	Py_ssize_t i, rws, cls;

	PyObject* row;
	
	seq = PySequence_Fast(obj, "expected a sequence");
	rws = PySequence_Size(obj);

	std::vector<pos*>* nodes = new std::vector<pos*>();
	// nodes->reserve((int)rws);

	if (PyList_Check(seq)) {
		for (i = 0; i < rws; i++) {
			row = PySequence_Fast(PyList_GET_ITEM(seq, i), "here in rows");
			if (PyList_Check(row)) {
				cls = PySequence_Size(row);

				if (cls != 2) {
					std::stringstream ss;
					ss << "Expected a position, got list of size " << cls << " instead";
					PyErr_SetString(PyExc_TypeError,ss.str().c_str());
					return NULL;
				}

				pos* p = new pos;
				PyObject* xVal = PyList_GET_ITEM(row, 0);
				PyObject* yVal = PyList_GET_ITEM(row, 1);

				double x = PyFloat_AsDouble(xVal);
				double y = PyFloat_AsDouble(yVal);
				p->x = x;
				p->y = y;
				// p->x = PyFloat_AsDouble(xVal);
				// p->y = PyFloat_AsDouble(yVal);

				nodes->push_back(p);
			}
			else {
				PyErr_SetString(PyExc_TypeError,"Expected a list");
				return NULL;
			}
		}
	}
	else {
		PyErr_SetString(PyExc_TypeError,"Expected a sequence");
	}

	return nodes;
}

static std::unordered_map<pos*,std::vector<double>*>* parseEdges(std::vector<pos*>* nodes, PyObject* obj) {
	PyObject* seq;
	Py_ssize_t i, j, rws, cls;

	PyObject* row;
	PyObject* col;
	
	seq = PySequence_Fast(obj, "expected a sequence");
	rws = PySequence_Size(obj);

	std::unordered_map<pos*, std::vector<double>*>* edges = new std::unordered_map<pos*, std::vector<double>*>();

	if (PyList_Check(seq)) {
		for (i = 0; i < rws; i++) {
			row = PySequence_Fast(PyList_GET_ITEM(seq, i), "here in rows");
			if (PyList_Check(row)) {
				cls = PySequence_Size(row);

				PyObject* item = PyList_GET_ITEM(seq, i);
				col = PySequence_Fast(item, "here in rows");

				std::vector<double>* node = new std::vector<double>();

				if (PyList_Check(col)) {
					for (j = 0; j < cls; j++) {
						PyObject* val = PyList_GET_ITEM(col, j);
						node->push_back(PyFloat_AsDouble(val));
					}
				}


				edges->insert({nodes->at(i), node});
			}
			else {
				PyErr_SetString(PyExc_TypeError,"Expected a list");
				return NULL;
			}
		}
	}
	else {
		PyErr_SetString(PyExc_TypeError,"Expected a sequence");
	}

	return edges;
}

/* DEPRECATED - was only used for inital testing
static PyObject* nodesToObject(std::vector<pos*>* nodes) {
	PyObject* obj;
	PyObject* row;

	obj = PyList_New(nodes->size());
	for (unsigned int i = 0; i < nodes->size(); i++) {
		row = PyList_New(2);
		PyList_SetItem(row, 0, PyFloat_FromDouble(nodes->at(i)->x));
		PyList_SetItem(row, 1, PyFloat_FromDouble(nodes->at(i)->y));
		PyList_SetItem(obj, i, row);
	}

	return obj;
}
*/

static pos* parsePos(PyObject* obj) {

	if (PyList_Check(obj)) {
		pos* p = new pos;
		PyObject* xVal = PyList_GET_ITEM(obj, 0);
		PyObject* yVal = PyList_GET_ITEM(obj, 1);

		p->x = PyFloat_AsDouble(xVal);
		p->y = PyFloat_AsDouble(yVal);

		return p;
	}
	else {
		PyErr_SetString(PyExc_TypeError,"Expected a position");
	}

	return NULL;
}

static PyObject* pathToObject(std::vector<pos*>* positions) {
	PyObject* obj;
	PyObject* row;

	obj = PyList_New(positions->size());
	for (unsigned int i = 0; i < positions->size(); i++) {
		row = PyList_New(2);
		PyList_SetItem(row, 0, PyFloat_FromDouble(positions->at(i)->x));
		PyList_SetItem(row, 1, PyFloat_FromDouble(positions->at(i)->y));
		PyList_SetItem(obj, i, row);
	}

	delete positions;
	return obj;
}

void toLower(std::string& s) {
	for (std::string::iterator p = s.begin(); p != s.end(); ++p) {
		*p = tolower(*p);
	}
}

typedef double (*fp)(const pos*, const pos*);
static fp getHFunction(std::string func) {

	if (func == "euclidean distance" ||
	    func == "euclidean_distance" ||
	    func == "euclidean dist" ||
	    func == "euclidean_dist" ||
	    func == "euclid distance" ||
	    func == "euclid_distance" ||
	    func == "euclid dist" ||
	    func == "euclid_dist" ||
	    func == "euclidean" ||
	    func == "euclid") {
		return euclid_dist;
	}
	else if (func == "manhattan distance" ||
	    func == "manhattan_distance" ||
	    func == "manhattan dist" ||
	    func == "manhattan_dist" ||
	    func == "man distance" ||
	    func == "man_distance" ||
	    func == "man dist" ||
	    func == "man_dist" ||
	    func == "manhattan" ||
	    func == "man") {
		return man_dist;
	}

	return NULL;
}

static PyObject* runAlgorithm(PyObject* self, PyObject* args, PyObject* kwargs) {
	Py_ssize_t TupleSize = PyTuple_Size(args);

	static const char* kwlist[] = {"", "", "", "", "algorithm", "cost_function", NULL};
	char* method = NULL;
	char* func = NULL;

	if (!TupleSize || TupleSize < 4) {
		if(!PyErr_Occurred()) 
			PyErr_SetString(PyExc_TypeError,"You must supply four inputs.");
		return NULL;
	}

	PyObject *obj1;
	PyObject *obj2;
	PyObject *obj3;
	PyObject *obj4;

	/*
	PyObject_Print(self, stdout, 0);
	fprintf(stdout, "\n");
	PyObject_Print(args, stdout, 0);
	fprintf(stdout, "\n");
	PyObject_Print(kwargs, stdout, 0);
	fprintf(stdout, "\n");
	*/

	if (!PyArg_ParseTupleAndKeywords(args, kwargs, "OOOO|ss", const_cast<char**>(kwlist), &obj1, &obj2, &obj3, &obj4, &method, &func)) {
		PyErr_SetString(PyExc_TypeError,"Error parsing input");
		return NULL;
	}

	auto nodes = parseNodes(obj1);
	auto edges = parseEdges(nodes, obj2);
	pos* start_tmp = parsePos(obj3);
	pos* goal_tmp = parsePos(obj4);
	pos* start = NULL;
	pos* goal = NULL;

	bool start_found = false;
	bool goal_found = false;
	for (auto p : *nodes) {
		if (equal_pos(start_tmp, p)) {
			start = p;
			start_found = true;
		}
		if (equal_pos(goal_tmp, p)) {
			goal = p;
			goal_found = true;
		}

		if (start_found && goal_found)
			break;
	}

	delete start_tmp;
	delete goal_tmp;

	if (start == NULL) {
		PyErr_SetString(PyExc_TypeError,"Start position not found in nodes");
		return NULL;
	}
	if (goal == NULL) {
		PyErr_SetString(PyExc_TypeError,"Goal position not found in nodes");
		return NULL;
	}

	if (nodes == NULL || edges == NULL)
		return NULL;

	// default values
	if (method == NULL) method = (char*)"a*";
	if (func == NULL) func = (char*)"euclid";

	std::string algorithm = std::string(method);
	std::string cost_function = std::string(func);

	toLower(algorithm);
	toLower(cost_function);

	double (*h_func)(const pos*, const pos*) = getHFunction(func);
	
	if (h_func == NULL) {
		PyErr_SetString(PyExc_TypeError,(std::string("Heuristic function '") + func + std::string("' not recognized")).c_str());
		return NULL;
	}

	PyObject* rtn = NULL;
	
	if (algorithm == "breadth_first_search" ||
	    algorithm == "breadth_first" ||
	    algorithm == "breadth first search" ||
	    algorithm == "breadth first" ||
	    algorithm == "bfs" ||
	    algorithm == "bf") {
		rtn = pathToObject(run_bfs(nodes, edges, start, goal, h_func));
	}
	else if (algorithm == "a star" ||
	    algorithm == "a_star" ||
	    algorithm == "a*") {
		rtn = pathToObject(run_a_star(nodes, edges, start, goal, h_func));
	}
	else {
		PyErr_SetString(PyExc_TypeError,(std::string("Pathfinding algorithm '") + algorithm + std::string("' not recognized")).c_str());
		return NULL;
	}

	// std::cout << "Ran '" << algorithm << "' using '" << func << "':" << std::endl;

	// clean up all memory
	// TODO: NEED TO FIX ALL MEMORY LEAKS/BUGS - currently will not work (SegFault) if memory is cleared
	/*
	for (unsigned int i = 0; i < nodes->size(); i++) {
		delete edges->at(nodes->at(i));
		// delete nodes->at(i); // TODO: NEED TO FIGURE OUT MEMORY ERROR
	}
	delete edges;
	delete nodes;

	delete start;
	delete goal;
	*/

	return rtn;
}


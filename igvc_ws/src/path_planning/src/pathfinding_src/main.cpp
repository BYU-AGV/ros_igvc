/*
Description: This is the main entry point between python and c++
Last Modified: 13 Feb 2019
Author: Isaac Draper
*/

#include<Python.h>
#include<iostream>
#include<sstream>
#include<string>
#include<vector>

#include "structs.cpp"

// Reference of all functions to be exported
static PyObject* runAlgorithm(PyObject* self, PyObject* args);

// ------------------------------------------------------------------------------------
// All functions in this section are required functions for building
// ------------------------------------------------------------------------------------

static PyMethodDef Methods[] = {
	{"run",  runAlgorithm, METH_VARARGS, "Run a pathing algorithm"},
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
	nodes->reserve(rws);

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
				p->x = PyFloat_AsDouble(PyList_GET_ITEM(row, 0));
				p->y = PyFloat_AsDouble(PyList_GET_ITEM(row, 1));

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

static PyObject* runAlgorithm(PyObject* self, PyObject* args) {
	Py_ssize_t TupleSize = PyTuple_Size(args);

	if (!TupleSize || TupleSize < 2) {
		if(!PyErr_Occurred()) 
			PyErr_SetString(PyExc_TypeError,"You must supply two inputs.");
		return NULL;
	}

	PyObject *obj1;
	PyObject *obj2;

	if (!PyArg_ParseTuple(args, "OO", &obj1, &obj2)) {
		PyErr_SetString(PyExc_TypeError,"Error parsing input");
		return NULL;
	}

	auto vec = parseNodes(obj1);

	if (vec == NULL)
		return NULL;

	return nodesToObject(vec);
}



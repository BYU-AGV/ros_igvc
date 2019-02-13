/*
Description: This is the main entry point between python and c++
Last Modified: 13 Feb 2019
Author: Isaac Draper
*/

#include<Python.h>

// Reference of all functions to be exported
static PyObject* run_algorithm(PyObject* self, PyObject* args);

// ------------------------------------------------------------------------------------
// All functions in this section are required functions for building
// ------------------------------------------------------------------------------------

static PyMethodDef Methods[] = {
	{"run",  run_algorithm, METH_VARARGS, "Run a pathing algorithm"},
	{NULL, NULL, 0, NULL}        /* Sentinel */
};

PyMODINIT_FUNC initpathfinding(void) {
	(void) Py_InitModule("pathfinding", Methods);
}

// ------------------------------------------------------------------------------------
// All functions in this section deal with converting between python and c++
// ------------------------------------------------------------------------------------

static PyObject* run_algorithm(PyObject* self, PyObject* args) {
	PyErr_SetString(PyExc_TypeError,"This error will always be thrown");
	return NULL;
}



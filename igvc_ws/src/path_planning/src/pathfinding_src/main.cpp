/*
Description: This is the main entry point between python and c++
Last Modified: 13 Feb 2019
Author: Isaac Draper
*/

#include<Python.h>

static PyObject* run_algorithm(PyObject* self, PyObject* args);

static PyMethodDef Methods[] = {
    {"run",  run_algorithm, METH_VARARGS, "Run a pathing algorithm"},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

PyMODINIT_FUNC initpathfinding(void) {
    (void) Py_InitModule("pathfinding", Methods);
}



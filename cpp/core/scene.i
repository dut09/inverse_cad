%module scene
%{
#include "scene.h"
%}
%include <std_string.i>
%include <std_vector.i>

%exception {
    try {
	    $action
	} catch (const std::runtime_error& e) {
        PyErr_SetString(PyExc_RuntimeError, const_cast<char*>(e.what()));
        SWIG_fail;
    } catch (...) {
        PyErr_SetString(PyExc_RuntimeError, "Unknown error.");
        SWIG_fail;
    }
}

namespace std {
    %template(VecInt) vector<int>;
    %template(VecVecInt) vector<vector<int>>;
};

%include "config.h"
%include "python_struct.h"
%include "scene.h"

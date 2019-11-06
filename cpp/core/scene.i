%module scene
%{
#include "scene.h"
%}
%include <std_string.i>
%include <std_vector.i>

namespace std {
    %template(VecInt) vector<int>;
    %template(VecVecInt) vector<vector<int>>;
};

%include "config.h"
%include "python_struct.h"
%include "scene.h"

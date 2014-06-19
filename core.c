/*
 * The core calculation of distortion in projection, written in C 
 * Library for python available at https://github.com/MacroBull/lib-python-macrobull/tree/master/macrobull/projCorrection
 * @author: macrobull
 * @date: June 2014
 * 
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define MAXSIZE (640*480 * 2)
#define _abs(x) (x>=0 ? x : -x)

#define dprint if (debug) printf
#define _assert(x,s) if (x) {printf(s);exit(0);}


typedef int32_t coord; // coordinate is int(or float)

int32_t res_x[MAXSIZE], res_y[MAXSIZE]; // Static storage of remapping result

////////////////// Using global vars for better performance/////////
coord cp13;
double sp13;


coord denom, numx, numy;
coord xs, ys, xt, yt;
coord cp0, cp1, cp2, cp3;
double angs, angt;

coord xp, yp, xtl, ytl, xbr, ybr;
coord xq, yq;
double sq, tq;

//// Calculate the angle between given two vectors
double crossAngle(coord vx1, coord vy1, coord vx2, coord vy2){
	
// 	_assert(((vx1 == 0 )&&( vy1 == 0))||((vx2 == 0 )&&( vy2 == 0)), "Error: Zero vector!");
	
	cp13 = vx1 * vy2 - vx2 * vy1;
	sp13 = (vx1 * vx1 + vy1 * vy1);
	sp13 *= (vx2 * vx2 + vy2 * vy2);
	sp13 = sqrt(sp13);

	return asin(cp13/sp13);
}

/// Main model building function 
void do_build(coord tar_x, coord tar_y,
	coord x0, coord y0, coord x1, coord y1, 
	coord x2, coord y2, coord x3, coord y3, 
	int32_t debug){

	dprint("ProjCorrection: Debug on\n");
	dprint("Resolution: %dx%d\n", tar_x, tar_y);
// 	dprint("CrossAngle test:%f\n", crossAngle(1,0,1,1));
	
	/* 
	 * Step1: calculate two FAR POINT s and t
	 * denom = 0 means the opposite sizes are parallel,
	 * TODO: SHOULD SKIP, but no worry ~~~~~~~
	 * 
	 */
	
	denom = (y1 - y0) * (x3 - x2) - (y2 - y3) * (x0 - x1);
	if (denom){
		cp1 = x2*y3 - x3*y2;
		cp2 = x1*y0 - x0*y1;
		numx = cp1 * (x0 - x1) - cp2 * (x3 - x2);
		numy = cp1 * (y1 - y0) - cp2 * (y2 - y3);
		xs = numx / denom;
		ys = numy / -denom;
	}
	
	denom = (y3 - y0) * (x1 - x2) - (y2 - y1) * (x0 - x3);
	if (denom){
		cp1 = x2*y1 - x1*y2;
		cp2 = x3*y0 - x0*y3;
		numx = cp1 * (x0 - x3) - cp2 * (x1 - x2);
		numy = cp1 * (y3 - y0) - cp2 * (y2 - y1);
		xt = numx / denom;
		yt = numy / -denom;
	}
	
	/* 
	 * Step2: calculate two angles from s/t to the opposite sides
	 * (Result 0 means parallel)
	 */
	
	angs = crossAngle(x1 - x0, y1 - y0, x2 - x3, y2 - y3);
	angt = crossAngle(x3 - x0, y3 - y0, x2 - x1, y2 - y1);
	
	dprint("Modelling: as = %f, at = %f, s = (%d,%d), t = (%d,%d)\n", angs, angt, xs, ys, xt, yt);
	
	//assert((angs == 0) || (angt ==0));
	
	/*
	 * Step3: find the region in the rectangle, which is about to be remapped
	 * Simplified approximately to TopLeft + RightBottom
	 */
	
	xtl = x0>x1 ? x1 : x0;
	xtl = xtl>x2 ? x2 : xtl;
	xtl = xtl>x3 ? x3 : xtl;
	
	ytl = y0>y1 ? y1 : y0;
	ytl = ytl>y2 ? y2 : ytl;
	ytl = ytl>y3 ? y3 : ytl;
	
	xbr = x0<x1 ? x1 : x0;
	xbr = xbr<x2 ? x2 : xbr;
	xbr = xbr<x3 ? x3 : xbr;
	
	ybr = y0<y1 ? y1 : y0;
	ybr = ybr<y2 ? y2 : ybr;
	ybr = ybr<y3 ? y3 : ybr;
	
	
	dprint("Rigion: (%d,%d) - (%d,%d)\n", xtl, ytl, xbr, ybr);
	
	/* Step4: give the coresponding result list
	 *  q(x, y)  ->  q(s, t)
	 */
	
	for (xp = xtl + 1; xp < xbr; xp++) 
	for (yp = ytl + 1; yp < ybr; yp++) {
		if (((xp == xs) && ( yp == ys))||((xp == xt) && ( yp == yt))) continue; // DANGER! SKIP!
		
// 		_assert((x1 == x0) && ( y1 == y0), "AHAHAHAH");
// 		_assert((x3 == x0) && ( y3 == y0), "AHAHAHAH");
		_assert((xp == xs) && ( yp == ys), "GAGAGAGA");
		_assert((xp == xt) && ( yp == yt), "GAGAGAGA");
		
		sq = crossAngle(x1 - x0, y1 - y0, xp - xs, yp - ys) / angs;
		tq = crossAngle(x3 - x0, y3 - y0, xp - xt, yp - yt) / angt;
		
		xq = (coord)(_abs(sq) * tar_x); // Direction is uncertain 
		yq = (coord)(_abs(tq) * tar_y);
		
// 		dprint("(%d,%d) -> (%d,%d) | (%f,%f)\n", xp, yp, xq, yq, sq, tq);
		// in Quad
		cp0 = (x0 - xp) * (y1 - yp) - (x1 - xp) * (y0 - yp);
		cp1 = (x1 - xp) * (y2 - yp) - (x2 - xp) * (y1 - yp);
		cp2 = (x2 - xp) * (y3 - yp) - (x3 - xp) * (y2 - yp);
		cp3 = (x3 - xp) * (y0 - yp) - (x0 - xp) * (y3 - yp);
		if ( (xq>=0)&&(xq<=tar_x) && (yq>=0)&&(yq<=tar_y)  && 
			(((cp0>0) && (cp1>0) && (cp2>0) && (cp3>0)) || ((cp0<0) && (cp1<0) && (cp2<0) && (cp3<0)))){ // a pixel in region
// 			dprint("(%d,%d) is inside\n", xp, yp);
			res_x[yq * tar_x + xq] = xp;
			res_y[yq * tar_x + xq] = yp;
		}
	}
	dprint("Correction done\n");
	
}

/////////////Python warpper//////////////////////
#include <Python.h>
static PyObject* hello(PyObject* self, PyObject* args)
{
        return Py_BuildValue("sO", "Hello, correction!", args);
}

static PyObject* build(PyObject* self, PyObject* args)
{
        coord tar_x, tar_y, x0, y0, x1, y1, x2, y2, x3, y3;
		int32_t debug;
        if(! PyArg_ParseTuple(args, "iiiiiiiiiii", &tar_x, &tar_y, &x0, &y0, &x1, &y1, &x2, &y2, &x3, &y3, &debug))
                return NULL;
        
		do_build(tar_x, tar_y, x0, y0, x1, y1, x2, y2, x3, y3, debug);
		
		int32_t i, len = tar_x * tar_y;
		PyObject *list_x, *list_y;
		list_x = PyList_New(len);
		list_y = PyList_New(len);
		if (list_x != NULL) for (i=0; i<len; i++) PyList_SET_ITEM(list_x, i, PyInt_FromLong(res_x[i]));
		if (list_y != NULL) for (i=0; i<len; i++) PyList_SET_ITEM(list_y, i, PyInt_FromLong(res_y[i]));
		
        return Py_BuildValue("OO", list_x, list_y);
}


static PyObject* build2(PyObject* self, PyObject* args) { // Return result in-place without memory leaks
		coord tar_x, tar_y, x0, y0, x1, y1, x2, y2, x3, y3;
		PyObject *list_x, *list_y;
		int32_t debug;
        if(! PyArg_ParseTuple(args, "OOiiiiiiiiiii", &list_x, &list_y, &tar_x, &tar_y, &x0, &y0, &x1, &y1, &x2, &y2, &x3, &y3, &debug))
                return NULL;
        
		do_build(tar_x, tar_y, x0, y0, x1, y1, x2, y2, x3, y3, debug);

		int32_t i, len = tar_x * tar_y;
		for (i=0; i<len; i++) PyList_SetItem(list_x, i, PyInt_FromLong(res_x[i]));
		for (i=0; i<len; i++) PyList_SetItem(list_y, i, PyInt_FromLong(res_y[i]));
		
// 		return Py_BuildValue("i",0);
		return Py_BuildValue("OO", list_x, list_y);
}


static PyObject* redist(PyObject* self, PyObject* args) {
}

////////////API///////////////////
static PyMethodDef Methods[]=
{
        {"hello", hello, METH_VARARGS, "For test..."},
        {"build", build, METH_VARARGS, "Do calculation."},
        {"build2", build2, METH_VARARGS, "Do calculation."},
        {"redist", redist, METH_VARARGS, "Array transfer."},
        {NULL, NULL}
};

////////////init///////////////////
PyMODINIT_FUNC initcore(void)
{
        Py_InitModule3("core", Methods, "Projection correct");
}
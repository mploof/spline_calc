// spline_calc.h

#ifndef _SPLINE_CALC_h
#define _SPLINE_CALC_h
#include "matrix_math.h"

class Spline
{
	public:
		Spline();
		void init();
		void setInterpPts(int* p_points, int p_count);
		void getLoc(int p_t, int p_point, float** p_output);
		void getVel(int p_t);
		void getAccel(int p_t);

		void printInterpPts();
		void printCtrlPts();

	private:
		matrix m_interp_pts;
		matrix m_ctrl_pts;
		void solveControlPnts();
		int whichCurve();
		void getLoc(int p_t, matrix p_points, int p_point, float** p_output);

};
#endif


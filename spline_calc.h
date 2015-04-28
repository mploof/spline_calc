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
		void getCurvePts(int p_point_count);
		void getVel(int p_t);
		void getAccel(int p_t);

		void printInterpPts();
		void printCtrlPts();
		void printCurvePts();

	private:
		matrix		m_interp_pts;
		matrix		m_b_ctrl_pts;
		matrix*		m_bez_ctrl_pts;
		floatMatrix m_curve_pts;
		int			m_bez_count;
		void		solveBsplineCtlPnts();
		void		solveBezCtrlPts();
		int			whichCurve();
};
#endif


// spline_calc.h

#ifndef _SPLINE_CALC_h
#define _SPLINE_CALC_h
#include "matrix_math.h"

class Spline
{
	public:
		Spline();
		void init();
		void setInterpPts(float* p_points);
		void getCurvePts(int p_point_count);
		void getVel(float p_t);
		void getAccel(float p_t);

		void printInterpPts(bool p_monitor);
		void printCtrlPts(bool p_monitor);
		void printCurvePts(bool p_monitor);

	private:
		matrix		m_interp_pts;
		matrix		m_b_ctrl_pts;
		matrix*		m_bez_ctrl_pts;
		matrix		m_curve_pts;
		int			m_bez_count;
		void		solveBsplineCtlPnts();
		void		solveBezCtrlPts();
};
#endif


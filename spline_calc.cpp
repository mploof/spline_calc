// 
// 
// http://www.math.ucla.edu/~baker/149.1.02w/handouts/dd_splines.pdf

#include "Arduino.h"
#include "matrix_math.h"
#include "spline_calc.h"


void Spline::init()
{


}

Spline::Spline() {

}


void Spline::setInterpPts(int* p_points, int p_count){

	// Initialize the interpolation points matrix
	m_interp_pts.init(p_count, 2);
	
	// Set the 1D array as the new interpolation point matrix
	m_interp_pts.setValues(p_points, p_count * 2);

	// Solve for the Bezier control points
	solveControlPnts();
}

// Find the XY coordinate for Bezier spline at parameter p_t. Uses spline object's own control points. Public function.
void Spline::getLoc(float p_t, matrix p_output){
	getLoc(p_t, m_ctrl_pts);
}

// Find the XY coordinate for Bezier spline at parameter p_t given control point p_points. p_points must be a float matrix. Private function.
void Spline::getLoc(float p_t, matrix p_points, matrix p_output){

	const int COLUMNS = 2;	// Number of values to represent an XY point

	// Determine how many curves comprise the spline
	int curve_count = m_ctrl_pts.rowCount() - 1;
	
	// Exceute de Casteljau's Algorithm
	while (curve_count > 0){
		for (byte r = 0; r < curve_count; r++){
			for (byte c = 0; c < COLUMNS; c++){
				float temp = (1.0 - p_t) * p_points.getValue(r, c) + p_t * p_points.getValue(r + 1, c);
				p_points.setValue(r, c, temp);
			}
		}
		curve_count--;
	}

	p_output.initFloat(1, 2);
	p_output.setValue(0, 0, p_points.getValue(0, 0));
	p_output.setValue(0, 1, p_points.getValue(0, 1));

}

void Spline::getVel(int p_t){

}

void Spline::getAccel(int p_t){

}

void Spline::solveControlPnts(){

	// Create a 141 matrix with order interpolation point count - 2
	matrix diag141(m_interp_pts.rowCount() - 2, m_interp_pts.rowCount() - 2);
	diag141.set141();
	
	// Find inverse of 141 matrix
	matrix inv_diag141;
	matrix::inverse(diag141, inv_diag141);

	// Create a matrix to hold the constants for solving the control points
	matrix constants(m_interp_pts.rowCount() - 2, 2);

	
	// Set the constant values based upon interpolation points
	for (byte r = 0; r < constants.rowCount(); r++){
		for (byte c = 0; c < constants.colCount(); c++){
			// First row
			if (r == 0){
				constants.setValue(r, c, 6 * m_interp_pts.getValue(r + 1, c) - m_interp_pts.getValue(r, c));
			}
			// Last row
			else if (r == constants.rowCount() - 1){
				constants.setValue(r, c, 6 * m_interp_pts.getValue(r + 1, c) - m_interp_pts.getValue(r + 2, c));
			}
			// All other rows
			else
				constants.setValue(r, c, 6 * m_interp_pts.getValue(r + 1, c));
		}
	}

	// Solve for the control points
	matrix temp_ctrl_pts;
	matrix::mult(inv_diag141, constants, temp_ctrl_pts);

	// Save to control points float matrix, including first and last interpolation points
	m_ctrl_pts.initFloat(m_interp_pts.rowCount(), 2);
	for (byte r = 0; r < m_ctrl_pts.rowCount(); r++){
		for (byte c = 0; c < m_ctrl_pts.colCount(); c++){
			if (r == 0 || r == m_ctrl_pts.rowCount() - 1)
				m_ctrl_pts.setValue(r, c, m_interp_pts.getValue(r, c));
			else
				m_ctrl_pts.setValue(r, c, temp_ctrl_pts.getValue(r - 1, c));
		}
	}
}

int Spline::whichCurve(){

}

void Spline::printInterpPts(){
	m_interp_pts.print("Interpolation points");
}

void Spline::printCtrlPts(){
	m_ctrl_pts.print("Control Points");
}
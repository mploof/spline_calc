// 
// 
// http://www.math.ucla.edu/~baker/149.1.02w/handouts/dd_splines.pdf

#include "Arduino.h"
#include "matrix_math.h"
#include "spline_calc.h"


void Spline::init()
{
	//m_bez_ctrl_pts = NULL;

}

Spline::Spline() {
	
}


	}
}

void Spline::setInterpPts(float* p_points, int p_count){	

	//Com.print("Ponts counted: ");
	//Com.println (count);
	if (p_count == 3) {
		//Com.println("Can't use 3 points. Don't know why, but it doesn't work.");
		return;
	}
	
	// Initialize the interpolation points matrix
	m_interp_pts.init(p_count, 2);
		
	// Set the 1D array as the new interpolation point matrix
	m_interp_pts.setValues(p_points, p_count * 2);
	
	// Solve for the B-spline control points
	solveBsplineCtlPnts();
	
	// Solve for the Bezier control points
	solveBezCtrlPts();
	
	
}

void Spline::solveBezCtrlPts(){

	const int XY_COL = 2;			// Number of values to represent an XY point
	const int BEZ_PNT_CNT = 4;		// Number of control points required to define a cubic Bezier curve

	// Determine how many curves comprise the spline
	m_bez_count = m_b_ctrl_pts.rowCount() - 1;

	//// If memory for the bezier points has already been allocated, free it before reallocating
	//if (m_bez_ctrl_pts != NULL)
	//	free(m_bez_ctrl_pts);

	//// Allocate memory for Bezier control point matrix and initialize matricies
	//m_bez_ctrl_pts = (matrix *)malloc(m_bez_count * sizeof(matrix));
	for (byte i = 0; i < m_bez_count; i++){
		m_bez_ctrl_pts[i].init(BEZ_PNT_CNT, XY_COL);
	}
	
	// Populate the Bezier control points
	for (byte i = 0; i < m_bez_count; i++){
		for (byte r = 0; r < BEZ_PNT_CNT; r++){
			for (byte c = 0; c < XY_COL; c++){
				switch (r)
				{
					case 0:
					{		
						m_bez_ctrl_pts[i].setValue(r, c, m_interp_pts.getValue(i, c));
						break;
					}
					case 1:
					{
						float pnt = (float)m_b_ctrl_pts.getValue(i, c) * 2 / 3 + (float)m_b_ctrl_pts.getValue(i + 1, c) * 1 / 3;
						m_bez_ctrl_pts[i].setValue(r, c, pnt);
						break;
					}
					case 2:
					{
						float pnt = (float)m_b_ctrl_pts.getValue(i, c) * 1 / 3 + (float)m_b_ctrl_pts.getValue(i + 1, c) * 2 / 3;
						m_bez_ctrl_pts[i].setValue(r, c, pnt);
						break;
					}
					case 3:
					{
						m_bez_ctrl_pts[i].setValue(r, c, m_interp_pts.getValue(i + 1, c));
						break;
					}
				}
			}
		}
	}
}

// Find the XY coordinate for Bezier spline at parameter p_t
void Spline::calcCurvePts(int p_point_count){
	
	//Com.println("Getting curve points");
	const int XY_COL = 2;			// Number of values to represent an XY point

	// If the output matrix isn't the right size, reinitialize it
	if (m_curve_pts.rowCount() != p_point_count || m_curve_pts.colCount() != XY_COL)
		m_curve_pts.init(p_point_count, XY_COL);

	float increment = (float)m_bez_count / (float)(p_point_count - 1);	// Amount by which t should increment
	int which = 0;														// Current curve being operated upon
	float t = 0;														// Current t value

	for (float r = 0; r < p_point_count; r++){
		
		// If this is the last point, force t to be the 
		// end of the curve so we get an accurate end point
		if (r == p_point_count - 1)
			t = 1;

		// Compute the XY location for the current t parameter value
		for (byte c = 0; c < XY_COL; c++){
			float loc = pow((1 - t), 3) * m_bez_ctrl_pts[which].getValue(0, c) + 3 * pow((1 - t), 2) * t * m_bez_ctrl_pts[which].getValue(1, c) +
				3 * (1 - t) * pow(t, 2) * m_bez_ctrl_pts[which].getValue(2, c) + pow(t, 3) * m_bez_ctrl_pts[which].getValue(3, c);
			m_curve_pts.setValue(r, c, loc);
		}

		// Move to the next t
		t += increment;

		// If the new t is greater than 1, move to the next curve and reduce t to keep it between 0 and 1
		if (t >= 1){			
			which++;
			t -= 1;			
		}
	}
}

void Spline::getAccel(float p_t){

}

// Finds B-spline control points
void Spline::solveBsplineCtlPnts(){
	
	float inv_1_row = 0.25;

	// Create a 141 matrix with order interpolation point count - 2
	matrix diag141(m_interp_pts.rowCount() - 2, m_interp_pts.rowCount() - 2);	
	diag141.set141();
	
	// Find inverse of 141 matrix
	matrix inv_diag141;
	if (diag141.rowCount() > 1){
		matrix::inverse(diag141, inv_diag141);
	}

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
	if (diag141.rowCount() > 1)
		matrix::mult(inv_diag141, constants, temp_ctrl_pts);
	else
		matrix::mult(inv_1_row, constants, temp_ctrl_pts);
	
	// Save to control points matrix, including first and last interpolation points
	m_b_ctrl_pts.init(m_interp_pts.rowCount(), 2);
	
	for (byte r = 0; r < m_b_ctrl_pts.rowCount(); r++){
		for (byte c = 0; c < m_b_ctrl_pts.colCount(); c++){
			if (r == 0 || r == m_b_ctrl_pts.rowCount() - 1)
				m_b_ctrl_pts.setValue(r, c, m_interp_pts.getValue(r, c));
			else
				m_b_ctrl_pts.setValue(r, c, temp_ctrl_pts.getValue(r - 1, c));
		}
	}	
}

void Spline::printInterpPts(bool p_monitor){
	if (p_monitor)
		m_interp_pts.print("Interpolation points");
	else
		m_interp_pts.print();
}

void Spline::printCtrlPts(bool p_monitor){
	if (p_monitor)
		m_b_ctrl_pts.print("Control points");
	else
		m_b_ctrl_pts.print();
}

void Spline::printCurvePts(bool p_monitor){
	if (p_monitor)
		m_curve_pts.print("Curve points");
	else
		m_curve_pts.print();
}

float Spline::getCurvePntVal(int p_row, int p_col){

	if (p_row > m_curve_pts.rowCount() || p_row < 0 
		|| p_col > m_curve_pts.colCount() || p_col < 0)
		return;

	return m_curve_pts.getValue(p_row, p_col);
}
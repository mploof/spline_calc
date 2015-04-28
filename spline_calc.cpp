// 
// 
// http://www.math.ucla.edu/~baker/149.1.02w/handouts/dd_splines.pdf

#include "Arduino.h"
#include "matrix_math.h"
#include "spline_calc.h"


void Spline::init()
{
	m_bez_ctrl_pts = NULL;

}

Spline::Spline() {

}


void Spline::setInterpPts(int* p_points, int p_count){

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

	// If memory for the bezier points has already been allocated, free it before reallocating
	if (m_bez_ctrl_pts != NULL)
		delete[] m_bez_ctrl_pts;

	// Allocate memory for Bezier control point matrix and initialize matricies
	m_bez_ctrl_pts = new matrix[m_bez_count];
	for (byte i = 0; i < m_bez_count; i++){
		m_bez_ctrl_pts[i].init(BEZ_PNT_CNT, XY_COL);
	}

	//m_b_ctrl_pts.print("B Spline Ctrl Points", false);

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
						int pnt = (float)m_b_ctrl_pts.getValue(i, c) * 2 / 3 + (float)m_b_ctrl_pts.getValue(i + 1, c) * 1 / 3;
						m_bez_ctrl_pts[i].setValue(r, c, pnt);
						break;
					}
					case 2:
					{
						int pnt = (float)m_b_ctrl_pts.getValue(i, c) * 1 / 3 + (float)m_b_ctrl_pts.getValue(i + 1, c) * 2 / 3;
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
		//m_bez_ctrl_pts[i].print("Bez ctrl", false);
	}
}

// Find the XY coordinate for Bezier spline at parameter p_t
void Spline::getCurvePts(int p_point_count){
	
	//Serial.println("Getting curve points");

	const int XY_COL = 2;											// Number of values to represent an XY point
	const int BEZ_PNT_CNT = 4;										// Number of control points required to define a cubic Bezier curve

	// If the output matrix isn't the right size, reinitialize it
	if (m_curve_pts.rowCount() != p_point_count || m_curve_pts.colCount() != XY_COL)
		m_curve_pts.init(p_point_count, XY_COL);

	float increment = (float)m_bez_count / (float)p_point_count;	// Amount by which t should increment
	int which = 0;													// Current curve being operated upon
	float t = 0;													// Current t value

	for (int r = 0; r < p_point_count; r++){
		
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

void Spline::getAccel(int p_t){

}

// Finds B-spline control points
void Spline::solveBsplineCtlPnts(){

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

int Spline::whichCurve(){

}

void Spline::printInterpPts(){
	m_interp_pts.print(true);
}

void Spline::printCtrlPts(){
	m_b_ctrl_pts.print(true);
}

void Spline::printCurvePts(){
	m_curve_pts.print(true);
}
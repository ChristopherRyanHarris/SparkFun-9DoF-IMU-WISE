
/*************************************************
** FILE : Math
** This file contains all the math helper functions.
** These functions help compute common functions
** which are not defined in the standard libraries.
**************************************************/


/*******************************************************************
** Includes ********************************************************
********************************************************************/


#include "./Include/Common_Config.h"

#if EXE_MODE==1 /* Emulator Mode */
#include <math.h>
#include "./Include/Emulator_Config.h"
#endif /* End Emulator Mode */


/*******************************************************************
** Functions *******************************************************
********************************************************************/


/*************************************************
** Vector_Dot_Product
** Computes the dot product of two vectors
**   scalar = Vector_Dot_Product( v1[], v2[] )
**   scalar = <v1,v2>
*/
float Vector_Dot_Product(const float v1[3], const float v2[3])
{
  float result = 0;
  for(int c = 0; c < 3; c++) { result += v1[c] * v2[c]; }
  return result;
} /* End Vector_Dot_Product */


/*************************************************
** Vector_Cross_Product
** Computes the cross product of two vectors
**   Vector_Cross_Product( v1[], v2[], out[] )
**   out[] = v1[] x v2[]
*/
void Vector_Cross_Product( const float v1[3], const float v2[3], float out[3] )
{
  out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
  out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
  out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
} /* End Vector_Cross_Product */


/*************************************************
** Vector_Scale
** Multiply a vector by a scalar (element-wise)
**   Vector_Scale( v[], scalar, out[] )
**   out[] = v[] .* scalar
*/
void Vector_Scale( const float v[3], float scale, float out[3] )
{
  for(int c = 0; c < 3; c++) { out[c] = v[c] * scale; }
} /* End Vector_Scale */


/*************************************************
** Vector_Add
** Adds two vectors element-wise
**   Vector_Add( out[], v1[], v2[] )
**   out[] = v1[] + v2[]
*/
void Vector_Add( const float v1[3], const float v2[3], float out[3] )
{
  for(int c = 0; c < 3; c++) { out[c] = v1[c] + v2[c]; }
} /* End Vector_Add */


/*************************************************
** Matrix_Matrix_Multiply
** Multiply two 3x3 matrices
**   Matrix_Matrix_Multiply( m1[][], m2[][], out[][] )
**   out = m1 * m2
*/
void Matrix_Matrix_Multiply(const float m1[3][3], const float m2[3][3], float out[3][3])
{
  for(int x = 0; x < 3; x++)  // rows
  {
    for(int y = 0; y < 3; y++)  // columns
    {
      out[x][y] = m1[x][0] * m2[0][y] + m1[x][1] * m2[1][y] + m1[x][2] * m2[2][y];
    }
  }
} /* End Matrix_Matrix_Multiply */


/*************************************************
** Matrix_Vector_Multiply
** Multiply 3x3 matrix with 3x1 vector
** Outputs a 3x1 vector
**   Matrix_Vector_Multiply( m[][], v[], out[] )
**   out = m * v
*/
void Matrix_Vector_Multiply(const float m[3][3], const float v[3], float out[3])
{
  for(int x = 0; x < 3; x++) { out[x] = m[x][0] * v[0] + m[x][1] * v[1] + m[x][2] * v[2]; }
} /* End Matrix_Vector_Multiply */


/*************************************************
** Rolling_Mean
** Compute the rolling mean given an inital mean
** a sample, and a sample number.
** The rolling mean is a real-time meathod of
** computing a mean.
** m = m + (x - m)/n
** Input:
**	n: Sample Number
** 	m: An Initial Mean
**	x: Sample
*/
float Rolling_Mean( const int n, float m, float x )
{
	return ( m + (x-m)/n );
} /* End Rolling_Mean */


/*************************************************
** Windowed_Mean
** Compute the approximate moving average of the
** data in a real-time method without storing
** previous samples.
** m = m*(1-alpha) + (alpha)*x;
** Input:
**	m: Initial mean
** 	x: New sample value
**  n: Sample number
**  a: Exponential factor
*/
float Windowed_Mean( float m, float x, int n, float a )
{
	/* Error checking */
	if( n==0 ){ return(0.0f); }

	/* Compute and return the moving average */
	return( m*(1-a) + x*(a) );
}

/*************************************************
** Rolling_Variance
** Compute the rolling standard deviation (xN) given
** the current mean, the previous mean, and a sample.
** The rolling std is a real-time meathod of
** computing a stadard deviation.
** S = S + (x-m) * (x-m_prev);
** Input:
**	m_prev: Previous Mean (at last sample)
**	m:			Current Mean
**	x: 			Sample
**	S: 			Curent STD
*/
float Rolling_Variance( const float m_prev, const float m, float x, float S )
{
	return ( S + (x-m)*(x-m_prev) );
} /* End Rolling_Std */


/*************************************************
** f_asin
** A faster arcsin
*/
float f_asin( float x )
{
  float negate = (float)(x < 0);
  x = fabs(x);
  float ret = -0.0187293;
  ret *= x;
  ret += 0.0742610;
  ret *= x;
  ret -= 0.2121144;
  ret *= x;
  ret += 1.5707288;
  ret = 3.14159265358979*0.5 - sqrt(1.0 - x)*ret;
  return ret - 2 * negate * ret;
} /* End f_asin */


/*************************************************
** f_atan2
** A faster arctan2
*/
float f_atan2( float y, float x )
{
  float t0, t1, t3, t4;

  t3 = fabs(x);
  t1 = fabs(y);
  t0 = fmax(t3, t1);
  t1 = fmin(t3, t1);
  t3 = (1.0/t0);
  t3 = t1 * t3;

  t4 = t3 * t3;
  t0 = (-0.013480470);
  t0 = t0 * t4 + (0.057477314);
  t0 = t0 * t4 - (0.121239071);
  t0 = t0 * t4 + (0.195635925);
  t0 = t0 * t4 - (0.332994597);
  t0 = t0 * t4 + (0.999995630);
  t3 = t0 * t3;

  t3 = (fabs(y) > fabs(x)) ? (1.570796327) - t3 : t3;
  t3 = (x < 0) ?  (3.141592654) - t3 : t3;
  t3 = (y < 0) ? -t3 : t3;

  return t3;
} /* End f_atan2 */

/*************************************************
** calc_circle_center
** compute the center of a circle given 3 points
*/
void calc_circle_center( float p1[2], float p2[2], float p3[2], float xcyc[2] )
{

	float x1, x2, x3;
	float y1, y2, y3;
	float mr, mt;
	int i;

	/* Initialize the center */
	for( i=0; i<2; i++ ){ xcyc[i]=0; }

	x1 = p1[0];
	x2 = p2[0];
	x3 = p3[0];
	y1 = p1[1];
	y2 = p2[1];
	y3 = p3[1];

	/* Chcek for 0 */
	if( x1==y1 && x1==0 ){ return; }
	if( x2==y2 && x2==0 ){ return; }
	if( x3==y3 && x3==0 ){ return; }

	/* Check for 0 denominator later */
	if( x1==x2 ){ return; }
	if( x2==x3 ){ return; }

	mr = (y2-y1)/(x2-x1);
	mt = (y3-y2)/(x3-x2);

	/* More checking */
	if( mr==mt ){ return; }
	if( mr==0 ){ return; }

	//idf1  = isinf(mr);
	//idf2  = isinf(mt);
	//idf34 = isequaln(mr,mt) | isnan(mr) | isnan(mt);
	//idmr0 = mr==0;

	/* Compute center */
	xcyc[0] = (mr*mt*(y3-y1) + mr*(x2+x3) - mt*(x1+x2)) / (2*(mr-mt));
	//xcyc[idf1]  = (mt(idf1).*(y3(idf1)-y1(idf1))+(x2(idf1)+x3(idf1)))/2;
	//xcyc(idf2)  = ((x1(idf2)+x2(idf2))-mr(idf2).*(y3(idf2)-y1(idf2)))/2;
	//xcyc(idf34) = NaN;

	xcyc[1] = -1/mr * (xcyc[0]-(x1+x2)/2) + (y1+y2)/2;
	//xcyc(2,idmr0) = -1./mt(idmr0).*(xcyc(idmr0)-(x2(idmr0)+x3(idmr0))/2)+(y2(idmr0)+y3(idmr0))/2;
	//xcyc(2,idf34) = NaN;

}










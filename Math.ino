
/*******************************************************************
** FILE:
**    Math
** DESCRIPTION:
**    This file contains all the math helper functions.
**    These functions help compute common functions
**    which are not defined in the standard libraries.
********************************************************************/


/*******************************************************************
** Includes ********************************************************
********************************************************************/

#ifndef COMMON_CONFIG_H
  #include "../Include/Common_Config.h"
#endif
#if EXE_MODE==1 /* Emulator Mode */
  /* In emulatiom mode, "Emulator_Protos" is needed to
  ** use funcitons in other files.
  ** NOTE: This header should contain the function
  **       prototypes for all execution functions */
  #include "../Include/Emulator_Protos.h"
#endif  /* End Emulator Mode */

/*******************************************************************
** Functions *******************************************************
********************************************************************/

/*************************************************
** FUNCTION: Vector_Magnitude
** VARIABLES:
**    [I ]  const float v1  :
** RETURN:
**    float result
** DESCRIPTION:
**    Computes the magnitude of a vector
**    result = (v[0]*v[0] + v[1]*v[1] + v[2]*v[2])^0.5
*/
float Vector_Magnitude ( const float v1[3] )
{
  float result = 0;
  for(int c = 0; c < 3; c++) { result += v1[c] * v1[c]; }
  result = sqrt(result);
  return result;
} /* End Vector_Magnitude */

/*************************************************
** FUNCTION: Vector_Dot_Product
** VARIABLES:
**    [I ]  const float v1  :
**    [I ]  const float v2  :
** RETURN:
**    float result
** DESCRIPTION:
**    Computes the dot product of two vectors
**    result = <v1,v2>
*/
float Vector_Dot_Product ( const float v1[3], const float v2[3] )
{
  float result = 0;
  for(int c = 0; c < 3; c++) { result += v1[c] * v2[c]; }
  return result;
} /* End Vector_Dot_Product */


/*************************************************
** FUNCTION: Vector_Cross_Product
** VARIABLES:
**    [I ]  const float v1[3]
**    [I ]  const float v2[3]
**    [IO]        float out[3]
** RETURN:
**    NONE
** DESCRIPTION:
**    Computes the cross product of two vectors
**    out[] = v1[] x v2[]
*/
void Vector_Cross_Product( const float v1[3], const float v2[3], float out[3] )
{
  out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
  out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
  out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
} /* End Vector_Cross_Product */


/*************************************************
** FUNCTION: Vector_Scale
** VARIABLES:
**    [I ]  const float v[3]
**    [I ]  const float scalar
**    [IO]        float out[3]
** RETURN:
**    NONE
** DESCRIPTION:
**    Multiply a vector by a scalar (element-wise)
**    out[] = v[] .* scalar
*/
void Vector_Scale( const float v[3], const float scalar, float out[3] )
{
  for( int i=0; i<3; i++) { out[i] = v[i] * scalar; }
} /* End Vector_Scale */


/*************************************************
** FUNCTION: Vector_Add
** VARIABLES:
**    [I ]  const float v1[3]
**    [I ]  const float v2[3]
**    [IO]        float out[3]
** RETURN:
**    NONE
** DESCRIPTION:
**    Adds two vectors element-wise
**    out[] = v1[] + v2[]
*/
void Vector_Add( const float v1[3], const float v2[3], float out[3] )
{
  for(int c = 0; c < 3; c++) { out[c] = v1[c] + v2[c]; }
} /* End Vector_Add */


/*************************************************
** FUNCTION: Matrix_Matrix_Multiply
** VARIABLES:
**    [I ]  const float m1[3][3]
**    [I ]  const float m2[3][3]
**    [IO]        float out[3][3]
** RETURN:
**    NONE
** DESCRIPTION:
**    Multiply two 3x3 matrices
**    out = m1 * m2
*/
void Matrix_Matrix_Multiply(const float m1[3][3], const float m2[3][3], float out[3][3])
{
  int i, j;
  for( i=0; i<3; i++ )
  {
    for( j=0; j<3; j++ )
    {
      out[i][j] = m1[i][0] * m2[0][j] + m1[i][1] * m2[1][j] + m1[i][2] * m2[2][j];
    }
  }
} /* End Matrix_Matrix_Multiply */


/*************************************************
** FUNCTION: Matrix_Vector_Multiply
** VARIABLES:
**    [I ]  const float m[3][3]
**    [I ]  const float v[3]
**    [IO]        float out[3]
** RETURN:
**    NONE
** DESCRIPTION:
**    Multiply 3x3 matrix with 3x1 vector
**    Outputs a 3x1 vector
**    out = m * v
*/
void Matrix_Vector_Multiply(const float m[3][3], const float v[3], float out[3])
{
  int i;
  for( i=0; i<3; i++ ) { out[i] = m[i][0]*v[0] + m[i][1]*v[1] + m[i][2]*v[2]; }
} /* End Matrix_Vector_Multiply */


/*************************************************
** FUNCTION: Rolling_Mean
** VARIABLES:
**    [I ]  const float m : Initial mean
**    [I ]  const float x : Sample value
**    [I ]    const int n : Sample number
** RETURN:
**    float return
** DESCRIPTION:
**    Compute the rolling mean given an initial mean
**    a sample, and a sample number.
**    The rolling mean is a real-time method of
**    computing a mean.
*/
float Rolling_Mean( const float m, const float x, const int n )
{
  return ( m + (x-m)/n );
} /* End Rolling_Mean */


/*************************************************
** FUNCTION: Windowed_Mean
** VARIABLES:
**    [I ]  const float m : Initial mean
**    [I ]  const float x : Sample value
**    [I ]    const int n : Sample number
**    [I ]  const float a : Exponential factor
** RETURN:
**    float return
** DESCRIPTION:
**    Compute the approximate moving average of the
**    data in a real-time method without storing
**    previous samples.
**    m = m*(1-alpha) + (alpha)*x;
*/
float Windowed_Mean( float m, float x, int n, float a )
{
  /* Error checking */
  if( n==0 ){ return(0.0f); }

  /* Compute and return the moving average */
  return( m*(1-a) + x*(a) );
}

/*************************************************
** FUNCTION: Rolling_SumOfSquares
** VARIABLES:
**    [I ]  const float m_prev  : Previous Mean (at last sample)
**    [I ]  const float m       : Current mean
**    [I ]  const float x       : Sample value
**    [I ]  const float S       : Prev sum of squares
** RETURN:
**    float return
** DESCRIPTION:
**    Compute the rolling sum of squares given
**    the current mean, the previous mean, and a sample.
**    The rolling sum of squares is used as a real-time method of
**    computing a standard deviation.
**    M2 = M2 + (x-m) * (x-m_prev);
*/
float Rolling_SumOfSquares( const float m_prev, const float m, const float x, const float M2 )
{
  return ( M2 + (x-m)*(x-m_prev) );
} /* End Rolling_SumOfSquares */


/*************************************************
** FUNCTION: Rolling_Sample_Variance
** VARIABLES:
**    [I ]  const int N         : Number of samples
**    [I ]  const float M2      : Current sum of squares
** RETURN:
**    float return
** DESCRIPTION:
**    Compute the sample variance from the sum of squares.
**    S2 = M2/(N-1);
*/
float Rolling_Sample_Variance( const int N, const float M2 )
{
  return( M2/(N-1) );
} /* End Rolling_Sample_Variance */


/*************************************************
** FUNCTION: Rolling_Population_Variance
** VARIABLES:
**    [I ]  const int N         : Number of samples
**    [I ]  const float M2      : Current sum of squares
** RETURN:
**    float return
** DESCRIPTION:
**    Compute the population variance from the sum of squares.
**    S2 = M2/(N);
*/
float Rolling_Population_Variance( const int N, const float M2 )
{
  return( M2/N );
} /* End Rolling_Population_Variance */


/*************************************************
** FUNCTION: f_asin
** VARIABLES:
**    [I ]  const float x
** RETURN:
**    float return
** DESCRIPTION:
**    A faster arcsin
*/
float f_asin( float x )
{
  float negate = (float)(x < 0);
  float ret = -0.0187293;
  x = fabs(x);
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
** FUNCTION: f_atan2
** VARIABLES:
**    [I ]  const float x
**    [I ]  const float y
** RETURN:
**    float return
** DESCRIPTION:
**    A faster arctan2
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
** FUNCTION: calc_circle_center
** VARIABLES:
**    [I ]  const float p1[2]   : First point
**    [I ]  const float p1[2]   : Second point
**    [I ]  const float p1[2]   : Third point
**    [IO]  const float xcyc[2] : Center
** RETURN:
**    NONE
** DESCRIPTION:
**    Compute the center of a circle given 3 points
**    in x/y
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

  /* Check for 0 */
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

  /* Compute center */
  xcyc[0] = (mr*mt*(y3-y1) + mr*(x2+x3) - mt*(x1+x2)) / (2*(mr-mt));
  xcyc[1] = -1/mr * (xcyc[0]-(x1+x2)/2) + (y1+y2)/2;

}

/*************************************************
** FUNCTION: circular_shift_1D
** VARIABLES:
**    [IO]  float* A   : The (1D) array to circular shift
**    [I ]  int    len : The length of the array
**    [I ]  int    n   : The number of elements to shift (with -:Left, +:right)
** RETURN:
**    NONE
** DESCRIPTION:
**    Cicular shift a 1D array (i.e. a vector)
*/
void circular_shift_array_1D( 
  float*  A, 
  int     len, 
  int     n )
{
  int i;
  float* imap;
  n %= len;
  imap = (float*)malloc( sizeof(float)*len );
  for(i=0;i<len;i++){ imap[i]=A[ (len+i-n)%len ]; }
  for( i=0; i<len; i++ ){ A[i] = imap[i]; }
  free(imap);
} /* End circular_shift_array_1D() */

/*************************************************
** FUNCTION: circular_shift_2D
** VARIABLES:
**    [IO]  float* A   : The (2D) array to circular shift
**    [I ]  int    nr  : The number of rows in A
**    [I ]  int    nc  : The number of columns in A
**    [I ]  int    n   : The number of elements to shift (with -:Left, +:right)
**    [I ]  int    dim : The dimension to rotate (1:rotate rows, 2:rotate cols)
** RETURN:
**    NONE
** DESCRIPTION:
**    Cicular shift a 2D array (i.e. a matrix)
*/
void circular_shift_array_2D( 
  float*  A, 
  int     nr, 
  int     nc, 
  int     n, 
  int     rdim )
{
  int i,j;
  float* imap;
  
  if( rdim==1 ) /* rotate rows */
  {
    n %= nr;
    imap = (float*)malloc(sizeof(float)*nr);
    for( j=0; j<nc; j++)
    {
      for( i=0; i<nr; i++ ){ imap[i] = A[ nc*((nr+i-n)%nr)+j ]; }
      for( i=0; i<nr; i++ ){ A[i*nc+j] = imap[i]; }
    }
  }
  else if( rdim==2 ) /* rotate cols */
  {
    n %= nc;
    imap = (float*)malloc(sizeof(float)*nc);
    for( j=0; j<nr; j++)
    {
      for( i=0; i<nc; i++ ){ imap[i] = A[ ((nr+i-n)%nr)+j*nc ]; }
      for( i=0; i<nc; i++ ){ A[i+j*nc] = imap[i]; }
    }
  }
  free(imap);
} /* End circular_shift_array_2D() */


/*************************************************
** FUNCTION: Init_Stats_1D
** VARIABLES:
**    [IO]  
** RETURN:
**    NONE
** DESCRIPTION:
**    Initialize the statistics/metrics of the
**    1D sample type.
*/
void Init_Stats_1D( 
  CONTROL_TYPE        *p_control, 
  SAMPLE_DATA_1D_TYPE *p_sample_data )
{
  memset( p_sample_data->val, 0, 3*sizeof(float) );
  p_sample_data->val_min        = FLOAT_MIN;
  p_sample_data->val_min        = FLOAT_MAX;
  p_sample_data->val_mave_alpha = 1; /* 1 : ave[t]==x[t] */
  p_sample_data->val_ave        = 0;
  p_sample_data->val_mave       = 0;
  p_sample_data->val_sam_var    = 0;
  p_sample_data->val_pop_var    = 0;
  memset( p_sample_data->val_sum,  0, 3*sizeof(float) );
  memset( p_sample_data->val_diff, 0, 3*sizeof(float) );  
} /* End Init_Stats_1D() */


/*************************************************
** FUNCTION: Init_Stats_2D
** VARIABLES:
**    [IO]  
** RETURN:
**    NONE
** DESCRIPTION:
**    Initialize the statistics/metrics of a 
**    2D sample type.
*/
void Init_Stats_2D( 
  CONTROL_TYPE        *p_control, 
  SAMPLE_DATA_2D_TYPE *p_sample_data )
{
  memset( p_sample_data->val, 0, 3*3*sizeof(float) );
  memset( p_sample_data->mag, 0, 3*sizeof(float) );
  memset( p_sample_data->dc_offset, 0, 3*sizeof(float) );
  p_sample_data->scale          = 1; 
  p_sample_data->mag_min        = FLOAT_MIN;
  p_sample_data->mag_min        = FLOAT_MAX;
  p_sample_data->mag_mave_alpha = 1; /* 1 : ave[t]==x[t] */
  p_sample_data->mag_ave        = 0;
  p_sample_data->mag_mave       = 0;
  p_sample_data->mag_sam_var    = 0;
  p_sample_data->mag_pop_var    = 0;
  memset( p_sample_data->mag_sum,  0, 3*sizeof(float) );
  memset( p_sample_data->mag_diff, 0, 3*sizeof(float) );  
} /* End Init_Stats_2D() */


/*************************************************
** FUNCTION: Update_Stats_2D
** VARIABLES:
**    [IO]  
** RETURN:
**    NONE
** DESCRIPTION:
**    Adds new sample to structure.
**    Updates the statistics/metrics of a 
**    2D sample type. 
**    The stats/metrics include :
**      1) Get magnitude ( i.e. sqrt(a*a + b*b + c*c) )
**      2) Update magnitude min/max
**      3) Update magnitudewindowed average
**      4) Update magnitudewindowed sample & poulation variance
**      5) Update magnitude cummulative sum and 2-point difference
*/
void Update_Stats_2D(
  CONTROL_TYPE        *p_control, 
  SAMPLE_DATA_2D_TYPE *p_sample_data,
  float               *p_new_sample )
{
  
  float tmp_ave;
  float tmp_mave;
  float tmp_M2;
  
  /* Add new sample into array */
  //circular_shift_array_2D( p_sample_data->val, 3, 3, 1, 1 );
  memcpy( p_sample_data->val, p_new_sample, 3*sizeof(float) );
  
  /* Update magnitude */
  circular_shift_array_1D( p_sample_data->mag, 3, 1 );
  p_sample_data->mag[0] = Vector_Magnitude ( p_sample_data->val );
  p_sample_data->mag[0] = p_sample_data->mag[0]*p_sample_data->scale; /* TO DO : add dc offset */
  
  /* Update min/max */
  p_sample_data->mag_min = MIN(p_sample_data->mag_min,p_sample_data->mag[0]);
  p_sample_data->mag_max = MAX(p_sample_data->mag_min,p_sample_data->mag[0]);
  
  /* Get the new mean */
  tmp_ave = Rolling_Mean( p_sample_data->mag_ave, p_sample_data->mag[0], p_control->SampleNumber );
  
  /* Get the new windowed mean */
  tmp_mave = Windowed_Mean( p_sample_data->mag_mave, p_sample_data->mag[0], p_control->SampleNumber, p_sample_data->mag_mave_alpha );
  
  /* Update M2 (sum of squares on nth point) */
  tmp_M2 = Rolling_SumOfSquares( p_sample_data->mag_ave, tmp_ave, p_sample_data->mag[0], p_sample_data->mag_M2 ); /* from full mean */
  //tmp_M2 = Rolling_SumOfSquares( p_sample_data->mag_mave, tmp_mave, p_sample_data->mag[0], p_sample_data->mag_M2 ); /* from windowed mean */
  
  /* Update rolling sample variance */
  p_sample_data->mag_sam_var = Rolling_Sample_Variance( p_control->SampleNumber, p_sample_data->mag_M2 );
  
  /* Update rolling population variance */
  p_sample_data->mag_pop_var = Rolling_Population_Variance( p_control->SampleNumber, p_sample_data->mag_M2 );
  
  /* Finish update */
  p_sample_data->mag_ave  = tmp_ave;
  p_sample_data->mag_mave = tmp_mave;
  
} /* End Update_Stats_2D() */

/*************************************************
** FUNCTION: Update_Stats_1D
** VARIABLES:
**    [IO]  
** RETURN:
**    NONE
** DESCRIPTION:
**    Adds new sample to structure.
**    Updates the statistics/metrics of a 
**    1D sample type. 
**    The stats/metrics include :
**      1) Update min/max
**      2) Update windowed average
**      3) Update windowed sample & poulation variance
**      4) Update cummulative sum and 2-point difference
*/
void Update_Stats_1D( 
  CONTROL_TYPE        *p_control, 
  SAMPLE_DATA_1D_TYPE *p_sample_data,
  float                new_sample )
{
  
  float tmp_ave;
  float tmp_mave;
  float tmp_M2;
  
  /* Add new sample into array */
  circular_shift_array_1D( p_sample_data->val, 3, 1 );
  p_sample_data->val[0] = new_sample;
  
  /* Update min/max */
  p_sample_data->val_min = MIN(p_sample_data->val_min,p_sample_data->val[0]);
  p_sample_data->val_max = MAX(p_sample_data->val_min,p_sample_data->val[0]);
  
  /* Get the new mean */
  tmp_ave = Rolling_Mean( p_sample_data->val_ave, p_sample_data->val[0], p_control->SampleNumber );
  
  /* Get the new windowed mean */
  tmp_mave = Windowed_Mean( p_sample_data->val_mave, p_sample_data->val[0], p_control->SampleNumber, p_sample_data->val_mave_alpha );
  
  /* Update M2 (sum of squares on nth point) */
  tmp_M2 = Rolling_SumOfSquares( p_sample_data->val_ave, tmp_ave, p_sample_data->val[0], p_sample_data->val_M2 ); /* from full mean */
  ////tmp_M2 = Rolling_SumOfSquares( p_sample_data->val_mave, tmp_mave, p_sample_data->val[0], p_sample_data->val_M2 ); /* from windowed mean */
  
  /* Update rolling sample variance */
  p_sample_data->val_sam_var = Rolling_Sample_Variance( p_control->SampleNumber, p_sample_data->val_M2 );
  
  /* Update rolling population variance */
  p_sample_data->val_pop_var = Rolling_Population_Variance( p_control->SampleNumber, p_sample_data->val_M2 );
  
  /* Finish update */
  p_sample_data->val_ave  = tmp_ave;
  p_sample_data->val_mave = tmp_mave;
  
} /* End Update_Stats_1D() */




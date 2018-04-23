// Type E Thermocouple library per ITS-90

// *** BSD License ***
// ------------------------------------------------------------------------------------------
//
// Author: T81
//
// Redistribution and use in source and binary forms, with or without modification, are 
// permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this list of 
// conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice, this list 
// of conditions and the following disclaimer in the documentation and/or other materials 
// provided with the distribution.
//
// Neither the name of the MLG Properties, LLC nor the names of its contributors may be 
// used to endorse or promote products derived from this software without specific prior 
// written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
// THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// Original Author of Type K thermocouples library: Jim Gallt
// ------------------------------------------------------------------------------------------

#include "TypeE.h"
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif


// -------------------------------------
const int TypeE::nranges_inv = 2;  // number of mV ranges for inverse lookup
const int TypeE::ncoeff_inv = 10;  // number of coefficients for inverse lookup
const float TypeE::mv_min = -8.825;
const float TypeE::mv_max = 76.373;

// coefficients for inverse lookup (given mV, find C)
const double TypeE::coeff_inv[10][2] = {
	{  0.0000000E+00,  0.0000000E+00 },
	{  1.6977288E+01,  1.7057035E+01 }, 
	{ -4.3514970E-01, -2.3301759E-01 },
	{ -1.5859697E-01,  6.5435585E-03 },
	{ -9.2502871E-02, -7.3562749E-05 },
	{ -2.6084314E-02, -1.7896001E-06 },
	{ -4.1360199E-03,  8.4036165E-08 },
	{ -3.4034030E-04, -1.3735879E-09 },
	{ -1.1564890E-05,  1.0629823E-11 },
	{  0.0000000E+00, -3.2447087E-14 }
};

// mV ranges for inverse lookup coefficients
const float TypeE::range_inv[2][2] = {
  { -8.825,          0.000 },
  {  0.000,         76.373 }
};

// coefficients for direct lookup (given C, find mV)
const double TypeE::coeff_dir[14][2] = {
	{  0.000000000000E+00,  0.000000000000E+00 },
	{  0.586655087080E-01,  0.586655087100E-01 },
	{  0.454109771240E-04,  0.450322755820E-04 },
	{ -0.779980486860E-06,  0.289084072120E-07 },
	{ -0.258001608430E-07, -0.330568966520E-09 },
	{ -0.594525830570E-09,  0.650244032700E-12 },
	{ -0.932140586670E-11, -0.191974955040E-15 },
	{ -0.102876055340E-12, -0.125366004970E-17 },
	{ -0.803701236210E-15,  0.214892175690E-20 },
	{ -0.439794973910E-17, -0.143880417820E-23 },
	{ -0.164147763550E-19,  0.359608994810E-27 },
	{ -0.396736195160E-22,  0.000000000000E+00 },
	{ -0.558273287210E-25,  0.000000000000E+00 },
	{ -0.346578420130E-28,  0.000000000000E+00 }
};

// ranges for direct lookup
const double TypeE::range_dir[2][2] = {
  { -270.000 ,  0.000 },
  {    0.000 ,1000.00 }
};

const float TypeE::C_max = 1000.0;
const float TypeE::C_min = -270.0;

// -------------------------- constructor
TypeE::TypeE() {
  F_max = C_TO_F( C_max );
  F_min = C_TO_F( C_min );
}

// ------------------- given mv reading, returns absolute temp C
double TypeE::Temp_C( float mv ) {
  double x = 1.0;
  double sum = 0.0;
  int i,j,ind;
  ind = 0;
  if ( ! inrange_mV( mv ) ) return TC_RANGE_ERR;
  // first figure out which range of values
  for( j = 0; j < nranges_inv; j++ ) {
    if((mv >= range_inv[0][j]) && (mv <= range_inv[1][j]))
      ind = j;
  };
//  Serial.println(ind);
  for( i = 0; i < ncoeff_inv; i++ ) {
    sum += x * coeff_inv[i][ind];
    x *= mv;
  }
  return sum;  
}

// --------- given mv reading and ambient temp, returns compensated (true)
//           temperature at tip of sensor
double TypeE::Temp_C( float mv, float amb ) {
  float mv_amb;
  mv_amb = mV_C( amb );
  return Temp_C( mv + mv_amb );
};

// --------------------- returns compensated temperature in F units
double TypeE::Temp_F( float mv, float amb ) {
  return C_TO_F( Temp_C( mv, F_TO_C( amb ) ) );
};

// --------------------- returns absolute temperature in F units
double TypeE::Temp_F( float mv ) {
  float temp = Temp_C( mv );
  if( temp == TC_RANGE_ERR ) return TC_RANGE_ERR;
  return C_TO_F( temp );  
}

// --------------------- checks to make sure mv signal in range
boolean TypeE::inrange_mV( float mv ) {
  return ( mv >= mv_min ) & ( mv <= mv_max );
};

// ---------------------- checks to make sure temperature in range
boolean TypeE::inrange_C( float ambC ) {
  return ( ambC >= C_min ) & ( ambC <= C_max );
};

// ----------------------- checks to make sure temperature in range
boolean TypeE::inrange_F( float ambF ) {
  return ( ambF >= F_min ) & ( ambF <= F_max );
};

// ---------------- returns mV corresponding to temp reading
//                  used for cold junction compensation
double TypeE::mV_C( float ambC ) {
  double sum = 0.0;
  double x = 1.0;
  int i;
  if( !inrange_C( ambC ) ) return TC_RANGE_ERR;

  if( (ambC >= range_dir[0][0]) && ( ambC <= range_dir[1][0] ) ) {
    for( i = 0; i < 14; i++ ) {
      sum += x * coeff_dir[i][0];
      x *= ambC;
    } 
  }
  else {
    for( i = 0; i < 10; i++ ) {
      sum += x * coeff_dir[i][1];
      x *= ambC;    
    };
//    Serial.print( sum ); Serial.print(" , ");
  };  
  return sum;
};

// -------------------- cold junction compensation in F units
double TypeE::mV_F( float ambF ) {
  if( inrange_F( ambF ) )
    return mV_C( F_TO_C( ambF ) );
  else
    return TC_RANGE_ERR;
};

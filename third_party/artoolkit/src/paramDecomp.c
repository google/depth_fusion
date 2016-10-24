/*******************************************************
 *
 * Author: Takeshi Mita, Shinsaku Hiura, Hirokazu Kato
 *
 *         tmita@inolab.sys.es.osaka-u.ac.jp
 *         shinsaku@sys.es.osaka-u.ac.jp
 *         kato@sys.im.hiroshima-cu.ac.jp
 *
 * Revision: 4.1
 * Date: 01/12/07
 *
*******************************************************/

#include <stdio.h>
#include <math.h>
#include <AR/param.h>
#include <AR/matrix.h>

static double norm( double a, double b, double c );
static double dot( double a1, double a2, double a3,
                   double b1, double b2, double b3 );

int  arParamDecomp( ARParam *source, ARParam *icpara, double trans[3][4] )
{
    icpara->xsize          = source->xsize;
    icpara->ysize          = source->ysize;
    icpara->dist_factor[0] = source->dist_factor[0];
    icpara->dist_factor[1] = source->dist_factor[1];
    icpara->dist_factor[2] = source->dist_factor[2];
    icpara->dist_factor[3] = source->dist_factor[3];

    return arParamDecompMat( source->mat, icpara->mat, trans );
}

int  arParamDecompMat( double source[3][4], double cpara[3][4], double trans[3][4] )
{
    int       r, c;
    double    Cpara[3][4];
    double    rem1, rem2, rem3;

    if( source[2][3] >= 0 ) {
        for( r = 0; r < 3; r++ ){
	    for( c = 0; c < 4; c++ ){
                Cpara[r][c] = source[r][c];
            }
        }
    }
    else {
        for( r = 0; r < 3; r++ ){
	    for( c = 0; c < 4; c++ ){
                Cpara[r][c] = -(source[r][c]);
            }
        }
    }

    for( r = 0; r < 3; r++ ){
	for( c = 0; c < 4; c++ ){
            cpara[r][c] = 0.0;
	}
    }
    cpara[2][2] = norm( Cpara[2][0], Cpara[2][1], Cpara[2][2] );
    trans[2][0] = Cpara[2][0] / cpara[2][2];
    trans[2][1] = Cpara[2][1] / cpara[2][2];
    trans[2][2] = Cpara[2][2] / cpara[2][2];
    trans[2][3] = Cpara[2][3] / cpara[2][2];
	
    cpara[1][2] = dot( trans[2][0], trans[2][1], trans[2][2],
                       Cpara[1][0], Cpara[1][1], Cpara[1][2] );
    rem1 = Cpara[1][0] - cpara[1][2] * trans[2][0];
    rem2 = Cpara[1][1] - cpara[1][2] * trans[2][1];
    rem3 = Cpara[1][2] - cpara[1][2] * trans[2][2];
    cpara[1][1] = norm( rem1, rem2, rem3 );
    trans[1][0] = rem1 / cpara[1][1];
    trans[1][1] = rem2 / cpara[1][1];
    trans[1][2] = rem3 / cpara[1][1];

    cpara[0][2] = dot( trans[2][0], trans[2][1], trans[2][2],
                       Cpara[0][0], Cpara[0][1], Cpara[0][2] );
    cpara[0][1] = dot( trans[1][0], trans[1][1], trans[1][2],
                       Cpara[0][0], Cpara[0][1], Cpara[0][2] );
    rem1 = Cpara[0][0] - cpara[0][1]*trans[1][0] - cpara[0][2]*trans[2][0];
    rem2 = Cpara[0][1] - cpara[0][1]*trans[1][1] - cpara[0][2]*trans[2][1];
    rem3 = Cpara[0][2] - cpara[0][1]*trans[1][2] - cpara[0][2]*trans[2][2];
    cpara[0][0] = norm( rem1, rem2, rem3 );
    trans[0][0] = rem1 / cpara[0][0];
    trans[0][1] = rem2 / cpara[0][0];
    trans[0][2] = rem3 / cpara[0][0];

    trans[1][3] = (Cpara[1][3] - cpara[1][2]*trans[2][3]) / cpara[1][1];
    trans[0][3] = (Cpara[0][3] - cpara[0][1]*trans[1][3]
                               - cpara[0][2]*trans[2][3]) / cpara[0][0];

    for( r = 0; r < 3; r++ ){
	for( c = 0; c < 3; c++ ){
            cpara[r][c] /= cpara[2][2];
	}
    }

    return 0;
}

int arsParamGetMat( double matL[3][4], double matR[3][4],
                    double cparaL[3][4], double cparaR[3][4], double matL2R[3][4] )
{
    ARMat    *t1, *t2, *t3;
    double   transL[3][4], transR[3][4];
    int      i, j;

    arParamDecompMat( matL, cparaL, transL );
    arParamDecompMat( matR, cparaR, transR );

    t1 = arMatrixAlloc( 4, 4 );
    t2 = arMatrixAlloc( 4, 4 );
    for( j = 0; j < 3; j++ ) {
       for( i = 0; i < 4; i++ ) {
            t1->m[j*4+i] = transL[j][i];
            t2->m[j*4+i] = transR[j][i];
        }
    }
    t1->m[12] = t1->m[13] = t1->m[14] = 0.0;
    t1->m[15] = 1.0;
    t2->m[12] = t2->m[13] = t2->m[14] = 0.0;
    t2->m[15] = 1.0;

    if( arMatrixSelfInv(t1) != 0 ) {
        arMatrixFree( t1 );
        arMatrixFree( t2 );
        return -1;
    }
    t3 = arMatrixAllocMul(t2, t1);
    if( t3 == NULL ) {
        arMatrixFree( t1 );
        arMatrixFree( t2 );
        return -1;
    }

    for( j = 0; j < 3; j++ ) {
       for( i = 0; i < 4; i++ ) {
            matL2R[j][i] = t3->m[j*4+i];
        }
    }

    arMatrixFree( t1 );
    arMatrixFree( t2 );
    arMatrixFree( t3 );

    return 0;
}

void argConvGLcpara2(ARParam *param, int width, int height, double gnear, double gfar, float m[16])
{
    double   icpara[3][4];
    double   trans[3][4];
    double   p[3][3], q[4][4];
    int      i, j;

    if (arParamDecompMat(param->mat, icpara, trans) < 0) {
        printf("gConvGLcpara: Parameter error!!\n");
    }

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            p[i][j] = icpara[i][j] / icpara[2][2];
        }
    }
    q[0][0] = (2.0 * p[0][0] / width);
    q[0][1] = -(2.0 * p[0][1] / width);
    q[0][2] = ((2.0 * p[0][2] / width) - 1.0);
    q[0][3] = 0.0;

    q[1][0] = 0.0;
    q[1][1] = -(2.0 * p[1][1] / height);
    q[1][2] = ((2.0 * p[1][2] / height) - 1.0);
    q[1][3] = 0.0;

    q[2][0] = 0.0;
    q[2][1] = 0.0;
    q[2][2] = (gfar + gnear) / (gfar - gnear);
    q[2][3] = -2.0 * gfar * gnear / (gfar - gnear);

    q[3][0] = 0.0;
    q[3][1] = 0.0;
    q[3][2] = 1.0;
    q[3][3] = 0.0;

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 3; j++) {
            m[i + j * 4] = (float)(q[i][0] * trans[0][j]
                + q[i][1] * trans[1][j]
                + q[i][2] * trans[2][j]);
        }
        m[i + 3 * 4] = (float)(q[i][0] * trans[0][3]
            + q[i][1] * trans[1][3]
            + q[i][2] * trans[2][3]
            + q[i][3]);
    }
}


static double norm( double a, double b, double c )
{
    return( sqrt( a*a + b*b + c*c ) );
}

static double dot( double a1, double a2, double a3,
		   double b1, double b2, double b3 )
{
    return( a1 * b1 + a2 * b2 + a3 * b3 );
}

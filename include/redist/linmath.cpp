// Copyright 2013,2016 <>< C. N. Lohr.  This file licensed under the terms of the MIT license.
#if defined(_WIN32) && !defined(TCC)
#define LINMATH_EXPORT __declspec(dllexport)
#endif
#include "linmath.h"
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

// #include "opencv2/opencv.hpp"
#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <malloc.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846 /* pi */
#endif

void cross3d(FLT *out, const FLT *a, const FLT *b)
{
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}

void sub3d(FLT *out, const FLT *a, const FLT *b)
{
	out[0] = a[0] - b[0];
	out[1] = a[1] - b[1];
	out[2] = a[2] - b[2];
}
LINMATH_EXPORT void addnd(FLT *out, const FLT *a, const FLT *b, size_t size)
{
	for (int i = 0; i < size; i++)
		out[i] = a[i] + b[i];
}
LINMATH_EXPORT void subnd(FLT *out, const FLT *a, const FLT *b, size_t size)
{
	for (int i = 0; i < size; i++)
		out[i] = a[i] - b[i];
}
void add3d(FLT *out, const FLT *a, const FLT *b)
{
	out[0] = a[0] + b[0];
	out[1] = a[1] + b[1];
	out[2] = a[2] + b[2];
}

void invert3d(FLT *out, const FLT *a)
{
	out[0] = 1. / a[0];
	out[1] = 1. / a[1];
	out[2] = 1. / a[2];
}
void scalend(FLT *out, const FLT *a, FLT scalar, size_t size)
{
	for (size_t i = 0; i < size; i++)
	{
		out[i] = a[i] * scalar;
	}
}
void scale3d(FLT *out, const FLT *a, FLT scalar)
{
	out[0] = a[0] * scalar;
	out[1] = a[1] * scalar;
	out[2] = a[2] * scalar;
}

LINMATH_EXPORT FLT norm3d(const FLT *in) { return FLT_SQRT(in[0] * in[0] + in[1] * in[1] + in[2] * in[2]); }
LINMATH_EXPORT FLT normnd2(const FLT *in, size_t n)
{
	FLT r = 0;
	for (int i = 0; i < n; i++)
	{
		r += in[i] * in[i];
	}
	return r;
}
LINMATH_EXPORT FLT normnd(const FLT *in, size_t n) { return FLT_SQRT(normnd2(in, n)); }
void normalize3d(FLT *out, const FLT *in)
{
	FLT r = ((FLT)1.) / norm3d(in);
	out[0] = in[0] * r;
	out[1] = in[1] * r;
	out[2] = in[2] * r;
}

void linmath_interpolate(FLT *out, int n, const FLT *A, const FLT *B, FLT t)
{
	for (int i = 0; i < n; i++)
	{
		out[i] = A[i] + (B[i] - A[i]) * t;
	}
}

LINMATH_EXPORT FLT dot3d(const FLT *a, const FLT *b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

int compare3d(const FLT *a, const FLT *b, FLT epsilon)
{
	if (!a || !b)
		return 0;
	if (a[2] - b[2] > epsilon)
		return 1;
	if (b[2] - a[2] > epsilon)
		return -1;
	if (a[1] - b[1] > epsilon)
		return 1;
	if (b[1] - a[1] > epsilon)
		return -1;
	if (a[0] - b[0] > epsilon)
		return 1;
	if (b[0] - a[0] > epsilon)
		return -1;
	return 0;
}

void copy3d(FLT *out, const FLT *in)
{
	out[0] = in[0];
	out[1] = in[1];
	out[2] = in[2];
}

LINMATH_EXPORT FLT magnitude3d(const FLT *a) { return FLT_SQRT(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }
FLT dist3d(const FLT *a, const FLT *b)
{
	LinmathPoint3d tmp;
	sub3d(tmp, a, b);
	return magnitude3d(tmp);
}

FLT anglebetween3d(FLT *a, FLT *b)
{
	FLT an[3];
	FLT bn[3];
	normalize3d(an, a);
	normalize3d(bn, b);
	FLT dot = dot3d(an, bn);
	if (dot < -0.9999999)
		return LINMATHPI;
	if (dot > 0.9999999)
		return 0;
	return FLT_ACOS(dot);
}

LINMATH_EXPORT void center3d(FLT *out_pts, FLT *out_mean, const FLT *pts, int num_pts)
{
	FLT center[3];
	if (out_mean == 0)
		out_mean = center;
	mean3d(out_mean, pts, num_pts);

	for (int i = 0; i < num_pts; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			out_pts[i * 3 + j] = pts[i * 3 + j] - out_mean[j];
		}
	}
}

LINMATH_EXPORT void mean3d(LinmathVec3d out, const FLT *pts, int num_pts)
{
	for (int i = 0; i < 3; i++)
	{
		out[i] = 0;
	}

	for (int i = 0; i < num_pts; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			out[j] += pts[i * 3 + j];
		}
	}

	for (int j = 0; j < 3; j++)
	{
		out[j] = out[j] / (FLT)num_pts;
	}
}

// algorithm found here: http://inside.mines.edu/fs_home/gmurray/ArbitraryAxisRotation/
void rotatearoundaxis(FLT *outvec3, const FLT *invec3, const FLT *_axis, FLT angle)
{
	FLT axis[3];
	normalize3d(axis, _axis);

	FLT s = FLT_SIN(angle);
	FLT c = FLT_COS(angle);

	FLT u = axis[0];
	FLT v = axis[1];
	FLT w = axis[2];

	FLT x = invec3[0];
	FLT y = invec3[1];
	FLT z = invec3[2];

	outvec3[0] = u * (u * x + v * y + w * z) * (1 - c) + x * c + (-w * y + v * z) * s;
	outvec3[1] = v * (u * x + v * y + w * z) * (1 - c) + y * c + (w * x - u * z) * s;
	outvec3[2] = w * (u * x + v * y + w * z) * (1 - c) + z * c + (-v * x + u * y) * s;
}

void axisanglerotatevector(FLT *vec3out, const LinmathAxisAngle axisAngle, const FLT *vec3in)
{
	FLT angle = norm3d(axisAngle);
	if (angle == 0.0)
	{
		copy3d(vec3out, vec3in);
	}
	else
	{
		rotatearoundaxis(vec3out, vec3in, axisAngle, angle);
	}
}

void angleaxisfrom2vect(FLT *angle, FLT *axis, FLT *src, FLT *dest)
{
	FLT v0[3];
	FLT v1[3];
	normalize3d(v0, src);
	normalize3d(v1, dest);

	FLT d = dot3d(v0, v1); // v0.dotProduct(v1);

	// If dot == 1, vectors are the same
	// If dot == -1, vectors are opposite
	if (FLT_FABS(d - 1) < DEFAULT_EPSILON)
	{
		axis[0] = 0;
		axis[1] = 1;
		axis[2] = 0;
		*angle = 0;
		return;
	}
	else if (FLT_FABS(d + 1) < DEFAULT_EPSILON)
	{
		axis[0] = 0;
		axis[1] = 1;
		axis[2] = 0;
		*angle = LINMATHPI;
		return;
	}

	FLT v0Len = magnitude3d(v0);
	FLT v1Len = magnitude3d(v1);

	*angle = FLT_ACOS(d / (v0Len * v1Len));

	// cross3d(c, v0, v1);
	cross3d(axis, v1, v0);
}

void axisanglefromquat(FLT *angle, FLT *axis, FLT *q)
{
	// this way might be fine, too.
	// FLT dist = FLT_SQRT((q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]));
	//
	//*angle = 2 * FLT_ATAN2(dist, q[0]);

	// axis[0] = q[1] / dist;
	// axis[1] = q[2] / dist;
	// axis[2] = q[3] / dist;

	// Good mathematical foundation for this algorithm found here:
	// http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm

	FLT tmp[4] = {q[0], q[1], q[2], q[3]};

	quatnormalize(tmp, q);

	if (FLT_FABS(q[0] - 1) < FLT_EPSILON)
	{
		// we have a degenerate case where we're rotating approx. 0 degrees
		*angle = 0;
		axis[0] = 1;
		axis[1] = 0;
		axis[2] = 0;
		return;
	}

	axis[0] = tmp[1] / sqrt(1 - (tmp[0] * tmp[0]));
	axis[1] = tmp[2] / sqrt(1 - (tmp[0] * tmp[0]));
	axis[2] = tmp[3] / sqrt(1 - (tmp[0] * tmp[0]));

	*angle = 2 * FLT_ACOS(tmp[0]);
}

/////////////////////////////////////QUATERNIONS//////////////////////////////////////////
// Originally from Mercury (Copyright (C) 2009 by Joshua Allen, Charles Lohr, Adam Lowman)
// Under the mit/X11 license.

FLT quatdifference(const LinmathQuat q1, const LinmathQuat q2)
{
	LinmathQuat diff;
	quatfind(diff, q1, q2);
	return 1. - diff[0];
}
FLT quatdist(const FLT *q1, const FLT *q2)
{
	FLT rtn = 0;
	for (int i = 0; i < 4; i++)
	{
		rtn += q1[i] * q2[i];
	}
	rtn = linmath_max(1., linmath_min(-1, rtn));
	return 2 * acos(FLT_FABS(rtn));
}

bool quatiszero(const LinmathQuat q) { return q[0] == 0 && q[1] == 0 && q[2] == 0 && q[3] == 0; }
void quatset(LinmathQuat q, FLT w, FLT x, FLT y, FLT z)
{
	q[0] = w;
	q[1] = x;
	q[2] = y;
	q[3] = z;
}

void quatsetnone(LinmathQuat q)
{
	q[0] = 1;
	q[1] = 0;
	q[2] = 0;
	q[3] = 0;
}

void quatcopy(LinmathQuat qout, const LinmathQuat qin)
{
	qout[0] = qin[0];
	qout[1] = qin[1];
	qout[2] = qin[2];
	qout[3] = qin[3];
}

void quatfromeuler(LinmathQuat q, const LinmathEulerAngle euler)
{
	FLT X = euler[0] / 2.0f; // roll
	FLT Y = euler[1] / 2.0f; // pitch
	FLT Z = euler[2] / 2.0f; // yaw

	FLT cx = FLT_COS(X);
	FLT sx = FLT_SIN(X);
	FLT cy = FLT_COS(Y);
	FLT sy = FLT_SIN(Y);
	FLT cz = FLT_COS(Z);
	FLT sz = FLT_SIN(Z);

	// Correct according to
	// http://en.wikipedia.org/wiki/Conversion_between_MQuaternions_and_Euler_angles
	q[0] = cx * cy * cz + sx * sy * sz; // q1
	q[1] = sx * cy * cz - cx * sy * sz; // q2
	q[2] = cx * sy * cz + sx * cy * sz; // q3
	q[3] = cx * cy * sz - sx * sy * cz; // q4
	quatnormalize(q, q);
}

void quattoeuler(LinmathEulerAngle euler, const LinmathQuat q)
{
	// According to http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles (Oct 26, 2009)
	euler[0] = FLT_ATAN2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
	euler[1] = FLT_ASIN(2 * (q[0] * q[2] - q[3] * q[1]));
	euler[2] = FLT_ATAN2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
}

void quatfromaxisanglemag(LinmathQuat q, const LinmathAxisAngleMag axisangle)
{
	FLT radians = norm3d(axisangle);

	if (radians == 0.0)
	{
		quatcopy(q, LinmathQuat_Identity);
	}
	else
	{
		FLT sn = FLT_SIN(radians / 2.0f);
		q[0] = FLT_COS(radians / 2.0f);
		q[1] = sn * axisangle[0] / radians;
		q[2] = sn * axisangle[1] / radians;
		q[3] = sn * axisangle[2] / radians;
		quatnormalize(q, q);
	}
}
void quatfromaxisangle(LinmathQuat q, const FLT *axis, FLT radians)
{
	if (radians == 0.0)
	{
		quatcopy(q, LinmathQuat_Identity);
		return;
	}
	FLT v[3];
	normalize3d(v, axis);

	FLT sn = FLT_SIN(radians / 2.0f);
	q[0] = FLT_COS(radians / 2.0f);
	q[1] = sn * v[0];
	q[2] = sn * v[1];
	q[3] = sn * v[2];

	quatnormalize(q, q);
}

FLT quatmagnitude(const LinmathQuat q)
{
	return FLT_SQRT((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]));
}

FLT quatinvsqmagnitude(const LinmathQuat q)
{
	return ((FLT)1.) / FLT_SQRT((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]));
}

void quatnormalize(LinmathQuat qout, const LinmathQuat qin)
{
	FLT norm = quatmagnitude(qin);
	quatdivs(qout, qin, norm);
}

void quattomatrix(FLT *matrix44, const LinmathQuat qin)
{
	FLT q[4];
	quatnormalize(q, qin);

	// Reduced calculation for speed
	FLT xx = 2 * q[1] * q[1];
	FLT xy = 2 * q[1] * q[2];
	FLT xz = 2 * q[1] * q[3];
	FLT xw = 2 * q[1] * q[0];

	FLT yy = 2 * q[2] * q[2];
	FLT yz = 2 * q[2] * q[3];
	FLT yw = 2 * q[2] * q[0];

	FLT zz = 2 * q[3] * q[3];
	FLT zw = 2 * q[3] * q[0];

	// opengl major
	matrix44[0] = 1 - yy - zz;
	matrix44[1] = xy - zw;
	matrix44[2] = xz + yw;
	matrix44[3] = 0;

	matrix44[4] = xy + zw;
	matrix44[5] = 1 - xx - zz;
	matrix44[6] = yz - xw;
	matrix44[7] = 0;

	matrix44[8] = xz - yw;
	matrix44[9] = yz + xw;
	matrix44[10] = 1 - xx - yy;
	matrix44[11] = 0;

	matrix44[12] = 0;
	matrix44[13] = 0;
	matrix44[14] = 0;
	matrix44[15] = 1;
}
void quatfrommatrix33(FLT *q, const FLT *m)
{
	FLT m00 = m[0], m01 = m[1], m02 = m[2], m10 = m[3], m11 = m[4], m12 = m[5], m20 = m[6], m21 = m[7], m22 = m[8];

	FLT tr = m00 + m11 + m22;

	if (tr > 0)
	{
		FLT S = sqrt(tr + 1.0) * 2; // S=4*qw
		q[0] = 0.25 * S;
		q[1] = (m21 - m12) / S;
		q[2] = (m02 - m20) / S;
		q[3] = (m10 - m01) / S;
	}
	else if ((m00 > m11) & (m00 > m22))
	{
		FLT S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*q[1]
		q[0] = (m21 - m12) / S;
		q[1] = 0.25 * S;
		q[2] = (m01 + m10) / S;
		q[3] = (m02 + m20) / S;
	}
	else if (m11 > m22)
	{
		FLT S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*q[2]
		q[0] = (m02 - m20) / S;
		q[1] = (m01 + m10) / S;
		q[2] = 0.25 * S;
		q[3] = (m12 + m21) / S;
	}
	else
	{
		FLT S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*q[3]
		q[0] = (m10 - m01) / S;
		q[1] = (m02 + m20) / S;
		q[2] = (m12 + m21) / S;
		q[3] = 0.25 * S;
	}
}

void quatfrommatrix(LinmathQuat q, const FLT *matrix44)
{
	// Algorithm from http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
	FLT tr = matrix44[0] + matrix44[5] + matrix44[10];

	if (tr > 0)
	{
		FLT S = FLT_SQRT(tr + 1.0) * 2.; // S=4*qw
		q[0] = 0.25f * S;
		q[1] = (matrix44[9] - matrix44[6]) / S;
		q[2] = (matrix44[2] - matrix44[8]) / S;
		q[3] = (matrix44[4] - matrix44[1]) / S;
	}
	else if ((matrix44[0] > matrix44[5]) && (matrix44[0] > matrix44[10]))
	{
		FLT S = FLT_SQRT(1.0 + matrix44[0] - matrix44[5] - matrix44[10]) * 2.; // S=4*qx
		q[0] = (matrix44[9] - matrix44[6]) / S;
		q[1] = 0.25f * S;
		q[2] = (matrix44[1] + matrix44[4]) / S;
		q[3] = (matrix44[2] + matrix44[8]) / S;
	}
	else if (matrix44[5] > matrix44[10])
	{
		FLT S = FLT_SQRT(1.0 + matrix44[5] - matrix44[0] - matrix44[10]) * 2.; // S=4*qy
		q[0] = (matrix44[2] - matrix44[8]) / S;
		q[1] = (matrix44[1] + matrix44[4]) / S;
		q[2] = 0.25f * S;
		q[3] = (matrix44[6] + matrix44[9]) / S;
	}
	else
	{
		FLT S = FLT_SQRT(1.0 + matrix44[10] - matrix44[0] - matrix44[5]) * 2.; // S=4*qz
		q[0] = (matrix44[4] - matrix44[1]) / S;
		q[1] = (matrix44[2] + matrix44[8]) / S;
		q[2] = (matrix44[6] + matrix44[9]) / S;
		q[3] = 0.25 * S;
	}
}

// Algorithm from http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/
void quattomatrix33(FLT *matrix33, const LinmathQuat qin)
{
	FLT q[4];
	quatnormalize(q, qin);

	// Reduced calulation for speed
	FLT xx = 2 * q[1] * q[1];
	FLT xy = 2 * q[1] * q[2];
	FLT xz = 2 * q[1] * q[3];
	FLT xw = 2 * q[1] * q[0];

	FLT yy = 2 * q[2] * q[2];
	FLT yz = 2 * q[2] * q[3];
	FLT yw = 2 * q[2] * q[0];

	FLT zz = 2 * q[3] * q[3];
	FLT zw = 2 * q[3] * q[0];

	// opengl major
	matrix33[0] = 1 - yy - zz;
	matrix33[1] = xy + zw;
	matrix33[2] = xz - yw;

	matrix33[3] = xy - zw;
	matrix33[4] = 1 - xx - zz;
	matrix33[5] = yz + xw;

	matrix33[6] = xz + yw;
	matrix33[7] = yz - xw;
	matrix33[8] = 1 - xx - yy;
}

void quatgetconjugate(LinmathQuat qout, const LinmathQuat qin)
{
	qout[0] = qin[0];
	qout[1] = -qin[1];
	qout[2] = -qin[2];
	qout[3] = -qin[3];
}

void quatgetreciprocal(LinmathQuat qout, const LinmathQuat qin)
{
	FLT m = quatinvsqmagnitude(qin);
	quatgetconjugate(qout, qin);
	quatscale(qout, qout, m);
}

void quatconjugateby(LinmathQuat q, const LinmathQuat r, const LinmathQuat v)
{
	LinmathQuat ir;
	quatgetconjugate(ir, r);
	quatrotateabout(q, ir, v);
	quatrotateabout(q, q, r);
	quatnormalize(q, q);
}
void quatsub(LinmathQuat qout, const FLT *a, const FLT *b)
{
	qout[0] = a[0] - b[0];
	qout[1] = a[1] - b[1];
	qout[2] = a[2] - b[2];
	qout[3] = a[3] - b[3];
}

void quatadd(LinmathQuat qout, const FLT *a, const FLT *b)
{
	qout[0] = a[0] + b[0];
	qout[1] = a[1] + b[1];
	qout[2] = a[2] + b[2];
	qout[3] = a[3] + b[3];
}

void quatfind_between_vectors(LinmathQuat q, const LinmathPoint3d _p0, const LinmathPoint3d _p1)
{
	LinmathPoint3d p0, p1;
	normalize3d(p0, _p0);
	normalize3d(p1, _p1);

	FLT d = dot3d(p0, p1);

	if (d < -0.999999)
	{
		LinmathPoint3d tmp, xUnit = {1, 0, 0}, yUnit = {0, 1, 0};

		cross3d(tmp, xUnit, p0);
		if (norm3d(tmp) < .00001)
			cross3d(tmp, yUnit, p0);
		normalize3d(tmp, tmp);
		scale3d(tmp, tmp, LINMATHPI);

		quatfromaxisanglemag(q, tmp);
	}
	else if (d >= 1.)
	{
		quatcopy(q, LinmathQuat_Identity);
	}
	else
	{
		cross3d(q + 1, p0, p1);
		q[0] = 1 + d;
		quatnormalize(q, q);
	}
}
void quatfind(LinmathQuat q, const LinmathQuat q0, const LinmathQuat q1)
{
	LinmathQuat iq0;
	quatgetconjugate(iq0, q0);
	quatrotateabout(q, q1, iq0);
}

void axisanglerotateabout(LinmathAxisAngle out, const LinmathAxisAngle a0, const LinmathAxisAngle a1)
{
	LinmathQuat q0, q1, qout;
	quatfromaxisangle(q0, a0, norm3d(a0));
	quatfromaxisangle(q1, a1, norm3d(a1));

	quatrotateabout(qout, q0, q1);
	FLT mag;
	axisanglefromquat(&mag, out, qout);
	scale3d(out, out, mag);
}
void quatrotateabout(LinmathQuat qout, const LinmathQuat q1, const LinmathQuat q2)
{
	// NOTE: Does not normalize
	LinmathQuat rtn;
	FLT *p = qout;
	bool aliased = q1 == qout || q2 == qout;
	if (aliased)
	{
		p = rtn;
	}

	p[0] = (q1[0] * q2[0]) - (q1[1] * q2[1]) - (q1[2] * q2[2]) - (q1[3] * q2[3]);
	p[1] = (q1[0] * q2[1]) + (q1[1] * q2[0]) + (q1[2] * q2[3]) - (q1[3] * q2[2]);
	p[2] = (q1[0] * q2[2]) - (q1[1] * q2[3]) + (q1[2] * q2[0]) + (q1[3] * q2[1]);
	p[3] = (q1[0] * q2[3]) + (q1[1] * q2[2]) - (q1[2] * q2[1]) + (q1[3] * q2[0]);

	if (aliased)
	{
		quatcopy(qout, rtn);
	}

	// for (int i = 0; i < 4; i++)
	// assert(!isnan(qout[i]));
}

void findnearestaxisanglemag(LinmathAxisAngleMag out, const LinmathAxisAngleMag inc,
							 const LinmathAxisAngleMag match)
{
	FLT norm_match = match ? norm3d(match) : 0;
	FLT norm_inc = norm3d(inc);
	FLT new_norm_inc = norm_inc;

	while (new_norm_inc > norm_match + M_PI)
	{
		new_norm_inc -= 2 * M_PI;
	}
	while (new_norm_inc + M_PI < norm_match)
	{
		new_norm_inc += 2 * M_PI;
	}
	scale3d(out, inc, new_norm_inc / norm_inc);
}

void quattoaxisanglemag(LinmathAxisAngleMag ang, const LinmathQuat q)
{
	FLT axis_len = sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	FLT angle = 2. * atan2(axis_len, q[0]);
	scale3d(ang, q + 1, axis_len == 0 ? 0 : angle / axis_len);
}

void quatmultiplyrotation(LinmathQuat q, const LinmathQuat qv, FLT t)
{
	LinmathAxisAngleMag aa;
	quattoaxisanglemag(aa, qv);
	scale3d(aa, aa, t);
	quatfromaxisanglemag(q, aa);
}

void quatscale(LinmathQuat qout, const LinmathQuat qin, FLT s)
{
	qout[0] = qin[0] * s;
	qout[1] = qin[1] * s;
	qout[2] = qin[2] * s;
	qout[3] = qin[3] * s;
}

void quatdivs(LinmathQuat qout, const LinmathQuat qin, FLT s)
{
	qout[0] = qin[0] / s;
	qout[1] = qin[1] / s;
	qout[2] = qin[2] / s;
	qout[3] = qin[3] / s;
}

FLT quatinnerproduct(const LinmathQuat qa, const LinmathQuat qb)
{
	return (qa[0] * qb[0]) + (qa[1] * qb[1]) + (qa[2] * qb[2]) + (qa[3] * qb[3]);
}

void quatouterproduct(FLT *outvec3, LinmathQuat qa, LinmathQuat qb)
{
	outvec3[0] = (qa[0] * qb[1]) - (qa[1] * qb[0]) - (qa[2] * qb[3]) + (qa[3] * qb[2]);
	outvec3[1] = (qa[0] * qb[2]) + (qa[1] * qb[3]) - (qa[2] * qb[0]) - (qa[3] * qb[1]);
	outvec3[2] = (qa[0] * qb[3]) - (qa[1] * qb[2]) + (qa[2] * qb[1]) - (qa[3] * qb[0]);
}

void quatevenproduct(LinmathQuat q, LinmathQuat qa, LinmathQuat qb)
{
	q[0] = (qa[0] * qb[0]) - (qa[1] * qb[1]) - (qa[2] * qb[2]) - (qa[3] * qb[3]);
	q[1] = (qa[0] * qb[1]) + (qa[1] * qb[0]);
	q[2] = (qa[0] * qb[2]) + (qa[2] * qb[0]);
	q[3] = (qa[0] * qb[3]) + (qa[3] * qb[0]);
}

void quatoddproduct(FLT *outvec3, LinmathQuat qa, LinmathQuat qb)
{
	outvec3[0] = (qa[2] * qb[3]) - (qa[3] * qb[2]);
	outvec3[1] = (qa[3] * qb[1]) - (qa[1] * qb[3]);
	outvec3[2] = (qa[1] * qb[2]) - (qa[2] * qb[1]);
}

void quatslerp(LinmathQuat q, const LinmathQuat qa, const LinmathQuat qb, FLT t)
{
	FLT an[4];
	FLT bn[4];
	quatnormalize(an, qa);
	quatnormalize(bn, qb);

	// Switched implementation to match https://en.wikipedia.org/wiki/Slerp

	// Compute the cosine of the angle between the two vectors.
	double dot = dot3d(qa, qb);

	LinmathQuat nqb;

	// If the dot product is negative, slerp won't take
	// the shorter path. Note that v1 and -v1 are equivalent when
	// the negation is applied to all four components. Fix by
	// reversing one quaternion.
	if (dot < 0.0f)
	{
		quatscale(nqb, qb, -1);
		dot = -dot;
	}
	else
	{
		quatscale(nqb, qb, 1);
	}

	const double DOT_THRESHOLD = 0.9995;
	if (dot > DOT_THRESHOLD)
	{
		// If the inputs are too close for comfort, linearly interpolate
		// and normalize the result.

		//Quaternion result = v0 + t*(v1 - v0);
		LinmathQuat tmp;
		quatsub(tmp, nqb, qa);
		quatscale(tmp, tmp, t);
		quatadd(q, qa, tmp);
		quatnormalize(q, q);
		return;
	}

	// Since dot is in range [0, DOT_THRESHOLD], acos is safe
	double theta_0 = FLT_ACOS(dot);		   // theta_0 = angle between input vectors
	double theta = theta_0 * t;			   // theta = angle between v0 and result
	double sin_theta = FLT_SIN(theta);	   // compute this value only once
	double sin_theta_0 = FLT_SIN(theta_0); // compute this value only once

	double s0 = cos(theta) - dot * sin_theta / sin_theta_0; // == sin(theta_0 - theta) / sin(theta_0)
	double s1 = sin_theta / sin_theta_0;

	LinmathQuat aside;
	LinmathQuat bside;
	quatscale(bside, nqb, s1);
	quatscale(aside, qa, s0);
	quatadd(q, aside, bside);
	quatnormalize(q, q);
}

void eulerrotatevector(FLT *vec3out, const LinmathEulerAngle eulerAngle, const FLT *vec3in)
{
	LinmathQuat q;
	quatfromeuler(q, eulerAngle);
	quatrotatevector(vec3out, q, vec3in);
}
void quatrotatevector(FLT *vec3out, const LinmathQuat quat, const FLT *vec3in)
{
	// See: http://www.geeks3d.com/20141201/how-to-rotate-a-vertex-by-a-quaternion-in-glsl/

	FLT tmp[3];
	FLT tmp2[3];
	cross3d(tmp, &quat[1], vec3in);
	tmp[0] += vec3in[0] * quat[0];
	tmp[1] += vec3in[1] * quat[0];
	tmp[2] += vec3in[2] * quat[0];
	cross3d(tmp2, &quat[1], tmp);
	vec3out[0] = vec3in[0] + 2 * tmp2[0];
	vec3out[1] = vec3in[1] + 2 * tmp2[1];
	vec3out[2] = vec3in[2] + 2 * tmp2[2];
}

// Matrix Stuff

Matrix3x3 inverseM33(const Matrix3x3 mat)
{
	Matrix3x3 newMat;
	for (int a = 0; a < 3; a++)
	{
		for (int b = 0; b < 3; b++)
		{
			newMat.val[a][b] = mat.val[a][b];
		}
	}

	for (int i = 0; i < 3; i++)
	{
		for (int j = i + 1; j < 3; j++)
		{
			FLT tmp = newMat.val[i][j];
			newMat.val[i][j] = newMat.val[j][i];
			newMat.val[j][i] = tmp;
		}
	}

	return newMat;
}

void rotation_between_vecs_to_m3(Matrix3x3 *m, const FLT v1[3], const FLT v2[3])
{
	FLT q[4];

	quatfrom2vectors(q, v1, v2);

	quattomatrix33(&(m->val[0][0]), q);
}

void rotate_vec(FLT *out, const FLT *in, Matrix3x3 rot)
{
	out[0] = rot.val[0][0] * in[0] + rot.val[1][0] * in[1] + rot.val[2][0] * in[2];
	out[1] = rot.val[0][1] * in[0] + rot.val[1][1] * in[1] + rot.val[2][1] * in[2];
	out[2] = rot.val[0][2] * in[0] + rot.val[1][2] * in[1] + rot.val[2][2] * in[2];

	return;
}

// This function based on code from Object-oriented Graphics Rendering Engine
// Copyright(c) 2000 - 2012 Torus Knot Software Ltd
// under MIT license
// http://www.ogre3d.org/docs/api/1.9/_ogre_vector3_8h_source.html

void eulerfrom2vectors(LinmathEulerAngle euler, const FLT *src, const FLT *dest)
{
	LinmathQuat q;
	quatfrom2vectors(q, src, dest);
	quattoeuler(euler, q);
}
/** Gets the shortest arc quaternion to rotate this vector to the destination
vector.
@remarks
If you call this with a dest vector that is close to the inverse
of this vector, we will rotate 180 degrees around a generated axis if
since in this case ANY axis of rotation is valid.
*/
void quatfrom2vectors(FLT *q, const FLT *src, const FLT *dest)
{
	// Based on Stan Melax's article in Game Programming Gems

	// Copy, since cannot modify local
	FLT v0[3];
	FLT v1[3];
	normalize3d(v0, src);
	normalize3d(v1, dest);

	FLT d = dot3d(v0, v1); // v0.dotProduct(v1);
	// If dot == 1, vectors are the same
	if (d >= 1.0f)
	{
		quatsetnone(q);
		return;
	}
	if (d < (1e-6f - 1.0f))
	{
		// Generate an axis
		FLT unitX[3] = {1, 0, 0};
		FLT unitY[3] = {0, 1, 0};

		FLT axis[3];
		cross3d(axis, unitX, src);												  // pick an angle
		if ((axis[0] < 1.0e-35f) && (axis[1] < 1.0e-35f) && (axis[2] < 1.0e-35f)) // pick another if colinear
		{
			cross3d(axis, unitY, src);
		}
		normalize3d(axis, axis);
		quatfromaxisangle(q, axis, LINMATHPI);
	}
	else
	{
		FLT s = FLT_SQRT((1 + d) * 2);
		FLT invs = 1 / s;

		FLT c[3];
		cross3d(c, v0, v1);

		q[0] = s * 0.5f;
		q[1] = c[0] * invs;
		q[2] = c[1] * invs;
		q[3] = c[2] * invs;

		quatnormalize(q, q);
	}
}

void matrix44copy(FLT *mout, const FLT *minm) { memcpy(mout, minm, sizeof(FLT) * 16); }

void matrix44transpose(FLT *mout, const FLT *minm)
{
	mout[0] = minm[0];
	mout[1] = minm[4];
	mout[2] = minm[8];
	mout[3] = minm[12];

	mout[4] = minm[1];
	mout[5] = minm[5];
	mout[6] = minm[9];
	mout[7] = minm[13];

	mout[8] = minm[2];
	mout[9] = minm[6];
	mout[10] = minm[10];
	mout[11] = minm[14];

	mout[12] = minm[3];
	mout[13] = minm[7];
	mout[14] = minm[11];
	mout[15] = minm[15];
}

void ApplyPoseToPoint(LinmathPoint3d pout, const LinmathPose *pose, const LinmathPoint3d pin)
{
	LinmathPoint3d tmp;
	quatrotatevector(tmp, pose->Rot, pin);
	add3d(pout, tmp, pose->Pos);
	// for (int i = 0; i < 3; i++)
	// assert(!isnan(pout[i]));
}

void ApplyPoseToPose(LinmathPose *pout, const LinmathPose *lhs_pose, const LinmathPose *rhs_pose)
{
	ApplyPoseToPoint(pout->Pos, lhs_pose, rhs_pose->Pos);
	quatrotateabout(pout->Rot, lhs_pose->Rot, rhs_pose->Rot);
	// for (int i = 0; i < 3; i++)
	// assert(!isnan(pout->Pos[i]));
}

void ApplyAxisAnglePoseToPoint(LinmathPoint3d pout, const LinmathAxisAnglePose *pose, const LinmathPoint3d pin)
{
	LinmathPoint3d tmp;
	axisanglerotatevector(tmp, pose->AxisAngleRot, pin);
	add3d(pout, tmp, pose->Pos);
	// for (int i = 0; i < 3; i++)
	// assert(isfinite(pout[i]));
}
void ApplyAxisAnglePoseToPose(LinmathAxisAnglePose *pout, const LinmathAxisAnglePose *lhs_pose,
							  const LinmathAxisAnglePose *rhs_pose)
{
	ApplyAxisAnglePoseToPoint(pout->Pos, lhs_pose, rhs_pose->Pos);

	axisanglerotateabout(pout->AxisAngleRot, lhs_pose->AxisAngleRot, rhs_pose->AxisAngleRot);
	// quatrotateabout(pout->Rot, lhs_pose->Rot, rhs_pose->Rot);
	// for (int i = 0; i < 3; i++)
	// assert(!isnan(pout->Pos[i]));
}

void InvertPose(LinmathPose *poseout, const LinmathPose *pose)
{
	quatgetreciprocal(poseout->Rot, pose->Rot);

	quatrotatevector(poseout->Pos, poseout->Rot, pose->Pos);
	scale3d(poseout->Pos, poseout->Pos, -1);
}

void PoseToMatrix(FLT *matrix44, const LinmathPose *pose_in)
{
	quattomatrix(matrix44, pose_in->Rot);

	/*
	matrix44[12] = pose_in->Pos[0];
	matrix44[13] = pose_in->Pos[1];
	matrix44[14] = pose_in->Pos[2];
	*/
	matrix44[3] = pose_in->Pos[0];
	matrix44[7] = pose_in->Pos[1];
	matrix44[11] = pose_in->Pos[2];
}

LINMATH_EXPORT LinmathQuat LinmathQuat_Identity = {1.0};
LINMATH_EXPORT LinmathPose LinmathPose_Identity = {.Rot = {1.0}};

FLT linmath_rand(FLT min, FLT max)
{
	FLT r = rand() / (FLT)RAND_MAX;
	r *= (max - min);
	r += min;
	return r;
}
FLT linmath_normrand(FLT mu, FLT sigma)
{
	static const double epsilon = 0.0000001;

	static double z1;
	static bool generate;
	generate = !generate;

	if (!generate)
		return z1 * sigma + mu;

	double u1, u2;
	do
	{
		u1 = rand() * (1.0 / RAND_MAX);
		u2 = rand() * (1.0 / RAND_MAX);
	} while (u1 <= epsilon);

	double z0;
	z0 = sqrt(-2.0 * log(u1)) * cos(LINMATHPI * 2. * u2);
	z1 = sqrt(-2.0 * log(u1)) * sin(LINMATHPI * 2. * u2);
	return z0 * sigma + mu;
}

#pragma GCC push_options
#pragma GCC optimize("O3")
#define RM_IDX(row, col, cols) (col + (row * cols))

#ifdef _MSC_VER
#define RESTRICT_KEYWORD
#else
#define RESTRICT_KEYWORD restrict
#endif

#pragma GCC pop_options

void linmath_get_line_dir(LinmathPoint3d dir, const struct LinmathLine3d *ray)
{
	sub3d(dir, ray->b, ray->a);
	normalize3d(dir, dir);
}

void linmath_pt_along_line(LinmathPoint3d pt, const struct LinmathLine3d *ray, FLT t)
{
	LinmathPoint3d rayDirection;
	linmath_get_line_dir(rayDirection, ray);
	scale3d(pt, rayDirection, t);
	add3d(pt, ray->a, pt);
}

FLT linmath_point_distance_from_line(struct LinmathLine3d *ray, LinmathVec3d pt, FLT *t)
{
	LinmathVec3d pt_a, pt_b, b_a;
	sub3d(pt_a, pt, ray->a);
	sub3d(pt_b, pt, ray->b);
	sub3d(b_a, ray->b, ray->a);

	LinmathVec3d pt_cross;
	cross3d(pt_cross, pt_a, pt_b);

	if (t)
	{
		LinmathVec3d a_pt;
		sub3d(a_pt, ray->a, pt);

		*t = -dot3d(a_pt, b_a) / norm3d(b_a) / norm3d(b_a);
	}
	return norm3d(pt_cross) / norm3d(b_a);
}
//--------------------------------------------------------------------------------------------------
/**
@file	q3Matrix.h
@author	Randy Gaul
@date	10/10/2014
	Copyright (c) 2014 Randy Gaul http://www.randygaul.net
	This software is provided 'as-is', without any express or implied
	warranty. In no event will the authors be held liable for any damages
	arising from the use of this software.
	Permission is granted to anyone to use this software for any purpose,
	including commercial applications, and to alter it and redistribute it
	freely, subject to the following restrictions:
	  1. The origin of this software must not be misrepresented; you must not
	     claim that you wrote the original software. If you use this software
	     in a product, an acknowledgment in the product documentation would be
	     appreciated but is not required.
	  2. Altered source versions must be plainly marked as such, and must not
	     be misrepresented as being the original software.
	  3. This notice may not be removed or altered from any source distribution.
*/
/*
Ported to CSharp by Madhawa Vidanapathirana 
https://github.com/madhawav
*/
//--------------------------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Qu3ECSharp.Math
{
    public struct Matrix3
    {
        Vector3 _x;
        Vector3 _y;
        Vector3 _z;

        public Matrix3(float a, float b, float c, float d, float e, float f, float g, float h, float i)
        {
            this._x = new Vector3(a, b, c);
            this._y = new Vector3(d, e, f);
            this._z = new Vector3(g, h, i);

        }

        public Matrix3(Vector3 x, Vector3 y, Vector3 z)
        {
            this._x = x;
            this._y = y;
            this._z = z;
        }

        public void Set(float a, float b, float c, float d, float e, float f, float g, float h, float i)
        {
            this._x = new Vector3(a, b, c);
            this._y = new Vector3(d, e, f);
            this._z = new Vector3(g, h, i);
        }

        public void Set(Vector3 axis, float angle)
        {
            float s = (float)System.Math.Sin(angle);
            float c = (float)System.Math.Cos(angle);
            float x = axis.X;
            float y = axis.Y;
            float z = axis.Z;

            float xy = x * y;
            float yz = y * z;
            float zx = z * x;

            float t = 1.0f - c;

            this.Set(x * x * t + c, xy * t + z * s, zx * t - y * s,
                    xy * t - z * s, y * y * t + c, yz * t + x * s,
                    zx * t + y * s, yz * t - x * s, z * z * t + c
                    );

        }

        public void SetRows(Vector3 x, Vector3 y, Vector3 z)
        {
            this._x = x;
            this._y = y;
            this._z = z;
        }

        public Vector3 this[int i]
        {
            get
            {
                if (i == 0)
                    return _x;
                else if (i == 1)
                    return _y;
                else if (i == 2)
                    return _z;
                throw new IndexOutOfRangeException("Index should be between 0 and 2");
            }
            set
            {
                if (i == 0)
                    _x = value;
                else if (i == 1)
                    _y = value;
                else if (i == 2)
                    _z = value;
                throw new IndexOutOfRangeException("Index should be between 0 and 2");
            }
        }

        public Vector3 Column0()
        {
            return new Vector3(_x.X, _y.X, _z.X);
        }

        public Vector3 Column1()
        {
            return new Vector3(_x.Y, _y.Y, _z.Y);
        }

        public Vector3 Column2()
        {
            return new Vector3(_x.Z, _y.Z, _z.Z);
        }


        public static Vector3 operator *(Matrix3 v1, Vector3 v2)
        {
            return new Vector3(
            v1._x.X * v2.X + v1._y.X * v2.Y + v1._z.X * v2.Z,
            v1._x.Y * v2.X + v1._y.Y * v2.Y + v1._z.Y * v2.Z,
            v1._x.Z * v2.X + v1._y.Z * v2.Y + v1._z.Z * v2.Z
            );

        }

        public static Matrix3 operator *(Matrix3 v1, Matrix3 v2)
        {
            return new Matrix3(
                v1 * v2._x,
                v1 * v2._y,
                v1 * v2._z
            );

        }

        public static Matrix3 operator *(Matrix3 v1, float v2)
        {
            return new Matrix3(
                v1._x * v2, v1._y * v2, v1._z * v2
            );

        }

        public static Matrix3 operator +(Matrix3 v1, Matrix3 v2)
        {
            return new Matrix3(
                v1._x + v2._x, v1._y + v2._y, v1._z + v2._z
            );


        }

        public static Matrix3 operator -(Matrix3 v1, Matrix3 v2)
        {
            return new Matrix3(
                v1._x - v2._x, v1._y - v2._y, v1._z - v2._z
            );


        }



        public Vector3 eX
        {
            get { return _x; }
            set { _x = value; }
        }

        public Vector3 eY
        {
            get { return _y; }
            set { _y = value; }
        }

        public Vector3 eZ
        {
            get { return _z; }
            set { _z = value; }
        }


        public static Matrix3 Transpose(Matrix3 m)
        {
            return new Matrix3(
                m.eX.X, m.eY.X, m.eZ.X,
                m.eX.Y, m.eY.Y, m.eZ.Y,
                m.eX.Z, m.eY.Z, m.eZ.Z
                );
        }


        public static Matrix3 Identity
        {
            get
            {
                return new Matrix3(
                    1.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f,
                    0.0f, 0.0f, 1.0f
                    );
            }
        }

        public static Matrix3 Rotate(Vector3 x, Vector3 y, Vector3 z)
        {
            return new Matrix3(x, y, z);
        }

        public static Matrix3 Zeros()
        {
            Matrix3 r = new Matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0);
            return r;
        }

        public static Matrix3 Diagonal(float a)
        {
            return new Matrix3(
                a, 0, 0,
                0, a, 0,
                0, 0, a
                );

        }

        public static Matrix3 Diagonal(float a, float b, float c)
        {
            return new Matrix3(
                a, 0, 0,
                0, b, 0,
                0, 0, c
                );

        }


        public static Matrix3 OuterProduct(Vector3 u, Vector3 v)
        {
            Vector3 a = v * u.X;
            Vector3 b = v * u.Y;
            Vector3 c = v * u.Z;

            return new Matrix3(
                a.X, a.Y, a.Z,
                b.X, b.Y, b.Z,
                c.X, c.Y, c.Z
                );

        }


        public static Matrix3 Covariance(Vector3[] points)
        {
            float invNumPoints = 1.0f / points.Length;
            Vector3 c = new Vector3(0.0f, 0.0f, 0.0f);

            for (int i = 0; i < points.Length; ++i)
                c += points[i];

            c /= (float)(points.Length);

            float m00, m11, m22, m01, m02, m12;
            m00 = m11 = m22 = m01 = m02 = m12 = 0.0f;

            for (int i = 0; i < points.Length; ++i)
            {
                Vector3 p = points[i] - c;

                m00 += p.X * p.X;
                m11 += p.Y * p.Y;
                m22 += p.Z * p.Z;
                m01 += p.X * p.Y;
                m02 += p.X * p.Z;
                m12 += p.Y * p.Z;
            }

            float m01inv = m01 * invNumPoints;
            float m02inv = m02 * invNumPoints;
            float m12inv = m12 * invNumPoints;

            return new Matrix3(
                m00 * invNumPoints, m01inv, m02inv,
                m01inv, m11 * invNumPoints, m12inv,
                m02inv, m12inv, m22 * invNumPoints
                );
        }

        public static Matrix3 Inverse(Matrix3 m)
        {
            Vector3 tmp0, tmp1, tmp2;
            float detinv;

            tmp0 = Vector3.Cross(m.eY, m.eZ);
            tmp1 = Vector3.Cross(m.eZ, m.eX);
            tmp2 = Vector3.Cross(m.eX, m.eY);

            detinv = 1.0f / Vector3.Dot(m.eZ, tmp2);

            return new Matrix3(
                tmp0.X * detinv, tmp1.X * detinv, tmp2.X * detinv,
                tmp0.Y * detinv, tmp1.Y * detinv, tmp2.Y * detinv,
                tmp0.Z * detinv, tmp1.Z * detinv, tmp2.Z * detinv
                );
        }


    }


}

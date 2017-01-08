//--------------------------------------------------------------------------------------------------
/**
@file	q3Vec3.h
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
    public struct Vector3
    {
        private float _x;
        private float _y;
        private float _z;

        public float X
        {
            get { return _x; }
            set { _x = value; }
        }

        public float Y
        {
            get { return _y; }
            set { _y = value; }
        }

        public float Z
        {
            get { return _z; }
            set { _z = value; }
        }

        public Vector3(float x, float y, float z)
        {
            this._x = x;
            this._y = y;
            this._z = z;
        }

        public void Set(float x, float y, float z)
        {
            this._x = x; this._y = y; this._z = z;
        }
        public void SetAll(float a)
        {
            this._x = this._y = this._z = a;
        }


        public static Vector3 operator +(Vector3 v1, Vector3 v2)
        {
            return new Vector3(v1._x + v2._x, v1._y + v2._y, v1._z + v2._z);
        }

        public static Vector3 operator -(Vector3 v1, Vector3 v2)
        {
            return new Vector3(v1._x - v2._x, v1._y - v2._y, v1._z - v2._z);
        }

        public static Vector3 operator -(Vector3 v1)
        {
            return new Vector3(-v1._x, -v1._y, -v1._z);
        }

        public static Vector3 operator *(Vector3 v1, float v2)
        {
            return new Vector3(v1._x * v2, v1._y * v2, v1._z * v2);
        }

        public static Vector3 operator /(Vector3 v1, float v2)
        {
            return new Vector3(v1._x / v2, v1._y / v2, v1._z / v2);
        }

        public float this[int i]
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
                else throw new IndexOutOfRangeException("Index should be between 0 and 2");
            }
        }

        public static Vector3 operator *(float v1, Vector3 v2)
        {
            return new Vector3(v2._x * v1, v2._y * v1, v2._z * v1);
        }


        public static Vector3 Identity
        {
            get
            {
                return new Vector3(0, 0, 0);
            }
        }

        public static Vector3 Multiply(Vector3 a, Vector3 b)
        {
            return new Vector3(a.X * b.X, a.Y * b.Y, a.Z * b.Z);
        }

        public static float Dot(Vector3 a, Vector3 b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        public static Vector3 Cross(Vector3 a, Vector3 b)
        {
            return new Vector3(
                (a.Y * b.Z) - (b.Y * a.Z),
                (b.X * a.Z) - (a.X * b.Z),
                (a.X * b.Y) - (b.X * a.Y)
                );
        }

        public static float Length(Vector3 v)
        {
            return (float) System.Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
        }

        public static float LengthSq(Vector3 v)
        {
            return v.X * v.X + v.Y * v.Y + v.Z * v.Z;
        }

        public static Vector3 Normalize(Vector3 v)
        {
            float l = Length(v);

            if (l != 0.0)
            {
                float inv = 1.0f / l;
                return v * inv;
            }

            return v;
        }

        public static float Distance(Vector3 a, Vector3 b)
        {
            float xp = a.X - b.X;
            float yp = a.Y - b.Y;
            float zp = a.Z - b.Z;

            return (float)System.Math.Sqrt(xp * xp + yp * yp + zp * zp);
        }

        public static float DistanceSq(Vector3 a, Vector3 b)
        {
            float xp = a.X - b.X;
            float yp = a.Y - b.Y;
            float zp = a.Z - b.Z;

            return (float)(xp * xp + yp * yp + zp * zp);
        }

        public static Vector3 Abs(Vector3 v)
        {
            return new Vector3(Math.Abs(v.X), Math.Abs(v.Y), Math.Abs(v.Z));
        }

        public static Vector3 Min(Vector3 a, Vector3 b)
        {
            return new Vector3(Math.Min(a.X,b.X), Math.Min(a.Y, b.Y), Math.Min(a.Z, b.Z));
        }

        public static Vector3 Max(Vector3 a, Vector3 b)
        {
            return new Vector3(Math.Max(a.X, b.X), Math.Max(a.Y, b.Y), Math.Max(a.Z, b.Z));
        }

        public static float MinPerElem(Vector3 a)
        {
            return Math.Min(a.X, Math.Min(a.Y, a.Z));
        }

        public static float MaxPerElem(Vector3 a)
        {
            return Math.Max(a.X, Math.Max(a.Y, a.Z));
        }

    }
}

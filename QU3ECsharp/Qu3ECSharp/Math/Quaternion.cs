//--------------------------------------------------------------------------------------------------
/**
@file	q3Quaternion.h
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
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Qu3ECSharp.Math
{
    public class Quaternion
    {
        private float _x;
        private float _y;
        private float _z;
        private float _w;

        public Quaternion(float x, float y, float z, float w)
        {
            _x = x; _y = y; _z = z; _w = w;
        }

        public Quaternion()
        {
            _x = _y = _z = _w = 0;
        }

        public Quaternion(Vector3 axis, float radians)
        {
            float halfAngle = 0.5f * radians;
            float s = (float)System.Math.Sin(halfAngle);
            _x = s * axis.X;
            _y = s * axis.Y;
            _z = s * axis.Z;
            _w = (float)System.Math.Cos(halfAngle);
        }

        public void Set(Vector3 axis, float radians)
        {
            float halfAngle = 0.5f * radians;
            float s = (float)System.Math.Sin(halfAngle);
            _x = s * axis.X;
            _y = s * axis.Y;
            _z = s * axis.Z;
            _w = (float)System.Math.Cos(halfAngle);
        }
        public void ToAxisAngle(out Vector3 axis, out float angle)
        {
            Debug.Assert(_w <= 1.0f);
            angle = (float)(2.0f * System.Math.Acos(_w));

            float l = (float)System.Math.Sqrt(1.0f - _w * _w);
            if(l == 0.0f)
            {
                axis = new Vector3(0, 0, 0);
            }
            else
            {
                l = 1.0f / l;
                axis = new Vector3(_x * l, _y * l, _z * l);

            }
        }

        public static Quaternion Normalize(Quaternion q)
        {
	        float x = q._x;
            float y = q._y;
            float z = q._z;
            float w = q._w;

            float d = q._w * q._w + q._x * q._x + q._y * q._y + q._z * q._z;

	        if( d == 0 )
		        w = 1.0f;

            d = 1.0f / (float)System.Math.Sqrt( d );

            if ( d >  1.0e-8)
            {
	            x *= d;
	            y *= d;
	            z *= d;
	            w *= d;
            }

	        return new Quaternion(x, y, z, w );
        }

        public void Intergrate(Vector3 dv, float dt)
        {
            Quaternion q = new Quaternion(dv.X * dt, dv.Y * dt, dv.Z * dt,0.0f);
            q = q * this;

            _x += q._x * 0.5f;
            _y += q._y * 0.5f;
            _z += q._z * 0.5f;
            _w += q._w * 0.5f;

            Quaternion r = Quaternion.Normalize(this);
            _x = r._x; _y = r._y; _z = r._z; _w = r._w;
        }


        public static Quaternion operator * (Quaternion a, Quaternion b)
        {
            return new Quaternion(
                a._w * b._x + a._x * b._w + a._y * b._z - a._z * b._y,
                a._w * b._y + a._y * b._w + a._z * b._x - a._x * b._z,
                a._w * b._z + a._z * b._w + a._x * b._y - a._y * b._x,
                a._w * b._w - a._x * b._x - a._y * b._y - a._z * b._z
                );
        }

        public Matrix3 ToMatrix()
        {
            float qx2 = _x + _x;
            float qy2 = _y + _y;
            float qz2 = _z + _z;
            float qxqx2 = _x * qx2;
            float qxqy2 = _x * qy2;
            float qxqz2 = _x * qz2;
            float qxqw2 = _w * qx2;
            float qyqy2 = _y * qy2;
            float qyqz2 = _y * qz2;
            float qyqw2 = _w * qy2;
            float qzqz2 = _z * qz2;
            float qzqw2 = _w * qz2;

            return new Matrix3(
                new Vector3(1.0f - qyqy2 - qzqz2, qxqy2 + qzqw2, qxqz2 - qyqw2),
                new Vector3(qxqy2 - qzqw2, 1.0f - qxqx2 - qzqz2, qyqz2 + qxqw2),
                new Vector3(qxqz2 + qyqw2, qyqz2 - qxqw2, 1.0f - qxqx2 - qyqy2)
            );
        }

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
        public float W
        {
            get { return _w; }
            set { _w = value; }
        }


    }
}

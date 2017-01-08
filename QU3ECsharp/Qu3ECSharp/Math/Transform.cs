//--------------------------------------------------------------------------------------------------

/*
Ported to CSharp by Madhawa Vidanapathirana 
https://github.com/madhawav
*/
//--------------------------------------------------------------------------------------------------


using Qu3ECSharp.Common;
/**
@file	q3Transform.h
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
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Qu3ECSharp.Math
{
    public class Transform
    {
        private Vector3 _position;
        private Matrix3 _rotation;

        public Transform Clone()
        {
            Transform result = new Transform();
            result.Position = _position;
            result.Rotation = _rotation;
            return result;
        }
        public Vector3 Position
        {
            get { return _position; }
            set { _position = value; }
        }

        public Matrix3 Rotation
        {
            get { return _rotation; }
            set { _rotation = value; }
        }

        public Transform(Vector3 position, Matrix3 rotation)
        {
            this._rotation = rotation;
            this._position = position;
        }

        public Transform()
        {
            _position = new Vector3();
            _rotation = new Matrix3();
        }

        public static Vector3 Multiply(Transform tx, Vector3 v)
        {
            return tx._rotation * v + tx._position;
        }

        public static Vector3 Multiply(Transform tx, Vector3 scale, Vector3 v)
        {
            return tx._rotation * Vector3.Multiply(scale, v) + tx._position;
        }

        public static Vector3 Multiply(Matrix3 r, Vector3 v)
        {
            return r * v;
        }

        public static Matrix3 Multiply(Matrix3 r, Matrix3 v)
        {
            return r * v;
        }

        public static Transform Multiply(Transform t, Transform u)
        {
            Transform v = new Transform();
            v._rotation = Multiply(t._rotation, u._rotation);
            v._position = Multiply(t._rotation, u._position) + t._position;
            return v;
        }

        //TODO: Implement Half Space Multiply

        public static Vector3 MultiplyTranspose(Transform tx, Vector3 v)
        {
            return Matrix3.Transpose(tx.Rotation) * (v - tx.Position);
        }

        public static Vector3 MultiplyTranspose(Matrix3 r, Vector3 v)
        {
            return Matrix3.Transpose(r) * v;
        }

        public static Matrix3 MultiplyTranspose(Matrix3 r, Matrix3 v)
        {
            return Matrix3.Transpose(r) * v;
        }

        public static Transform MultiplyTranspose(Transform t, Transform u)
        {
            Transform v = new Transform();
            v.Rotation = MultiplyTranspose(t.Rotation, u.Rotation);
            v.Position = MultiplyTranspose(t.Rotation, u.Position - t.Position);
            return v;
        }

        public static void Identity(Transform t)
        {
            t.Rotation = Matrix3.Identity;
            t.Position = Vector3.Identity;
        }

        public static HalfSpace Multiply(Transform tx, HalfSpace p)
        {
	        Vector3 origin = p.Origin;
            origin = Multiply(tx, origin);
            Vector3 normal = Multiply(tx.Rotation, p.Normal);

	        return new HalfSpace(normal, Vector3.Dot( origin, normal ));
        }

        public static HalfSpace Multiply(Transform tx, Vector3 scale, HalfSpace p)
        {
            Vector3 origin = p.Origin;
            origin = Multiply(tx, scale, origin);
            Vector3 normal = Multiply(tx.Rotation, p.Normal);

            return new HalfSpace(normal, Vector3.Dot(origin, normal));
        }


    }
}

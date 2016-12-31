/**
@file	Geometry.h
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
//--------------------------------------------------------------------------------------------------

/*
Ported to CSharp by Madhawa Vidanapathirana 
https://github.com/madhawav
*/
//--------------------------------------------------------------------------------------------------

using Qu3ECSharp.Math;

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Qu3ECSharp.Common
{   
    public class Geometry
    {
        private Geometry() { }
        public void ComputeBasis(Vector3 a, out Vector3 b, out Vector3 c)
        {
            if (System.Math.Abs(a.X) >= (0.57735027f))
                b = new Vector3(a.Y, -a.X, 0.0f);
            else
                b = new Vector3(0.0f, a.Z, -a.Y);

            b = Vector3.Normalize(b);
            c = Vector3.Cross(a, b);
        }
    }
    public struct AABB
    {
        Vector3 _min;
        Vector3 _max;

        public AABB(Vector3 min, Vector3 max)
        {
            this._max = max;
            this._min = min;
        }

        public Vector3 Min
        {
            get { return _min; }
            set { _min = value; }
        }

        public Vector3 Max
        {
            get { return _max; }
            set { _max = value; }
        }

        public bool Contains(AABB other)
        {
            return
                Min.X <= other.Min.X &&
                Min.Y <= other.Min.Y &&
                Min.Z <= other.Min.Z &&
                Max.X >= other.Max.X &&
                Max.Y >= other.Max.Y &&
                Max.Z >= other.Max.Z;
        }

        public bool Contains(Vector3 point)
        {
            return
                Min.X <= point.X &&
                Min.Y <= point.Y &&
                Min.Z <= point.Z &&
                Max.X >= point.X &&
                Max.Y >= point.Y &&
                Max.Z >= point.Z;
        }

        public float SurfaceArea
        {
            get
            {
                float x = Max.X - Min.X;
                float y = Max.Y - Min.Y;
                float z = Max.Z - Min.Z;

                return 2.0f * (x * y + x * z + y * z);
            }
        }

        public static AABB Combine(AABB a, AABB b)
        {
	        AABB c = new AABB();
            c.Min = Vector3.Min(a.Min, b.Min );
            c.Max = Vector3.Max(a.Max, b.Max );

	        return c;
        }


        public static bool CollisionAABBtoAABB( AABB a, AABB b )
        {
	        if ( a.Max.X<b.Min.X || a.Min.X> b.Max.X )
		        return false;

	        if ( a.Max.Y<b.Min.Y || a.Min.Y> b.Max.Y )
		        return false;

	        if ( a.Max.Z<b.Min.Z || a.Min.Z> b.Max.Z )
		        return false;

	        return true;
        }


}
    public struct HalfSpace
    {
        private Vector3 _normal;
        private float _distance;
        public HalfSpace( Vector3 normal, float distance)
        {
            this._normal = normal;
            this._distance = distance;
        }

        public void Set(Vector3 a, Vector3 b, Vector3 c)
        {
            _normal = Vector3.Normalize(Vector3.Cross(b - a, c - a));
            _distance = Vector3.Dot(_normal, a);
        }

        public void Set(Vector3 normal, Vector3 point)
        {
            _normal = Vector3.Normalize(normal);
            _distance = Vector3.Dot(normal, point);
        }

        public Vector3 Origin
        {
            get
            {
                return _normal * _distance;
            }
        }

        public float DistanceFrom(Vector3 p)
        {
            return Vector3.Dot(_normal, p) - _distance;
        }
 
        public Vector3 Projected(Vector3 p)
        {
            return p - _normal * DistanceFrom(p);
        }

        public float Distance { get { return _distance; } set { _distance = value; } }
        public Vector3 Normal { get { return _normal; } set { _normal = value; } }
        
    }



    public struct RaycastData
    {
        Vector3 start;   // Beginning point of the ray
        Vector3 dir;     // Direction of the ray (normalized)
        float t;          // Time specifying ray endpoint

        float toi;        // Solved time of impact
        Vector3 normal;  // Surface normal at impact

        public void Set(Vector3 startPoint, Vector3 direction, float endPointTime)
        {
            start = startPoint;
            dir = direction;
            t = endPointTime;
        }

        // Uses toi, start and dir to compute the point at toi. Should
        // only be called after a raycast has been conducted with a
        // return value of true.
        public Vector3 ImpactPoint
        {
            get
            {
                return start + dir * toi;
            }
        }

        public Vector3 Start { get { return start; } set { start = value; } }
        public Vector3 Direction { get { return dir; } set { dir = value; } }
        public float Time { get { return t; } set { t = value; } }
        public float TimeOfImpact { get { return toi; } set { toi = value; } }
        public Vector3 Normal { get { return normal; } set { normal = value; } }
    };




}


//--------------------------------------------------------------------------------------------------

/*
Ported to CSharp by Madhawa Vidanapathirana 
https://github.com/madhawav
*/
//--------------------------------------------------------------------------------------------------


using Qu3ECSharp.Common;
/**
@file	q3Box.h
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
using Qu3ECSharp.Math;

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Qu3ECSharp.Dynamics;

namespace Qu3ECSharp.Collision
{
    public struct MassData
    {
        private Matrix3 _innertia;
        private Vector3 _center;
        private float _mass;
        public Matrix3 Innertia { get { return _innertia; } set { _innertia = value; } }
        public Vector3 Center { get { return _center; } set { _center = value; } }
        public float Mass { get { return _mass; } set { _mass = value; } }

    }

    public class Box
    {
        private Transform _local = new Transform();
        private Vector3 _e = new Vector3(); //extent, as in extent of each OBB axis (Half width/height/length of box)

        private Box _next; //TODO: Change chaining logic here

        private float _friction = 0.0f;
        private float _restitution = 0.0f;
        private float _density = 0.0f;
        private int _broadPhaseNext = 0;

        private object _userData = null;
        private bool _sensor = false;

        private Body _body;

        //TODO: class q3Body* body;

        public object UserData
        {
            get { return _userData; }
            set
            {
                _userData = value;
            }
        }

        public bool Sensor
        {
            get { return _sensor; }
            set { _sensor = value; }
        }

        public Transform LocalTransform { get { return _local; } set { _local = value; } }
        public Vector3 Extent { get { return _e; } set { _e = value; } }
        public Box Next { get { return _next; } set { _next = value; } }

        public float Friction { get { return _friction; } set { _friction = value; } }
        public float Density { get { return _density; } set { _density = value; } }
      
        public float Restitution { get { return _restitution; } set { _restitution = value; } }
        
        public int BroadPhaseNext { get { return _broadPhaseNext; } set { _broadPhaseNext = value; } }

        public Body Body
        {
            get { return _body; }
            set { _body = value; }
        }

        public bool TestPoint(Transform tx, Vector3 p)
        {
            /**
            Test Collision of a Point with Box
            @tx World Matrix
            **/
            Transform world = Transform.Multiply(tx, _local);
            Vector3 p0 = Transform.MultiplyTranspose(world, p);

            for (int i = 0; i < 3; ++i)
            {
                float d = p0[i];
                float ei = _e[i];

                if (d > ei || d < -ei)
                {
                    return false;
                }
            }

            return true;
        }

        public bool Raycast(Transform tx, RaycastData raycast)
        {
            Transform world = Transform.Multiply(tx, _local);
            Vector3 d = Transform.MultiplyTranspose(world.Rotation, raycast.Direction);
            Vector3 p = Transform.MultiplyTranspose(world, raycast.Start);
            float epsilon = (float)(1.0e-8);
            float tmin = 0;
            float tmax = raycast.Time;

            // t = (e[ i ] - p.[ i ]) / d[ i ]
            float t0;
            float t1;
            Vector3 n0=new Vector3();

            for (int i = 0; i < 3; ++i)
            {
                // Check for ray parallel to and outside of AABB
                if (System.Math.Abs(d[i]) < epsilon)
                {
                    // Detect separating axes
                    if (p[i] < -_e[i] || p[i] > _e[i])
                    {
                        return false;
                    }
                }

                else
                {
                    float d0 = 1.0f / d[i];
                    float s = Math.Math.Sign(d[i]);
                    float ei = _e[i] * s;
                    Vector3 n = new Vector3(0, 0, 0);
                    n[i] = -s;

                    t0 = -(ei + p[i]) * d0;
                    t1 = (ei - p[i]) * d0;

                    if (t0 > tmin)
                    {
                        n0 = n;
                        tmin = t0;
                    }

                    tmax = System.Math.Min(tmax, t1);

                    if (tmin > tmax)
                    {
                        return false;
                    }
                }
            }

            raycast.Normal = Transform.Multiply(world.Rotation, n0);
            raycast.TimeOfImpact = tmin;

	        return true;
        }

        public AABB ComputeAABB(Transform tx)
        {
            /*
            Calculate broadphase collision box
            */
            AABB result = new AABB();
            Transform world = Transform.Multiply(tx, _local);
            //Get bounding box
            Vector3[] v = new Vector3 [8] {
                new Vector3( -_e.X, -_e.Y, -_e.Z),
                new Vector3( -_e.X, -_e.Y,  _e.Z ),
                new Vector3( -_e.X,  _e.Y, -_e.Z ),
                new Vector3( -_e.X,  _e.Y,  _e.Z ),
                new Vector3(  _e.X, -_e.Y, -_e.Z ),
                new Vector3(  _e.X, -_e.Y,  _e.Z ),
                new Vector3(  _e.X,  _e.Y, -_e.Z ),
                new Vector3(  _e.X,  _e.Y,  _e.Z )
            };

            //Transform using world coordinates
            for (int i = 0; i < 8; ++i)
                v[i] = Transform.Multiply(world, v[i]);

            Vector3 min = new Vector3(float.MaxValue,float.MaxValue, float.MaxValue);
            Vector3 max = new Vector3(-float.MaxValue, -float.MaxValue, -float.MaxValue );

            //found broadphase bounds
            for (int i = 0; i < 8; ++i)
            {
                min = Vector3.Min(min, v[i]);
                max = Vector3.Max(max, v[i]);
            }

            result.Min = min;
            result.Max = max;
            return result;
        }

        public MassData ComputeMass()
        {
            // Calculate inertia tensor
            float ex2 = 4.0f * _e.X * _e.X;
            float ey2 = 4.0f * _e.Y * _e.Y;
            float ez2 = 4.0f * _e.Z * _e.Z;
            float mass = 8.0f * _e.X * _e.Y * _e.Z * _density;
            float x = (float)(1.0 / 12.0) * mass * (ey2 + ez2);
            float y = (float)(1.0 / 12.0) * mass * (ex2 + ez2);
            float z = (float)(1.0 / 12.0) * mass * (ex2 + ey2);
            Matrix3 I = Matrix3.Diagonal(x, y, z); //This is the initial matrix if box is axis aligned

            // Transform tensor to local space
            I = _local.Rotation * I * Matrix3.Transpose(_local.Rotation);
            Matrix3 identity = Matrix3.Identity;
            I += (identity * Vector3.Dot(_local.Position, _local.Position) - Matrix3.OuterProduct(_local.Position, _local.Position)) * mass;
            MassData md = new MassData();
            md.Center = _local.Position;
            md.Innertia = I;
            md.Mass = mass;
            return md;
        }

    }

    public class BoxDefinition
    {
        private Transform m_tx;
        private Vector3 m_e;

        private float m_friction;
        private float m_restitution;
        private float m_density;
        private bool m_sensor;

        //TODO: friend class q3Body;

        public BoxDefinition()
        {
            m_friction = 0.4f;
            m_restitution = 0.2f;
            m_density = 1.0f;
            m_sensor = false;
        }


        public void Set(Transform tx, Vector3 extents)
        {
            m_tx = tx;
            m_e = extents * 0.5f;
        }
        public float Restitution
        {
            get { return m_restitution; }
            set { m_restitution = value; }
        }

        public float Friction
        {
            get { return m_friction; }
            set { m_friction = value; }
        }
        public float Density
        {
            get { return m_density; }
            set { m_density = value; }
        }

        public bool Sensor
        {
            get { return m_sensor; }
            set { m_sensor = value; }
        }

        public Transform Tx
        {
            get { return m_tx; }
            set { m_tx = value; }
        }

        public Vector3 Extent
        {
            get { return m_e; }
            set { m_e = value; }
        }
    }
}

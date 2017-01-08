//--------------------------------------------------------------------------------------------------
/**
@file	Island.h
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
using Qu3ECSharp.Common;
using Qu3ECSharp.Math;

namespace Qu3ECSharp.Dynamics
{
    public class VelocityState
    {
        Vector3 w = new Vector3();
        Vector3 v = new Vector3();

        public Vector3 W //Angular Velocity
        {
            get { return w; }
            set { w = value; }
        }

        public Vector3 V  // Linear Velocity
        {
            get { return v; }
            set { v = value; }
        }
    };

    public class Island
    {
        public Body[] Bodies
        {
            get { return m_bodies; }
            set { m_bodies = value; }
        }

        public VelocityState[] Velocities
        {
            get { return m_velocities; }
            set { m_velocities = value; }
        }

        public int BodyCapacity
        {
            get { return m_bodyCapacity; }
            set { m_bodyCapacity = value; }
        }

        public int BodyCount
        {
            get { return m_bodyCount; }
            set { m_bodyCount = value; }
        }

        public ContactConstraint[] Contacts
        {
            get { return m_contacts; }
            set { m_contacts = value; }
        }

        public ContactConstraintState[] ContactStates
        {
            get { return m_contactStates; }
            set { m_contactStates = value; }
        }

        public int ContactCount
        {
            get { return m_contactCount; }
            set { m_contactCount = value; }
        }

        public int ContactCapacity
        {
            get { return m_contactCapacity; }
            set { m_contactCapacity = value; }
        }

        public float Dt
        {
            get { return m_dt; }
            set { m_dt = value; }
        }

        public Vector3 Gravity
        {
            get { return m_gravity; }
            set { m_gravity = value; }
        }

        public int Iterations
        {
            get { return m_iterations; }
            set { m_iterations = value; }
        }

        public bool AllowSleep
        {
            get { return m_allowSleep; }
            set { m_allowSleep = value; }
        }

        public bool EnableFriction
        {
            get { return m_enableFriction; }
            set { m_enableFriction = value; }
        }

        private Body[] m_bodies = null;
        VelocityState[] m_velocities = null;
        int m_bodyCapacity = 0;
        int m_bodyCount = 0;

        ContactConstraint[] m_contacts = null;
        ContactConstraintState[] m_contactStates = null;
        int m_contactCount = 0;
        int m_contactCapacity = 0;

        float m_dt = 0;
        Vector3 m_gravity = new Vector3();
        int m_iterations = 0;

        bool m_allowSleep = false;
        bool m_enableFriction = false;

        public void Solve()
        {

            // Apply gravity
            // Integrate velocities and create state buffers, calculate world inertia
            for (int i = 0; i < m_bodyCount; ++i)
            {
                Body body = m_bodies[i];
                VelocityState v = m_velocities[i];

                if ((body.BodyFlags & Body.Flags.eDynamic)!=0)
                {
                    body.ApplyLinearForce(m_gravity * body.GravityScale);

                    // Calculate world space intertia tensor
                    Matrix3 r = body.Transform.Rotation;
                    body.InvInertiaModel = r * body.InvInertiaModel * Matrix3.Transpose(r);
                    

                    // Integrate velocity
                    body.LinearVelocity += (body.Force * body.InverseMass) * m_dt;
                    body.AngularVelocity += (body.InverseInertiaWorld * body.Torque) * m_dt;

                    // From Box2D!
                    // Apply damping.
                    // ODE: dv/dt + c * v = 0
                    // Solution: v(t) = v0 * exp(-c * t)
                    // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
                    // v2 = exp(-c * dt) * v1
                    // Pade approximation:
                    // v2 = v1 * 1 / (1 + c * dt)
                    body.LinearVelocity *= 1.0f / (1.0f + m_dt * body.LinearDamping);
                    body.AngularVelocity *= 1.0f / (1.0f + m_dt * body.AngularDamping);
                }
                v.V = body.LinearVelocity;
                v.W = body.AngularVelocity;
            }

            // Create contact solver, pass in state buffers, create buffers for contacts
            // Initialize velocity constraint for normal + friction and warm start
            ContactSolver contactSolver = new ContactSolver();
            contactSolver.Initialize(this);
            contactSolver.PreSolve(m_dt);

            // Solve contacts
            for (int i = 0; i < m_iterations; ++i)
                contactSolver.Solve();

            contactSolver.ShutDown();

            // Copy back state buffers
            // Integrate positions
            for (int i = 0; i < m_bodyCount; ++i)
            {
                Body body = m_bodies[i];
                VelocityState v = m_velocities[i];

                if ((body.BodyFlags & Body.Flags.eStatic) != 0)
                    continue;
                body.LinearVelocity = v.V;
                body.AngularVelocity = v.W;

                // Integrate position
                body.WorldCenter += body.LinearVelocity * m_dt;
                body.Quaternion.Intergrate(body.AngularVelocity,m_dt);
                body.Quaternion = Quaternion.Normalize(body.Quaternion);
                body.Transform.Rotation = body.Quaternion.ToMatrix();
            }

            if (m_allowSleep)
            {
                // Find minimum sleep time of the entire island
                float minSleepTime = float.MaxValue;
                for (int i = 0; i < m_bodyCount; ++i)
                {
                    Body body = m_bodies[i];

                    if ((body.BodyFlags & Dynamics.Body.Flags.eStatic) != 0)
                        continue;

                   float sqrLinVel = Vector3.Dot(body.LinearVelocity, body.LinearVelocity);
                    float cbAngVel = Vector3.Dot(body.AngularVelocity, body.AngularVelocity);
                    float linTol = Settings.SLEEP_LINEAR;
                   float angTol = Settings.SLEEP_ANGULAR;

                    if (sqrLinVel > linTol || cbAngVel > angTol)
                    {
                        minSleepTime = 0.0f;
                        body.SleepTime = 0.0f;
                    }

                    else
                    {
                        body.SleepTime += m_dt;
                        minSleepTime = (float)System.Math.Min(minSleepTime, body.SleepTime);
                    }
                }

                // Put entire island to sleep so long as the minimum found sleep time
                // is below the threshold. If the minimum sleep time reaches below the
                // sleeping threshold, the entire island will be reformed next step
                // and sleep test will be tried again.
                if (minSleepTime > Settings.SLEEP_TIME)
                {
                    for (int i = 0; i < m_bodyCount; ++i)
                        m_bodies[i].SetToSleep();
                }
            }
        }

        public void Add(Body body)
        {
            Debug.Assert(m_bodyCount < m_bodyCapacity);

            body.IslandIndex = m_bodyCount;

            m_bodies[m_bodyCount++] = body;
        }


        public void Add(ContactConstraint contact)
        {
            Debug.Assert(m_contactCount < m_contactCapacity);

            m_contacts[m_contactCount++] = contact;
        }

        public void Initialize()
        {
            for (int i = 0; i < m_contactCount; ++i)
            {
                ContactConstraint cc = m_contacts[i];

                ContactConstraintState c = m_contactStates[i];

                c.CenterA = cc.BodyA.WorldCenter;
                c.CenterB = cc.BodyB.WorldCenter;
                c.IA = cc.BodyA.InverseInertiaWorld;
                c.IB = cc.BodyB.InverseInertiaWorld;
                c.MA = cc.BodyA.InverseMass;
                c.MB = cc.BodyB.InverseMass;
                c.Restitution = cc.Restitution;
                c.Friction = cc.Friction;
                c.IndexA = cc.BodyB.IslandIndex;
                c.IndexB = cc.BodyB.IslandIndex;
                c.Normal = cc.Manifold.Normal;
                c.TangentVectors[0] = cc.Manifold.TangentVectors[0];
                c.TangentVectors[1] = cc.Manifold.TangentVectors[1];
                c.ContactCount = cc.Manifold.ContactCount;

                for (int j = 0; j < c.ContactCount; ++j)
                {
                    ContactState s = c.Contacts[j];
                    Contact cp = cc.Manifold.Contacts[j];
                    s.Ra = cp.Position - c.CenterA;
                    s.Rb = cp.Position - c.CenterB;
                    s.Penetration = cp.Penetration;
                    s.NormalImpulse = cp.NormalImpulse;
                    s.TangentImpulse[0] = cp.TangentImpulse[0];
                    s.TangentImpulse[1] = cp.TangentImpulse[1];
                }
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------
/**
@file	ContactSolver.h
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
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Qu3ECSharp.Common;
using Qu3ECSharp.Math;

namespace Qu3ECSharp.Dynamics
{
    public class ContactState
    {
        Vector3 ra = new Vector3();                  // Vector from C.O.M to contact position
        Vector3 rb = new Vector3();                  // Vector from C.O.M to contact position
        float penetration = 0;            // Depth of penetration from collision
        float normalImpulse = 0;          // Accumulated normal impulse
        float[] tangentImpulse = new float[2];  // Accumulated friction impulse
        float bias = 0;                   // Restitution + baumgarte
        float normalMass = 0;             // Normal constraint mass
        float[] tangentMass = new float[2];		// Tangent constraint mass

        public Vector3 Ra
        {
            get { return ra; }
            set { ra = value; }
        }

        public Vector3 Rb
        {
            get { return rb; }
            set { rb = value; }
        }

        public float Penetration
        {
            get { return penetration; }
            set { penetration = value; }
        }

        public float NormalImpulse
        {
            get { return normalImpulse; }
            set { normalImpulse = value; }
        }

        public float[] TangentImpulse
        {
            get { return tangentImpulse; }
            set { tangentImpulse = value; }
        }

        public float Bias
        {
            get { return bias; }
            set { bias = value; }
        }

        public float NormalMass
        {
            get { return normalMass; }
            set { normalMass = value; }
        }

        public float[] TangentMass
        {
            get { return tangentMass; }
            set { tangentMass = value; }
        }


    }

    public class ContactConstraintState
    {
        private ContactState[] contacts = new ContactState[8];
        int contactCount = 0;
        Vector3[] tangentVectors = new Vector3[2];   // Tangent vectors
        Vector3 normal = new Vector3();              // From A to B
        Vector3 centerA = new Vector3();
        Vector3 centerB = new Vector3();
        Matrix3 iA = new Matrix3();
        Matrix3 iB = new Matrix3();
        float mA = 0;
        float mB = 0;
        float restitution = 0;
        float friction = 0; 
        int indexA = 0;
        int indexB = 0;


        public ContactState[] Contacts
        {
            get { return contacts; }
            set { contacts = value; }
        }

        public int ContactCount
        {
            get { return contactCount; }
            set { contactCount = value; }
        }

        public Vector3[] TangentVectors
        {
            get { return tangentVectors; }
            set { tangentVectors = value; }
        }

        public Vector3 Normal
        {
            get { return normal; }
            set { normal = value; }
        }

        public Vector3 CenterA
        {
            get { return centerA; }
            set { centerA = value; }
        }

        public Vector3 CenterB
        {
            get { return centerB; }
            set { centerB = value; }
        }

        public Matrix3 IA
        {
            get { return iA; }
            set { iA = value; }
        }

        public Matrix3 IB
        {
            get { return iB; }
            set { iB = value; }
        }

        public float MA
        {
            get { return mA; }
            set { mA = value; }
        }

        public float MB
        {
            get { return mB; }
            set { mB = value; }
        }

        public float Restitution
        {
            get { return restitution; }
            set { restitution = value; }
        }

        public float Friction
        {
            get { return friction; }
            set { friction = value; }
        }

        public int IndexA
        {
            get { return indexA; }
            set { indexA = value; }
        }

        public int IndexB
        {
            get { return indexB; }
            set { indexB = value; }
        }
    }
    public class ContactSolver
    {
        private Island m_island = null;
       
        ContactConstraintState[] m_contacts = null;
        int m_contactCount = 0;
        //q3VelocityState* m_velocities;
        VelocityState[] m_velocities  = null;
        bool m_enableFriction = false;

        //TODO: Incomplete!

        public void Initialize(Island island)
        {
            m_island = island;
            m_contactCount = island.ContactCount;
            m_contacts = island.ContactStates;
            m_velocities = m_island.Velocities;
            m_enableFriction = island.EnableFriction;
        }

        public void PreSolve(float dt)
        {
            for (int i = 0; i < m_contactCount; ++i)
            {
                ContactConstraintState cs = m_contacts[i];

                Vector3 vA = m_velocities[cs.IndexA].V;
                Vector3 wA = m_velocities[cs.IndexA].W;
                Vector3 vB = m_velocities[cs.IndexB].V;
                Vector3 wB = m_velocities[cs.IndexB].W;

                for (int j = 0; j < cs.ContactCount; ++j)
                {
                    ContactState c = cs.Contacts[j];

                    // Precalculate JM^-1JT for contact and friction constraints
                    Vector3 raCn = Vector3.Cross(c.Ra, cs.Normal);
                    Vector3 rbCn = Vector3.Cross(c.Rb, cs.Normal);
                    float nm = cs.MA + cs.MB;
                    float[] tm = new float[2];
                    tm[0] = nm;
                    tm[1] = nm;

                    nm += Vector3.Dot(raCn, cs.IA * raCn) + Vector3.Dot(rbCn, cs.IB * rbCn);
                    c.NormalMass = Math.Math.Invert(nm);

                    for (int ii = 0; ii < 2; ++ii)
                    {
                        Vector3 raCt = Vector3.Cross(cs.TangentVectors[ii], c.Ra);
                        Vector3 rbCt = Vector3.Cross(cs.TangentVectors[ii], c.Rb);
                        tm[ii] += Vector3.Dot(raCt, cs.IA * raCt) + Vector3.Dot(rbCt, cs.IB * rbCt);
                        c.TangentMass[ii] = Math.Math.Invert(tm[ii]);
                    }

                    // Precalculate bias factor
                    c.Bias = -Settings.BAUMGARTE * (1.0f / dt) * System.Math.Min(0.0f, c.Penetration + Settings.PENETRATION_SLOP);

                    // Warm start contact
                    Vector3 P = cs.Normal * c.NormalImpulse;

                    if (m_enableFriction)
                    {
                        P += cs.TangentVectors[0] * c.TangentImpulse[0];
                        P += cs.TangentVectors[1] * c.TangentImpulse[1];
                    }

                    vA -= P * cs.MA;
                    wA -= cs.IA * Vector3.Cross(c.Ra, P);

                    vB += P * cs.MB;
                    wB += cs.IB * Vector3.Cross(c.Rb, P);

                    // Add in restitution bias
                    float dv = Vector3.Dot(vB + Vector3.Cross(wB, c.Rb) - vA - Vector3.Cross(wA, c.Ra), cs.Normal);

                    if (dv < -(1.0f))
                        c.Bias += -(cs.Restitution) * dv;
                }

                m_velocities[cs.IndexA].V = vA;
                m_velocities[cs.IndexA].W = wA;
                m_velocities[cs.IndexB].V = vB;
                m_velocities[cs.IndexB].W = wB;
            }
        }

        public void Solve()
        {
            for (int i = 0; i < m_contactCount; ++i)
            {
               ContactConstraintState cs = m_contacts[i];

                Vector3 vA = m_velocities[cs.IndexA].V;
                Vector3 wA = m_velocities[cs.IndexA].W;
                Vector3 vB = m_velocities[cs.IndexB].V;
                Vector3 wB = m_velocities[cs.IndexB].W;

                for (int j = 0; j < cs.ContactCount; ++j)
                {
                    ContactState c = cs.Contacts[j];

                    // relative velocity at contact
                    Vector3 dv = vB + Vector3.Cross(wB, c.Rb) - vA - Vector3.Cross(wA, c.Ra);

                    // Friction
                    if (m_enableFriction)
                    {
                        for (int ii = 0; ii < 2; ++ii)
                        {
                            float lambda = -Vector3.Dot(dv, cs.TangentVectors[ii]) * c.TangentMass[ii];

                            // Calculate frictional impulse
                            float maxLambda = cs.Friction * c.NormalImpulse;

                            // Clamp frictional impulse
                            float oldPT = c.TangentImpulse[ii];
                            c.TangentImpulse[ii] = Math.Math.Clamp(-maxLambda, maxLambda, oldPT + lambda);
                            lambda = c.TangentImpulse[ii] - oldPT;

                            // Apply friction impulse
                            Vector3 impulse = cs.TangentVectors[ii] * lambda;
                            vA -= impulse * cs.MA;
                            wA -= cs.IA * Vector3.Cross(c.Ra, impulse);

                            vB += impulse * cs.MB;
                            wB += cs.IB * Vector3.Cross(c.Rb, impulse);
                        }
                    }

                    // Normal
                    {
                        dv = vB + Vector3.Cross(wB, c.Rb) - vA - Vector3.Cross(wA, c.Ra);

                        // Normal impulse
                        float vn = Vector3.Dot(dv, cs.Normal);

                        // Factor in positional bias to calculate impulse scalar j
                        float lambda = c.NormalMass * (-vn + c.Bias);

                        // Clamp impulse
                        float tempPN = c.NormalImpulse;
                        c.NormalImpulse = (float)System.Math.Max(tempPN + lambda, 0.0f);
                        lambda = c.NormalImpulse - tempPN;

                        // Apply impulse
                        Vector3 impulse = cs.Normal * lambda;
                        vA -= impulse * cs.MA;
                        wA -= cs.IA * Vector3.Cross(c.Ra, impulse);

                        vB += impulse * cs.MB;
                        wB += cs.IB * Vector3.Cross(c.Rb, impulse);
                    }
                }

                m_velocities[cs.IndexA].V = vA;
                m_velocities[cs.IndexA].W = wA;
                m_velocities[cs.IndexB].V = vB;
                m_velocities[cs.IndexB].W = wB;
            }
        }

        public void ShutDown()
        {
            for (int i = 0; i < m_contactCount; ++i)
            {
                ContactConstraintState c = m_contacts[i];
                ContactConstraint cc = m_island.Contacts[i];

                for (int j = 0; j < c.ContactCount; ++j)
                {
                    Contact oc = cc.Manifold.Contacts[j];
                    ContactState cs = c.Contacts[j];
                    oc.NormalImpulse = cs.NormalImpulse;
                    oc.TangentImpulse[0] = cs.TangentImpulse[0];
                    oc.TangentImpulse[1] = cs.TangentImpulse[1];
                }
            }
        }
    }
}

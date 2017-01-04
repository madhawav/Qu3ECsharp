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
        //q3Island* m_island;
        ContactConstraintState[] m_contacts = null;
        int m_contactCount = 0;
        //q3VelocityState* m_velocities;

        bool m_enableFriction = false;

        //TODO: Incomplete!

        public void Initialize(Island island)
        {
            throw new NotImplementedException();
        }
    }
}

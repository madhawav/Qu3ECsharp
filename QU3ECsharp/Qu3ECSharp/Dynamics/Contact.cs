﻿//--------------------------------------------------------------------------------------------------

/*
Ported to CSharp by Madhawa Vidanapathirana 
https://github.com/madhawav
*/
//--------------------------------------------------------------------------------------------------



using Qu3ECSharp.Collision;
using Qu3ECSharp.Math;
/**
@file	q3Contact.h
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

namespace Qu3ECSharp.Dynamics
{
    public struct FeaturePair
    {
        private byte inR;
        private byte outR;
        private byte inI;
        private byte outI;
        private int key;

        public byte InR { get { return inR; } set { inR = value; } }
        public byte OutR { get { return outR; } set { outR = value; } }
        public byte InI { get { return inI; } set { inI = value; } }
        public byte OutI { get { return OutI; } set { OutI = value; } }
        public int Key { get { return key; } set { key = value; } }
    }

    public class Manifold
    { 
        Box _A = null;
        Box _B = null;

        Vector3 _normal = new Vector3();              // From A to B
        Vector3[] _tangentVectors = new Vector3[2];   // Tangent vectors [Array of 2]
        Contact[] _contacts = new Contact[8]; // [Array of 8]
        int _contactCount = 0;

        Manifold _next = null;
        Manifold _prev = null;

        bool _sensor = false;

        public void SetPair(Box a, Box b)
        {
            this._A = a;
            this._B = b;

            this._sensor = a.Sensor || b.Sensor;
           
        }

        public Box A { get { return _A; } }
        public Box B { get { return _B; } }
        public Vector3 Normal { get { return _normal; } set { _normal = value; } }
        public Vector3[] TangentVectors { get { return _tangentVectors; } set { _tangentVectors = value; } }
        public Contact[] Contacts { get { return _contacts; } set { _contacts = value; } }
        public int ContactsCount { get { return _contactCount; } set { _contactCount = value; } }
        public Manifold Next { get { return _next; } set { _next = value; } }
        public Manifold Previous { get { return _prev; } set { _prev = value; } }
        public bool Sensor { get { return _sensor; } set { _sensor = value; } }

    };

    public class ContactEdge
    {
        private Body _other = null;
        private ContactConstraint _constraint = null;
        private ContactEdge _next = null;
        private ContactEdge _prev = null;

        public Body Other { get { return _other; } set { _other = value; } }
        public ContactConstraint Constraint { get { return _constraint; } set { _constraint = value; } }

        public ContactEdge Next { get { return _next; } set { _next = value; } }
        public ContactEdge Previous { get { return _prev; } set { _prev = value; } }

    };

    public class Contact
    {
        Vector3 _position = new Vector3();            // World coordinate of contact
        float _penetration = 0;            // Depth of penetration from collision
        float _normalImpulse = 0;          // Accumulated normal impulse
        float[] _tangentImpulse = new float[2];  // Accumulated friction impulse
        float _bias = 0.0f;                   // Restitution + baumgarte
        float _normalMass = 0.0f;             // Normal constraint mass
        float[] _tangentMass = new float[2];     // Tangent constraint mass
        FeaturePair _fp = new FeaturePair();           // Features on A and B for this contact
        byte _warmStarted = 0;             // Used for debug rendering

        public Vector3 Position { get { return _position; } set { _position = value; } }
        public float Penetration { get { return _penetration; } set { _penetration = value; } }
        public float NormalImpulse { get { return _normalImpulse; } set { _normalImpulse = value; } }
        public float[] TangentImpulse { get { return _tangentImpulse; } set { _tangentImpulse = value; } }
        public float Bias { get { return _bias; } set { _bias = value; } }
        public float NormalMass { get { return _normalMass; } set { _normalMass = value; } }
        public float[] TangentMass { get { return _tangentMass; } set { _tangentMass = value; } }
        public FeaturePair FeaturePair { get { return _fp; } set { _fp = value; } }
        public byte WarmStarted { get { return _warmStarted ; } set { _warmStarted = value; } }
    }
}

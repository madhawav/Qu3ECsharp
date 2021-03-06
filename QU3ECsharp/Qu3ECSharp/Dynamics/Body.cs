﻿/**
@file	q3Body.h
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



using Qu3ECSharp.Collision;
using Qu3ECSharp.Math;

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Qu3ECSharp.Common;

namespace Qu3ECSharp.Dynamics
{
    public enum BodyType
    {
        StaticBody, DynamicBody, KinematicBody
    }
  
    public class Body
    {
        [Flags]
        public enum Flags
        {
            eAwake = 0x001,
            eActive = 0x002,
            eAllowSleep = 0x004,
            eIsland = 0x010,
            eStatic = 0x020,
            eDynamic = 0x040,
            eKinematic = 0x080,
            eLockAxisX = 0x100,
            eLockAxisY = 0x200,
            eLockAxisZ = 0x400,
        }

        private Matrix3 m_invInertiaModel = new Matrix3();
        private Matrix3 m_invInertiaWorld = new Matrix3();
    
        private float m_mass = 0;
        private float m_invMass = 0;
        private Vector3 m_linearVelocity = new Vector3();
        private Vector3 m_angularVelocity = new Vector3();
        private Vector3 m_force = new Vector3();
        private Vector3 m_torque = new Vector3();
        private Transform m_tx = new Transform();
        private Quaternion m_q = new Quaternion();
        private Vector3 m_localCenter = new Vector3();
        private Vector3 m_worldCenter = new Vector3();
        private float m_sleepTime = 0;
        private float m_gravityScale = 0;
        private int m_layers = 0;
        private Flags m_flags = 0;

        private Box m_boxes = null;
        private object m_userData = null;
        private Scene.Scene m_scene = null ;
        private Body m_next = null;
        private Body m_prev = null;
        private int m_islandIndex = 0;

        private float m_linearDamping = 0;
        private float m_angularDamping = 0;

        private ContactEdge m_contactList = null;


        internal Body(BodyDefinition def, Scene.Scene scene)
        {
            m_linearVelocity = def.LinearVelocity;
            m_angularVelocity = def.AngularVelocity;
            m_force = Vector3.Identity;
            m_torque = Vector3.Identity;
            m_q = new Quaternion();
            m_q.Set(Vector3.Normalize(def.Axis),def.Angle);
            m_tx.Rotation = m_q.ToMatrix();
            m_tx.Position = def.Position;
            m_sleepTime = 0.0f;
            m_gravityScale = def.GravityScale;
            m_layers = def.Layers;
            m_userData = def.UserData;
            m_scene = scene;
            m_flags = 0;
            m_linearDamping = def.LinearDamping;
            m_angularDamping = def.AngularDamping;

            if (def.BodyType == BodyType.DynamicBody)
                m_flags |= Flags.eDynamic;

            else
            {
                if (def.BodyType == BodyType.StaticBody)
                {
                    m_flags |= Flags.eStatic;
                    m_linearVelocity = Vector3.Identity;
                    m_angularVelocity = Vector3.Identity;
                    m_force = Vector3.Identity;
                    m_torque = Vector3.Identity;
                }

                else if (def.BodyType == BodyType.KinematicBody)
                    m_flags |= Flags.eKinematic;
            }

            if (def.AllowSleep)
                m_flags |= Flags.eAllowSleep;

            if (def.Awake)
                m_flags |= Flags.eAwake;

            if (def.Active)
                m_flags |= Flags.eActive;

            if (def.LockAxisX)
                m_flags |= Flags.eLockAxisX;

            if (def.LockAxisY)
                m_flags |= Flags.eLockAxisY;

            if (def.LockAxisZ)
                m_flags |= Flags.eLockAxisZ;

            m_boxes = null;
            m_contactList = null;
        }

        public Transform Transform
        {
            get { return m_tx; }
            set
            {
                m_tx = value;
                SynchronizeProxies();
            }
        }

        public void SetTransform(Vector3 position, Vector3 axis, float angle)
        {
            m_worldCenter = position;
            m_q.Set(axis, angle);
            m_tx.Rotation = m_q.ToMatrix();

            SynchronizeProxies();
        }

        public void SynchronizeProxies()
        {
            //Updates broadphase with new details

            BroadPhase.BroadPhase broadphase = m_scene.ContactManager.Broadphase;
            m_tx.Position = m_worldCenter - Transform.Multiply(m_tx.Rotation, m_localCenter);
           
            AABB aabb = new AABB();
            Transform tx = m_tx.Clone();

            Box box = m_boxes;
            
            while (box != null)
            {
                aabb = box.ComputeAABB(tx);
                broadphase.Update(box.BroadPhaseIndex,aabb);
                box = box.Next;
            }
        }

        public ContactEdge ContactList
        {
            get { return m_contactList; }
            set { m_contactList = value; }
        }

        public Box Boxes
        {
            get { return m_boxes; }
            set { m_boxes = value; }
        }

        public Flags BodyFlags
        {
            get { return m_flags; }
            set { m_flags = value; }
        }

        public float GravityScale
        {
            get { return m_gravityScale; }
            set { m_gravityScale = value; }
        }

        public Matrix3 InvInertiaModel
        {
            get { return m_invInertiaModel; }
            set { m_invInertiaModel = value; }
        }

        public Vector3 LinearVelocity
        {
            get { return m_linearVelocity; }
            set
            {
               
                // Velocity of static bodies cannot be adjusted
                if ((m_flags & Flags.eStatic) != 0)
                    Debug.Assert(false);

                if (Vector3.Dot(value, value) > 0.0f)
                {
                    SetToAwake();
                }

                m_linearVelocity = value;
            }
        }



        public Vector3 AngularVelocity
        {
            get { return m_angularVelocity; }
            set
            {
                // Velocity of static bodies cannot be adjusted
                if ((m_flags & Flags.eStatic)!=0)
                    Debug.Assert(false);

                if (Vector3.Dot(value, value) > 0.0f)
                {
                    SetToAwake();
                }
                m_angularVelocity = value;
            }
        }

        public Vector3 Force
        {
            get { return m_force; }
            set { m_force = value; }
        }

        public Vector3 Torque
        {
            get { return m_torque; }
            set { m_torque = value; }
        }

        public float InverseMass
        {
            get { return m_invMass; }
            set { m_invMass = value; }
        }

        public float Mass
        {
            get { return m_mass; }
            set { m_mass = value; }
        }

        public Matrix3 InverseInertiaWorld
        {
            get { return m_invInertiaWorld; }
            set { m_invInertiaWorld = value; }
        }

        public float AngularDamping
        {
            get { return m_angularDamping; }
            set { m_angularDamping = value; }
        }

        public float LinearDamping
        {
            get { return m_linearDamping; }
            set { m_linearDamping = value; }
        }

        public Vector3 LocalCenter
        {
            get { return m_localCenter; }
            set { m_localCenter = value; }
        }

        public Vector3 WorldCenter
        {
            get { return m_worldCenter; }
            set { m_worldCenter = value; }
        }

        public Quaternion Quaternion
        {
            get { return m_q; }

            set { m_q = value; }
        }


        public float SleepTime { get { return m_sleepTime; } set { m_sleepTime = value; } }
        public int IslandIndex { get {return m_islandIndex;} set { m_islandIndex = value; } }

        public int Layers
        {
            get { return m_layers; }
            set { m_layers = value; }
        }

        public object UserData
        {
            get { return m_userData; }
            set { m_userData = value; }
        }

        public Body Next
        {
            get { return m_next; }
            set { m_next = value; }
        }

        public Body Previous
        {
            get { return m_prev; }
            set { m_prev = value; }
        }


        // Adds a box to this body. Boxes are all defined in local space
        // of their owning body. Boxes cannot be defined relative to one
        // another. The body will recalculate its mass values. No contacts
        // will be created until the next q3Scene::Step( ) call.
        public Box AddBox(BoxDefinition def)
        {
            AABB aabb;
            Box box = new Box();
            box.LocalTransform = def.Tx;
            box.Extent = def.Extent;
            box.Next = m_boxes; //Link to Chain of Elements
            m_boxes = box;
            aabb = box.ComputeAABB(m_tx);

            box.Body = this;
            box.Friction = def.Friction;
            box.Restitution = def.Restitution;
            box.Density = def.Density;
            box.Sensor = def.Sensor;

            CalculateMassData();

            m_scene.ContactManager.Broadphase.InsertBox(box,aabb);
            m_scene.NewBox = true;
        
            return box;
        }

        // Removes this box from the body and broadphase. Forces the body
        // to recompute its mass if the body is dynamic. Frees the memory
        // pointed to by the box pointer.
        public void RemoveBox(Box box)
        {
            Debug.Assert(box != null);
            Debug.Assert(box.Body == this);

            Box node = m_boxes;

            bool found = false;
            if (node == box)
            {
                m_boxes = node.Next;
                found = true;
            }

            else
            {
                while (node != null)
                {
                    if (node.Next == box)
                    {
                        node.Next = box.Next;
                        found = true;
                        break;
                    }

                    node = node.Next;
                }
            }

            // This shape was not connected to this body.
            Debug.Assert(found);

            // Remove all contacts associated with this shape
            ContactEdge edge = m_contactList;
            while (edge != null)
            {
                ContactConstraint contact = edge.Constraint;
                edge = edge.Next;

                Box A = contact.A;
                Box B = contact.B;

                if (box == A || box == B)
                    m_scene.ContactManager.RemoveContact(contact);
            }

            m_scene.ContactManager.Broadphase.RemoveBox(box);

            CalculateMassData();

            box = null;
        }


        // Removes all boxes from this body and the broadphase.
        public void RemoveAllBoxes()
        {
            while (m_boxes != null)
            {
                Box next = m_boxes.Next;

                m_scene.ContactManager.Broadphase.RemoveBox(m_boxes);
                m_boxes = null;

                m_boxes = next;
            }

            m_scene.ContactManager.RemoveContactsFromBody(this);
        }

     

    private void CalculateMassData()
        {
            Matrix3 inertia = Matrix3.Diagonal(0.0f);
            m_invInertiaModel = Matrix3.Diagonal(0.0f);
            m_invInertiaWorld = Matrix3.Diagonal(0.0f);
            m_invMass = 0.0f;
            m_mass = 0.0f;
            float mass = 0.0f;

            if ((m_flags & Flags.eStatic) != 0 ||(m_flags & Flags.eKinematic)!=0)
            {
                m_localCenter = Vector3.Identity;
                m_worldCenter = m_tx.Position;
                return;
            }

            Vector3 lc = Vector3.Identity;
            

            for (Box box = m_boxes; box != null; box = box.Next)
            {
                if (box.Density == 0.0f)
                    continue;

                MassData md = box.ComputeMass();
                mass += md.Mass;
                inertia += md.Innertia;
                lc += md.Center * md.Mass;
            }

            if (mass > 0.0f)
            {
                m_mass = mass;
                m_invMass = 1.0f / mass;
                lc *= m_invMass;
                Matrix3 identity = Matrix3.Identity;
                inertia -= (identity * Vector3.Dot(lc, lc) - Matrix3.OuterProduct(lc, lc)) * mass;
                m_invInertiaModel = Matrix3.Inverse(inertia);

                if ((m_flags & Flags.eLockAxisX) != 0)
                    m_invInertiaModel.eX = Vector3.Identity;

                if ((m_flags & Flags.eLockAxisY) != 0)
                    m_invInertiaModel.eY = Vector3.Identity;

                if ((m_flags & Flags.eLockAxisZ) != 0)
                    m_invInertiaModel.eZ = Vector3.Identity;
            }
            else
            {
                // Force all dynamic bodies to have some mass
                m_invMass = 1.0f;
                m_invInertiaModel = Matrix3.Diagonal(0.0f);
                m_invInertiaWorld = Matrix3.Diagonal(0.0f);
            }

            m_localCenter = lc;
            m_worldCenter = Transform.Multiply(m_tx, lc);
        }

        public bool CanCollide(Body other)
        {
            if (this == other)
                return false;

            // Every collision must have at least one dynamic body involved
            if ((m_flags & Flags.eDynamic) == 0 && (other.m_flags & Flags.eDynamic) == 0)
                return false;

            if ((m_layers & other.m_layers) == 0)
                return false;

            return true;
        }

        public void SetToAwake()
        {
            if ((m_flags & Flags.eAwake)==0)
            {
                m_flags |= Flags.eAwake;
                m_sleepTime = 0.0f;
            }
        }

        public bool IsAwake()
        {
            return (m_flags & Flags.eAwake) != 0 ? true : false;
        }

        public void ApplyLinearForce(Vector3 force)
        {
            m_force += force * m_mass;

            SetToAwake();
        }
        public void ApplyForceAtWorldPoint(Vector3 force, Vector3 point)
        { 
            m_force += force * m_mass;
            m_torque += Vector3.Cross(point - m_worldCenter, force);

            SetToAwake();
        }

        public void ApplyLinearImpulse(Vector3 impulse)
        {
            m_linearVelocity += impulse * m_invMass;

            SetToAwake();
        }

        public void ApplyLinearImpulseAtWorldPoint(Vector3 impulse, Vector3 point)
        {
            m_linearVelocity += impulse * m_invMass;
            m_angularVelocity += m_invInertiaWorld * Vector3.Cross(point - m_worldCenter, impulse);

            SetToAwake();
        }

        public void ApplyTorque(Vector3 torque)
        {
            m_torque += torque;
        }
        public void SetToSleep()
        {
            m_flags &= ~Flags.eAwake;
            m_sleepTime = 0.0f;
            m_linearVelocity = Vector3.Identity;
            m_angularVelocity = Vector3.Identity;
            m_force = Vector3.Identity;
            m_torque = Vector3.Identity;
            
        }

        public Vector3 GetLocalPoint(Vector3 p)
        {
            return Transform.MultiplyTranspose(m_tx, p);
        }

        public Vector3 GetLocalVector(Vector3 v)
        {
            return Transform.MultiplyTranspose(m_tx.Rotation, v);
        }

        public Vector3 GetWorldPoint(Vector3 p)
        {
            return Transform.Multiply(m_tx, p);
        }

        public Vector3 GetWorldVector(Vector3 v)
        {
            return Transform.Multiply(m_tx.Rotation, v);
        }

        public Vector3 GetVelocityAtWorldPoint(Vector3 p)
        {
            Vector3 directionToPoint = p - m_worldCenter;
            Vector3 relativeAngularVel = Vector3.Cross(m_angularVelocity, directionToPoint);

            return m_linearVelocity + relativeAngularVel;
        }


    }

    public class BodyDefinition
    {
        private Vector3 _axis;            // Initial world transformation.
        private float _angle;              // Initial world transformation. Radians.
        private Vector3 _position;        // Initial world transformation.
        private Vector3 _linearVelocity;  // Initial linear velocity in world space.
        private Vector3 _angularVelocity; // Initial angular velocity in world space.
        private float _gravityScale;       // Convenient scale values for gravity x, y and z directions.
        private int _layers;             // Bitmask of collision layers. Bodies matching at least one layer can collide.
        private object _userData;         // Use to store application specific data.

        private float _linearDamping;
        private float _angularDamping;

        // Static, dynamic or kinematic. Dynamic bodies with zero mass are defaulted
       

        // to a mass of 1. Static bodies never move or integrate, and are very CPU
        // efficient. Static bodies have infinite mass. Kinematic bodies have
        // infinite mass, but *do* integrate and move around. Kinematic bodies do not
        // resolve any collisions.
        private BodyType _bodyType;

        private bool _allowSleep;    // Sleeping lets a body assume a non-moving state. Greatly reduces CPU usage.
        private bool _awake;         // Initial sleep state. True means awake.
        private bool _active;        // A body can start out inactive and just sits in memory.
        private bool _lockAxisX;     // Locked rotation on the x axis.
        private bool _lockAxisY;     // Locked rotation on the y axis.
        private bool _lockAxisZ;		// Locked rotation on the z axis.

        public BodyDefinition()
        {
            // Set all initial positions/velocties to zero
            _axis = Vector3.Identity;
            _angle = 0;
            _position = Vector3.Identity;
            _linearVelocity = Vector3.Identity;
            _angularVelocity = Vector3.Identity;
            _gravityScale = 1.0f; // Usually a gravity scale of 1 is the best
            _bodyType = BodyType.StaticBody;
            _layers = 0x000000001;
            _userData = null;
            _allowSleep = true;
            _awake = true;
            _active = true;
            _lockAxisX = false;
            _lockAxisY = false;
            _lockAxisZ = false;
            _linearDamping = 0;
            _angularDamping = 0.1f;
        }

        public Vector3 Axis { get { return _axis; } set { _axis = value; } }            // Initial world transformation.
        public float Angle { get { return _angle; } set { _angle = value; } }           // Initial world transformation. Radians.
        public Vector3 Position { get { return _position; } set { _position = value; } }                   // Initial world transformation.
        public Vector3 LinearVelocity { get { return _linearVelocity; } set { _linearVelocity = value; } }  // Initial linear velocity in world space.
        public Vector3 AngularVelocity { get { return _angularVelocity; } set { _angularVelocity = value; } } // Initial angular velocity in world space.
        public float GravityScale { get { return _gravityScale; } set { _gravityScale = value; } }

        public int Layers
        {
            get { return _layers; }
            set { _layers = value; }
        }

        public object UserData
        {
            get { return _userData; }
            set { _userData = value; }
        }

        public float LinearDamping
        {
            get { return _linearDamping; }
            set { _linearDamping = value; }
        }

        public float AngularDamping
        {
            get { return _angularDamping; }
            set { _angularDamping = value; }
        }

        public BodyType BodyType
        {
            get { return _bodyType; }
            set { _bodyType = value; }
        }

        public bool AllowSleep
        {
            get { return _allowSleep; }
            set { _allowSleep = value; }
        }

        public bool Awake
        {
            get { return _awake; }
            set { _awake = value; }
        }

        public bool Active
        {
            get { return _active; }
            set { _active = value; }
        }

        public bool LockAxisX
        {
            get { return _lockAxisX; }
            set { _lockAxisX = value; }
        }

        public bool LockAxisY
        {
            get { return _lockAxisY; }
            set { _lockAxisY = value; }
        }

        public bool LockAxisZ
        {
            get { return _lockAxisZ; }
            set { _lockAxisZ = value; }
        }
        // Convenient scale values for gravity x, y and z directions.


    }
}
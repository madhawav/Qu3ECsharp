//--------------------------------------------------------------------------------------------------
/**
@file	Scene.h
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
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using Qu3ECSharp.BroadPhase;
using Qu3ECSharp.Collision;
using Qu3ECSharp.Common;
using Qu3ECSharp.Dynamics;
using Qu3ECSharp.Math;

namespace Qu3ECSharp.Scene
{
    class SceneQueryWrapperAABB : IQuerable
    {

        private IQueryCallback cb;
        private BroadPhase.BroadPhase broadPhase;
        private AABB m_aabb;
        public SceneQueryWrapperAABB(BroadPhase.BroadPhase broadPhase, AABB m_aabb, IQueryCallback cb)
        {
            this.broadPhase = broadPhase;
            this.m_aabb = m_aabb;
            this.cb = cb;
        }
        public bool TreeCallBack(int id)
        {
            AABB aabb = new AABB();
            Box box = (Box) broadPhase.Tree.GetUserData(id);

            aabb = box.ComputeAABB(box.Body.Transform);
            
            if (AABB.CollisionAABBtoAABB(m_aabb, aabb))
            {
                return cb.ReportShape(box);
            }

            return true;
        }
    }

    class SceneQueryWrapperRayCast : IQuerable
    {

        private IQueryCallback cb;
        private BroadPhase.BroadPhase broadPhase;
        private RaycastData m_rayCast;
        public SceneQueryWrapperRayCast(BroadPhase.BroadPhase broadPhase, RaycastData raycast, IQueryCallback cb)
        {
            this.broadPhase = broadPhase;
            this.m_rayCast = raycast;
            this.cb = cb;
        }
        public bool TreeCallBack(int id)
        {
            Box box = (Box)broadPhase.Tree.GetUserData(id);

            if (box.Raycast(box.Body.Transform, m_rayCast))
            {
                return cb.ReportShape(box);
            }

            return true;
        }
    }
    class SceneQueryWrapperPoint : IQuerable
    {

        private IQueryCallback cb;
        private BroadPhase.BroadPhase broadPhase;
        private Vector3 m_point;
        public SceneQueryWrapperPoint(BroadPhase.BroadPhase broadPhase, Vector3 m_point, IQueryCallback cb)
        {
            this.broadPhase = broadPhase;
            this.m_point = m_point;
            this.cb = cb;
        }
        public bool TreeCallBack(int id)
        {
            Box box = (Box)broadPhase.Tree.GetUserData(id);

            if (box.TestPoint(box.Body.Transform, m_point))
            {
                cb.ReportShape(box);
            }

            return true;
        }
    }
    public class Scene
    {
        private int m_bodyCount = 0;
        private Body m_bodyList = null;
        

        private Vector3 m_gravity = new Vector3(0,-9.8f,0);
        private float m_dt;
        private int m_iterations = 20;

        private bool m_allowSleep = false;
        private bool m_enableFriction = false;

        private ContactManager m_contactManager = null;
        private bool m_newBox;

        public Scene(float dt) : this(dt, new Vector3(0.0f, -9.8f, 0.0f))
        {
        }

        public Scene(float dt, Vector3 gravity, int iterations = 20)
        {
            m_contactManager = new ContactManager();
            m_bodyCount = 0;
            m_bodyList = null;
            m_gravity = gravity;
            m_dt = dt;
            m_iterations = iterations;
            m_newBox = false;
            m_allowSleep = true;
            m_enableFriction  =true;
        }

        // Run the simulation forward in time by dt (fixed timestep). Variable
        // timestep is not supported.
        public void Step()
        {
            if (m_newBox)
            {
                m_contactManager.Broadphase.UpdatePairs();
                m_newBox = false;
            }

            m_contactManager.TestCollisions();

            for (Body body = m_bodyList; body != null; body = body.Next)
                body.BodyFlags &= ~Body.Flags.eIsland;

            // Size the stack island, pick worst case size
            /*m_stack.Reserve(
                sizeof(q3Body*) * m_bodyCount
                + sizeof(q3VelocityState) * m_bodyCount
                + sizeof(q3ContactConstraint*) * m_contactManager.m_contactCount
                + sizeof(q3ContactConstraintState) * m_contactManager.m_contactCount
                + sizeof(q3Body*) * m_bodyCount
            );*/

            Island island = new Island();
            island.BodyCapacity = m_bodyCount;
            island.ContactCapacity = m_contactManager.ContactCount;
            island.Bodies = new Body[m_bodyCount];
            island.Velocities = new VelocityState[m_bodyCount];
            for(int i = 0; i < island.Velocities.Length;i++)
                island.Velocities[i] = new VelocityState();
            island.Contacts = new ContactConstraint[island.ContactCapacity];
            for (int i = 0; i < island.Contacts.Length; i++)
                island.Contacts[i] = new ContactConstraint();
            island.ContactStates = new ContactConstraintState[island.ContactCapacity];
            for (int i = 0; i < island.ContactStates.Length; i++)
                island.ContactStates[i] = new ContactConstraintState();
            island.AllowSleep = m_allowSleep;
            island.EnableFriction = m_enableFriction;
            island.BodyCount = 0;
            island.ContactCount = 0;
            island.Dt = m_dt;
            island.Gravity = m_gravity;
            island.Iterations = m_iterations;

            // Build each active island and then solve each built island
            int stackSize = m_bodyCount;
            Body[] stack = new Body[stackSize];

            for (Body seed = m_bodyList; seed != null; seed = seed.Next)
            {
                // Seed cannot be apart of an island already
                if ((seed.BodyFlags & Body.Flags.eIsland) != 0)
                    continue;

                // Seed must be awake
                if ((seed.BodyFlags & Body.Flags.eAwake) == 0)
                    continue;

                // Seed cannot be a static body in order to keep islands
                // as small as possible
                if ((seed.BodyFlags & Body.Flags.eStatic)!=0)
                    continue;

                int stackCount = 0;
                stack[stackCount++] = seed;
                island.BodyCount = 0;
                island.ContactCount = 0;

                // Mark seed as apart of island
                seed.BodyFlags |= Body.Flags.eIsland;

                // Perform DFS on constraint graph
                while (stackCount > 0)
                {
                    // Decrement stack to implement iterative backtracking
                    Body body = stack[--stackCount];
                    island.Add(body);

                    // Awaken all bodies connected to the island
                    body.SetToAwake();

                    // Do not search across static bodies to keep island
                    // formations as small as possible, however the static
                    // body itself should be apart of the island in order
                    // to properly represent a full contact
                    if ((body.BodyFlags & Body.Flags.eStatic)!=0)
                        continue;

                    // Search all contacts connected to this body
                    ContactEdge contacts = body.ContactList;
                    for (ContactEdge edge = contacts; edge != null; edge = edge.Next)
                    {
                        ContactConstraint contact = edge.Constraint;

                        // Skip contacts that have been added to an island already
                        if ((contact.MFlags & ContactConstraint.Flags.Island)!=0)
                            continue;

                        // Can safely skip this contact if it didn't actually collide with anything
                        if ((contact.MFlags & ContactConstraint.Flags.Colliding) == 0)
                            continue;

                        // Skip sensors
                        if (contact.A.Sensor || contact.B.Sensor)
                            continue;

                        // Mark island flag and add to island
                        contact.MFlags |= Dynamics.ContactConstraint.Flags.Island;
                        island.Add(contact);

                        // Attempt to add the other body in the contact to the island
                        // to simulate contact awakening propogation
                        Body other = edge.Other;
                        if ((other.BodyFlags & Body.Flags.eIsland)!=0)
                            continue;

                        Debug.Assert(stackCount < stackSize);

                        stack[stackCount++] = other;
                        other.BodyFlags |= Body.Flags.eIsland;
                    }
                }

                Debug.Assert(island.BodyCount != 0);

                island.Initialize();
                island.Solve();

                // Reset all static island flags
                // This allows static bodies to participate in other island formations
                for (int i = 0; i < island.BodyCount; i++)
                {
                    Body body = island.Bodies[i];

                    if ((body.BodyFlags & Body.Flags.eStatic)!=0)
                        body.BodyFlags &= ~Body.Flags.eIsland;
                }
            }

            stack = null;
            island.ContactStates = null;
            island.Contacts = null;
            island.Velocities = null;
            island.Bodies = null;

            
            // Update the broadphase AABBs
            for (Body body = m_bodyList; body != null; body = body.Next)
            {
                if ((body.BodyFlags & Body.Flags.eStatic) != 0)
                    continue;

                body.SynchronizeProxies();
            }

            // Look for new contacts
            m_contactManager.FindNewContacts();

            // Clear all forces
            for (Body body = m_bodyList; body != null; body = body.Next)
            {
                body.Force = Vector3.Identity;
                body.Torque = Vector3.Identity;
            }
        }

        // Construct a new rigid body. The BodyDef can be reused at the user's
        // discretion, as no reference to the BodyDef is kept.
        public Body CreateBody(BodyDefinition def)
        {
            Body body = new Body(def,this);
           
            // Add body to scene bodyList
            body.Previous = null;
            body.Next = m_bodyList;

            if (m_bodyList != null)
                m_bodyList.Previous = body;

            m_bodyList = body;
            ++m_bodyCount;

            return body;

        }

	    // Frees a body, removes all shapes associated with the body and frees
	    // all shapes and contacts associated and attached to this body.
        public void RemoveBody(Body body)
        {
            Debug.Assert(m_bodyCount > 0);

            m_contactManager.RemoveContactsFromBody(body);

            body.RemoveAllBoxes();

            // Remove body from scene bodyList
            if (body.Next != null)
                body.Next.Previous = body.Previous;

            if (body.Previous != null)
                body.Previous.Next = body.Next;

            if (body == m_bodyList)
                m_bodyList = body.Next;

            --m_bodyCount;
            body = null;
        }

        public void RemoveAllBodies()
        {
            Body body = m_bodyList;

            while (body != null)
            {
                Body next = body.Next;

                body.RemoveAllBoxes();

                body = null;

                body = next;
            }

            m_bodyList = null;
        }

        // Enables or disables rigid body sleeping. Sleeping is an effective CPU
        // optimization where bodies are put to sleep if they don't move much.
        // Sleeping bodies sit in memory without being updated, until the are
        // touched by something that wakes them up. The default is enabled.
        public bool AllowSleep
        {
            get { return m_allowSleep;}
            set
            {
                m_allowSleep = value;

                if (!value)
                {
                    for (Body body = m_bodyList; body!= null; body = body.Next)
                        body.SetToAwake();
                }
            }
        }

        // Increasing the iteration count increases the CPU cost of simulating
        // Scene.Step(). Decreasing the iterations makes the simulation less
        // realistic (convergent). A good iteration number range is 5 to 20.
        // Only positive numbers are accepted. Non-positive and negative
        // inputs set the iteration count to 1.
        public int Iterations
        {
            get { return m_iterations;}
            set
            {
                m_iterations = System.Math.Max(1, value);
            }
        }

        // Friction occurs when two rigid bodies have shapes that slide along one
        // another. The friction force resists this sliding motion.
        public bool EnableFriction
        {
            get { return m_enableFriction; }
            set { m_enableFriction = value; }
        }


        // Gets and sets the global gravity vector used during integration
        public Vector3 Gravity
        {
            get { return m_gravity; }
            set { m_gravity = value; }
        }
       

	// Removes all bodies from the scene.
        public void Shutdown()
        {
            RemoveAllBodies();
        }

        // Sets the listener to report collision start/end. Provides the user
        // with a pointer to an q3ContactConstraint. The q3ContactConstraint
        // holds pointers to the two shapes involved in a collision, and the
        // two bodies connected to each shape. The q3ContactListener will be
        // called very often, so it is recommended for the funciton to be very
        // efficient. Provide a NULL pointer to remove the previously set
        // listener.
        public ContactListener ContactListener
        {
            get { return m_contactManager.ContactListener; }
            set { m_contactManager.ContactListener = value; }
        }

        // Query the world to find any shapes that can potentially intersect
        // the provided AABB. This works by querying the broadphase with an
        // AAABB -- only *potential* intersections are reported. Perhaps the
        // user might use lmDistance as fine-grained collision detection.
        public void QueryAABB(IQueryCallback cb, AABB aabb)
        {
            SceneQueryWrapperAABB wrapperAabb = new SceneQueryWrapperAABB(m_contactManager.Broadphase,aabb,cb);
            m_contactManager.Broadphase.Tree.Query(wrapperAabb, ref aabb);
        }

       


        // Query the world to find any shapes intersecting a world space point.
        public void QueryPoint(IQueryCallback cb, Vector3 point)
        {
            SceneQueryWrapperPoint wrapper = new SceneQueryWrapperPoint(m_contactManager.Broadphase,point,cb);

            float k_fattener = 0.5f;
            Vector3 v = new Vector3(k_fattener,k_fattener,k_fattener);
           
            AABB aabb = new AABB();
            aabb.Min = point - v;
            aabb.Max = point + v;
            m_contactManager.Broadphase.Tree.Query(wrapper,ref aabb);
        }

        // Query the world to find any shapes intersecting a ray.
        public void RayCast(IQueryCallback cb, RaycastData rayCast)
        {
            SceneQueryWrapperRayCast wrapper = new SceneQueryWrapperRayCast(m_contactManager.Broadphase,rayCast,cb);
        
            m_contactManager.Broadphase.Tree.Query(wrapper,ref rayCast);

        }


        public void Dispose()
        {
           Shutdown();
        }

        public ContactManager ContactManager
        {
            get { return m_contactManager; }
            set { m_contactManager = value; }
        }

        public bool NewBox
        {
            get { return m_newBox; }
            set { m_newBox = value; }
        }
    }


    // This class represents general queries for points, AABBs and Raycasting.
    // ReportShape is called the moment a valid shape is found. The return
    // value of ReportShape controls whether to continue or stop the query.
    // By returning only true, all shapes that fulfill the query will be re-
    // ported.
    public interface IQueryCallback
    {
       bool ReportShape(Box box);
    }



    // This listener is used to gather information about two shapes colliding. This
    // can be used for game logic and sounds. Physics objects created in these
    // callbacks will not be reported until the following frame. These callbacks
    // can be called frequently, so make them efficient.
    public interface ContactListener
    {
        void BeginContact(ContactConstraint contact);
        void EndContact(ContactConstraint contact);
    };
}

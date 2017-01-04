//--------------------------------------------------------------------------------------------------
/**
@file	ContactManager.h
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
using Qu3ECSharp.Collision;
using Qu3ECSharp.Math;
using Qu3ECSharp.Scene;

namespace Qu3ECSharp.Dynamics
{
    public class ContactManager
    {
        ContactConstraint m_contactList;
        int m_contactCount;

        BroadPhase.BroadPhase m_broadphase = null;
        ContactListener m_contactListener = null;

        public ContactManager()
        {
            m_contactList = null;
            m_contactCount = 0;
            m_contactListener = null;
        }

        public void AddContact(Box A, Box B)
        {
            Body bodyA = A.Body;
            Body bodyB = B.Body;
            if (!bodyA.CanCollide(bodyB))
                return;
            // Search for existing matching contact
            // Return if found duplicate to avoid duplicate constraints
            // Mark pre-existing duplicates as active
            ContactEdge edge = A.Body.ContactList;
            while (edge != null)
            {
                if (edge.Other == bodyB)
                {
                    Box shapeA = edge.Constraint.A;
                    Box shapeB = edge.Constraint.B;

                    // @TODO: Verify this against Box2D; not sure if this is all we need here
                    if ((A == shapeA) && (B == shapeB))
                        return;
                }

                edge = edge.Next;
            }


            // Create new contact
            ContactConstraint contact = new ContactConstraint();
            ;
            contact.A = A;
            contact.B = B;
            contact.BodyA = A.Body;
            contact.BodyB = B.Body;
            contact.Manifold.SetPair(A, B);
            contact.MFlags = 0;
            contact.Friction = Util.MixFriction(A, B);
            contact.Restitution = Util.MixRestitution(A, B);
            contact.Manifold.ContactCount = 0;

            for (int i = 0; i < 8; ++i)
                contact.Manifold.Contacts[i].WarmStarted = 0;

            contact.Previous = null;
            contact.Next = m_contactList;
            if (m_contactList != null)
                m_contactList.Previous = contact;
            m_contactList = contact;

            // Connect A
            contact.EdgeA.Constraint = contact;
            contact.EdgeA.Other = bodyB;

            contact.EdgeA.Previous = null;
            contact.EdgeA.Next = bodyA.ContactList;
            if (bodyA.ContactList != null)
                bodyA.ContactList.Previous = contact.EdgeA;
            bodyA.ContactList = contact.EdgeA;

            // Connect B
            contact.EdgeB.Constraint = contact;
            contact.EdgeB.Other = bodyA;

            contact.EdgeB.Previous = null;
            contact.EdgeB.Next = bodyB.ContactList;
            if (bodyB.ContactList != null)
                bodyB.ContactList.Previous = contact.EdgeB;
            bodyB.ContactList = contact.EdgeB;

            bodyA.SetToAwake();
            bodyB.SetToAwake();

            ++m_contactCount;

        }

        public void FindNewContacts()
        {
            m_broadphase.UpdatePairs();
        }

        public void RemoveContact(ContactConstraint contact)
        {
            Body A = contact.BodyA;
            Body B = contact.BodyB;

            // Remove from A
            if (contact.EdgeA.Previous != null)
                contact.EdgeA.Previous.Next = contact.EdgeA.Next;

            if (contact.EdgeA.Next != null)
                contact.EdgeA.Next.Previous = contact.EdgeA.Previous;

            if (contact.EdgeA == A.ContactList)
                A.ContactList = contact.EdgeA.Next;

            // Remove from B
            if (contact.EdgeB.Previous != null)
                contact.EdgeB.Previous.Next = contact.EdgeB.Next;

            if (contact.EdgeB.Next != null)
                contact.EdgeB.Next.Previous = contact.EdgeB.Previous;

            if (contact.EdgeB == B.ContactList)
                B.ContactList = contact.EdgeB.Next;

            A.SetToAwake();
            B.SetToAwake();

            // Remove contact from the manager
            if (contact.Previous != null)
                contact.Previous.Next = contact.Next;

            if (contact.Next != null)
                contact.Next.Previous = contact.Previous;

            if (contact == m_contactList)
                m_contactList = contact.Next;

            --m_contactCount;

            contact.Dispose();
        }

        public void RemoveContactsFromBody(Body body)
        {
            ContactEdge edge = body.ContactList;

            while (edge != null)
            {
                ContactEdge next = edge.Next;
                RemoveContact(edge.Constraint);
                edge = next;
            }
        }

        public void RemoveFromBroadphase(Body body)
        {
            Box box = body.Boxes;

            while (box != null)
            {
                m_broadphase.RemoveBox(box);
                box = box.Next;
            }
        }


        public void TestCollisions()
        {
            ContactConstraint constraint = m_contactList;

            while (constraint != null)
            {
                Box A = constraint.A;
                Box B = constraint.B;
                Body bodyA = A.Body;
                Body bodyB = B.Body;

                constraint.m_flags &= ~ContactConstraint.Flags.Island;
                if (!bodyA.IsAwake() && !bodyB.IsAwake())
                {
                    constraint = constraint.Next;
                    continue;
                }

                if (!bodyA.CanCollide(bodyB))
                {
                    ContactConstraint next = constraint.Next;
                    RemoveContact(constraint);
                    constraint = next;
                    continue;
                }

                // Check if contact should persist
                if (!m_broadphase.TestOverlap(A.BroadPhaseNext, B.BroadPhaseNext))
                {
                    ContactConstraint next = constraint.Next;
                    RemoveContact(constraint);
                    constraint = next;
                    continue;
                }

                Manifold manifold = constraint.Manifold;
                Manifold oldManifold = constraint.Manifold.Clone();
                Vector3 ot0 = oldManifold.TangentVectors[0];
                Vector3 ot1 = oldManifold.TangentVectors[1];
                constraint.SolveCollision();
                Vector3 tanx, tany;
                Common.Geometry.ComputeBasis(manifold.Normal, out tanx, out tany);
                manifold.TangentVectors[0] = tanx;
                manifold.TangentVectors[1] = tany;
                for (int i = 0; i < manifold.ContactCount; ++i)
                {
                    Contact c = manifold.Contacts[i];
                    c.TangentImpulse[0] = c.TangentImpulse[1] = c.NormalImpulse = 0.0f;
                    byte oldWarmStart = c.WarmStarted;
                    c.WarmStarted = 0;
                    for (int j = 0; j < oldManifold.ContactCount; ++j)
                    {
                        Contact oc = oldManifold.Contacts[j];
                        if (c.FeaturePair.Key == oc.FeaturePair.Key)
                        {
                            c.NormalImpulse = oc.NormalImpulse;

                            // Attempt to re-project old friction solutions
                            Vector3 friction = ot0 * oc.TangentImpulse[0] + ot1 * oc.TangentImpulse[1];
                            c.TangentImpulse[0] = Vector3.Dot(friction, manifold.TangentVectors[0]);
                            c.TangentImpulse[1] = Vector3.Dot(friction, manifold.TangentVectors[1]);
                            c.WarmStarted = System.Math.Max(oldWarmStart, (byte) (oldWarmStart + 1));
                            break;
                        }
                    }
                }

                if (m_contactListener != null)
                {
                    ContactConstraint.Flags now_colliding = constraint.MFlags & ContactConstraint.Flags.Colliding;
                    ContactConstraint.Flags was_colliding = constraint.MFlags & ContactConstraint.Flags.WasColliding;

                    if ((now_colliding != 0) && (was_colliding == 0))
                        m_contactListener.BeginContact(constraint);

                    else if ((now_colliding == 0) && (was_colliding != 0))
                        m_contactListener.EndContact(constraint);
                }

                constraint = constraint.Next;


            }


        }
    }
}

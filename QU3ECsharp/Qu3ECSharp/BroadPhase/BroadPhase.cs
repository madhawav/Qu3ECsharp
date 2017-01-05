//--------------------------------------------------------------------------------------------------
/**
@file	q3BroadPhase.h
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
using Qu3ECSharp.Common;
using Qu3ECSharp.Dynamics;

namespace Qu3ECSharp.BroadPhase
{

    public struct ContactPair
    {
        public int A
        {
            get { return _A; }
            set { _A = value; }
        }

        public int B
        {
            get { return _B; }
            set { _B = value; }
        }

        private int _A;
        private int _B;


    };
    public class BroadPhase :IQuerable
    {
        private ContactManager m_manager = null;

        private ContactPair[] m_pairBuffer = null;

        
        private int m_pairCount = 0;
        private int m_pairCapacity = 0;

        private int[] m_moveBuffer = null;
        private int m_moveCount = 0;
        private int m_moveCapacity = 0;

        private DynamicAABBTree m_tree = null;
        private int m_currentIndex = 0;

        private void BufferMove(int id)
        {
            if (m_moveCount == m_moveCapacity)
            {
                int[] oldBuffer = m_moveBuffer;
                m_moveCapacity *= 2;
                m_moveBuffer = new int[m_moveCapacity];
                int i = 0;
                foreach (int n in oldBuffer)
                    m_moveBuffer[i++] = n;
                oldBuffer = null;
            }

            m_moveBuffer[m_moveCount++] = id;
        }

     
        public BroadPhase(ContactManager manager)
        {
            m_manager = manager;

            m_pairCount = 0;
            m_pairCapacity = 64;
            m_pairBuffer = new ContactPair[m_pairCapacity];
            

            m_moveCount = 0;
            m_moveCapacity = 64;
            m_moveBuffer = new int[m_moveCapacity];
            
        }

        public void InsertBox(Box box, AABB aabb)
        {
            int id = m_tree.Insert(ref aabb, box);
            box.BroadPhaseIndex = id;
            BufferMove(id);
        }

        public static bool ContactPairSort(ref ContactPair lhs, ref ContactPair rhs)
        {
            if (lhs.A < rhs.A)
                return true;

            if (lhs.A == rhs.A)
                return lhs.B < rhs.B;

            return false;
        }

        public void Update(int id, AABB aabb)
        {
            if (m_tree.Update(id, ref aabb))
                BufferMove(id);
        }

        public void UpdatePairs()
        {
            m_pairCount = 0;

            // Query the tree with all moving boxs
            for (int i = 0; i < m_moveCount; ++i)
            {
                m_currentIndex = m_moveBuffer[i];
                AABB aabb = m_tree.GetFatAABB(m_currentIndex);

                // @TODO: Use a static and non-static tree and query one against the other.
                //        This will potentially prevent (gotta think about this more) time
                //        wasted with queries of static bodies against static bodies, and
                //        kinematic to kinematic.
                m_tree.Query(this,ref aabb);
            }

            // Reset the move buffer
            m_moveCount = 0;

            // Sort pairs to expose duplicates

           
            System.Array.Sort<ContactPair>(m_pairBuffer,new ContactPairComparer());
            //std::sort(m_pairBuffer, m_pairBuffer + m_pairCount, ContactPairSort);

            // Queue manifolds for solving
            {
                int i = 0;
                while (i < m_pairCount)
                {
                    // Add contact to manager
                    ContactPair pair = m_pairBuffer[i];
                    Box A = (Box)m_tree.GetUserData(pair.A);
                    Box B = (Box)m_tree.GetUserData(pair.B);
                    m_manager.AddContact(A, B);

                    ++i;

                    // Skip duplicate pairs by iterating i until we find a unique pair
                    while (i < m_pairCount)
                    {
                        ContactPair potentialDup = m_pairBuffer[i];

                        if (pair.A != potentialDup.A || pair.B != potentialDup.B)
                            break;

                        ++i;
                    }
                }
            }

            m_tree.Validate();
        }

        public void RemoveBox(Box box)
        {
            m_tree.Remove(box.BroadPhaseIndex);
        }

        public bool TestOverlap(int A, int B)
        {
            return Common.AABB.CollisionAABBtoAABB(m_tree.GetFatAABB(A), m_tree.GetFatAABB(B));
        }

        public bool TreeCallBack(int index)
        {
            // Cannot collide with self
            if (index == m_currentIndex)
                return true;

            if (m_pairCount == m_pairCapacity)
            {
                ContactPair[] oldBuffer = m_pairBuffer;
                m_pairCapacity *= 2;
                m_pairBuffer = new ContactPair[m_pairCapacity];
                int i = 0;
                foreach (ContactPair c in oldBuffer)
                    m_pairBuffer[i++] = c;
                oldBuffer = null;
            }

            int iA = System.Math.Min(index, m_currentIndex);
            int iB = System.Math.Max(index, m_currentIndex);

            m_pairBuffer[m_pairCount].A = iA;
            m_pairBuffer[m_pairCount].B = iB;
            ++m_pairCount;

            return true;
        }
    }

    public class ContactPairComparer : IComparer<ContactPair>
    {
        public int Compare(ContactPair x, ContactPair y)
        {
            if (x.A < y.A)
                return -1;
            else if (x.A > y.A)
                return 1;
            else
            {
                if (x.B < y.B)
                    return -1;
                else if (x.B > y.B)
                    return 1;
                else return 0;
            }
            
        }
    }

}

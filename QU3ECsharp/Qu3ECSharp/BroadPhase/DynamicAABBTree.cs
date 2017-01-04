//--------------------------------------------------------------------------------------------------
/**
@file	q3DynamicAABBTree.h
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
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading.Tasks;
using Qu3ECSharp.Common;
using Qu3ECSharp.Math;

namespace Qu3ECSharp.BroadPhase
{
    public class DynamicAABBTree
    {
        public const int Null = -1;
        private class Node
        {
            public AABB aabb = new AABB();
            public int next = Null;
            public int parent = Null;
            public int left = Null;
            public int right = Null;

            public object userData = null;
            public int height = Null;

            

            public bool IsLeaf()
            {
                // The right leaf does not use the same memory as the userdata,
                // and will always be Null (no children)
                return right == Null;
            }
        }

        private int AllocateNode()
        {
            if (m_freeList == Null)
            {
                m_capacity *= 2;
                Node[] newNodes = new Node[m_capacity];
                int pointer = 0;
                foreach (Node n in m_nodes)
                {
                    newNodes[pointer++] = n;
                }
                m_nodes = null;
                m_nodes = newNodes;

                AddToFreeList(m_count);
            }

            int freeNode = m_freeList;
            m_freeList = m_nodes[m_freeList].next;
            m_nodes[freeNode].height = 0;
            m_nodes[freeNode].left = Null;
            m_nodes[freeNode].right = Null;
            m_nodes[freeNode].parent = Null;
            m_nodes[freeNode].userData = null;
            ++m_count;
            return freeNode;
        }

        private void DeallocateNode(int index)
        {
            Debug.Assert(index >= 0 && index < m_capacity);

            m_nodes[index].next = m_freeList;
            m_nodes[index].height = Null;
            m_freeList = index;

            --m_count;
        }

        private int Balance(int iA)
        {
            Node A = m_nodes[iA];

            if (A.IsLeaf() || A.height == 1)
                return iA;

            /*      A
                  /   \
                 B     C
                / \   / \
               D   E F   G
            */

            int iB = A.left;
            int iC = A.right;
            Node B = m_nodes[iB];
            Node C = m_nodes[iC];

            int balance = C.height - B.height;

            // C is higher, promote C
            if (balance > 1)
            {
                int iF = C.left;
                int iG = C.right;
                Node F = m_nodes [iF];
                Node G = m_nodes [iG];

                // grandParent point to C
                if (A.parent != Null)
                {
                    if (m_nodes[A.parent].left == iA)
                        m_nodes[A.parent].left = iC;

                    else
                        m_nodes[A.parent].right = iC;
                }
                else
                    m_root = iC;

                // Swap A and C
                C.left = iA;
                C.parent = A.parent;
                A.parent = iC;

                // Finish rotation
                if (F.height > G.height)
                {
                    C.right = iF;
                    A.right = iG;
                    G.parent = iA;
                    A.aabb = AABB.Combine(B.aabb, G.aabb);
                    C.aabb = AABB.Combine(A.aabb, F.aabb);

                    A.height = 1 + System.Math.Max(B.height, G.height);
                    C.height = 1 + System.Math.Max(A.height, F.height);
                }

                else
                {
                    C.right = iG;
                    A.right = iF;
                    F.parent = iA;
                    A.aabb = AABB.Combine(B.aabb, F.aabb);
                    C.aabb = AABB.Combine(A.aabb, G.aabb);

                    A.height = 1 + System.Math.Max(B.height, F.height);
                    C.height = 1 + System.Math.Max(A.height, G.height);
                }

                return iC;
            }

            // B is higher, promote B
            else if (balance < -1)
            {
                int iD = B.left;
                int iE = B.right;
                Node D = m_nodes[iD];
                Node E = m_nodes[iE];

                // grandParent point to B
                if (A.parent != Null)
                {
                    if (m_nodes[A.parent].left == iA)
                        m_nodes[A.parent].left = iB;
                    else
                        m_nodes[A.parent].right = iB;
                }

                else
                    m_root = iB;

                // Swap A and B
                B.right = iA;
                B.parent = A.parent;
                A.parent = iB;

                // Finish rotation
                if (D.height > E.height)
                {
                    B.left = iD;
                    A.left = iE;
                    E.parent = iA;
                    A.aabb = AABB.Combine(C.aabb, E.aabb);
                    B.aabb = AABB.Combine(A.aabb, D.aabb);

                    A.height = 1 + System.Math.Max(C.height, E.height);
                    B.height = 1 + System.Math.Max(A.height, D.height);
                }

                else
                {
                    B.left = iE;
                    A.left = iD;
                    D.parent = iA;
                    A.aabb = AABB.Combine(C.aabb, D.aabb);
                    B.aabb = AABB.Combine(A.aabb, E.aabb);

                    A.height = 1 + System.Math.Max(C.height, D.height);
                    B.height = 1 + System.Math.Max(A.height, E.height);
                }

                return iB;
            }

            return iA;
        }

        private void InsertLeaf(int id)
        {
            if (m_root == Null)
            {
                m_root = id;
                m_nodes[m_root].parent = Null;
                return;
            }

            // Search for sibling
            int searchIndex = m_root;
            AABB leafAABB = m_nodes[id].aabb;
            while (!m_nodes[searchIndex].IsLeaf())
            {
                // Cost for insertion at index (branch node), involves creation
                // of new branch to contain this index and the new leaf
                AABB combined = AABB.Combine(leafAABB, m_nodes[searchIndex].aabb);
                float combinedArea = combined.SurfaceArea;
                float branchCost = 2.0f * combinedArea;

                // Inherited cost (surface area growth from heirarchy update after descent)
                float inheritedCost = 2.0f * (combinedArea - m_nodes[searchIndex].aabb.SurfaceArea);

                int left = m_nodes[searchIndex].left;
                int right = m_nodes[searchIndex].right;

                // Calculate costs for left/right descents. If traversal is to a leaf,
                // then the cost of the combind AABB represents a new branch node. Otherwise
                // the cost is only the inflation of the pre-existing branch.
                float leftDescentCost;
                if (m_nodes[left].IsLeaf())
                    leftDescentCost = AABB.Combine(leafAABB, m_nodes[left].aabb).SurfaceArea + inheritedCost;
                else
                {
                    float inflated = AABB.Combine(leafAABB, m_nodes[left].aabb).SurfaceArea;
                    float branchArea = m_nodes[left].aabb.SurfaceArea;
                    leftDescentCost = inflated - branchArea + inheritedCost;
                }

                // Cost for right descent
                float rightDescentCost;
                if (m_nodes[right].IsLeaf())
                    rightDescentCost = AABB.Combine(leafAABB, m_nodes[right].aabb).SurfaceArea + inheritedCost;
                else
                {
                    float inflated = AABB.Combine(leafAABB, m_nodes[right].aabb).SurfaceArea;
                    float branchArea = m_nodes[right].aabb.SurfaceArea;
                    rightDescentCost = inflated - branchArea + inheritedCost;
                }

                // Determine traversal direction, or early out on a branch index
                if (branchCost < leftDescentCost && branchCost < rightDescentCost)
                    break;

                if (leftDescentCost < rightDescentCost)
                    searchIndex = left;

                else
                    searchIndex = right;
            }

            int sibling = searchIndex;

            // Create new parent
            int oldParent = m_nodes[sibling].parent;
            int newParent = AllocateNode();
            m_nodes[newParent].parent = oldParent;
            m_nodes[newParent].userData = null;
            m_nodes[newParent].aabb = AABB.Combine(leafAABB, m_nodes[sibling].aabb);
            m_nodes[newParent].height = m_nodes[sibling].height + 1;

            // Sibling was root
            if (oldParent == Null)
            {
                m_nodes[newParent].left = sibling;
                m_nodes[newParent].right = id;
                m_nodes[sibling].parent = newParent;
                m_nodes[id].parent = newParent;
                m_root = newParent;
            }

            else
            {
                if (m_nodes[oldParent].left == sibling)
                    m_nodes[oldParent].left = newParent;

                else
                    m_nodes[oldParent].right = newParent;

                m_nodes[newParent].left = sibling;
                m_nodes[newParent].right = id;
                m_nodes[sibling].parent = newParent;
                m_nodes[id].parent = newParent;
            }

            SyncHeirarchy(m_nodes[id].parent);
        }

        private void RemoveLeaf(int id)
        {
            if (id == m_root)
            {
                m_root = Null;
                return;
            }

            // Setup parent, grandParent and sibling
            int parent = m_nodes[id].parent;
            int grandParent = m_nodes[parent].parent;
            int sibling;

            if (m_nodes[parent].left == id)
                sibling = m_nodes[parent].right;

            else
                sibling = m_nodes[parent].left;

            // Remove parent and replace with sibling
            if (grandParent != Null)
            {
                // Connect grandParent to sibling
                if (m_nodes[grandParent].left == parent)
                    m_nodes[grandParent].left = sibling;

                else
                    m_nodes[grandParent].right = sibling;

                // Connect sibling to grandParent
                m_nodes[sibling].parent = grandParent;
            }

            // Parent was root
            else
            {
                m_root = sibling;
                m_nodes[sibling].parent = Null;
            }

            DeallocateNode(parent);
            SyncHeirarchy(grandParent);
        }

        [ConditionalAttribute("DEBUG")]
        private void ValidateStructure(int index)
        {
            Node n = m_nodes[index];

            int il = n.left;
            int ir = n.right;

            if (n.IsLeaf())
            {
                Debug.Assert(ir == Null);
                Debug.Assert(n.height == 0);
                return;
            }

            Debug.Assert(il >= 0 && il < m_capacity);
            Debug.Assert(ir >= 0 && ir < m_capacity);
            Node l = m_nodes [il];
            Node r = m_nodes [ir];

            Debug.Assert(l.parent == index);
            Debug.Assert(r.parent == index);

            ValidateStructure(il);
            ValidateStructure(ir);
        }
        

        // Correct AABB hierarchy heights and AABBs starting at supplied
        // index traversing up the heirarchy
        private void SyncHeirarchy(int index)
        {
            while (index != Null)
            {
                index = Balance(index);

                int left = m_nodes[index].left;
                int right = m_nodes[index].right;

                m_nodes[index].height = 1 + System.Math.Max(m_nodes[left].height, m_nodes[right].height);
                m_nodes[index].aabb = AABB.Combine(m_nodes[left].aabb, m_nodes[right].aabb);

                index = m_nodes[index].parent;
            }
        }

        // Insert nodes at a given index until m_capacity into the free list
        private void AddToFreeList(int index)
        {
            for (int i = index; i < m_capacity - 1; ++i)
            {
                m_nodes[i].next = i + 1;
                m_nodes[i].height = Null;
            }

            m_nodes[m_capacity - 1].next = Null;
            m_nodes[m_capacity - 1].height = Null;
            m_freeList = index;
        }



        private int m_root = -1;
        private Node[] m_nodes = null;
        private int m_count = 0;    // Number of active nodes
        private int m_capacity = 0; // Max capacity of nodes
        private int m_freeList = 0;

        private static void FattenAABB(ref AABB aabb)
        {
            float k_fattener = 0.5f;
            Vector3 v = new Vector3(k_fattener, k_fattener, k_fattener);
            
            aabb.Min -= v;
            aabb.Min += v;
        }


        public DynamicAABBTree()
        {
            m_root = Null;

            m_capacity = 1024;
            m_count = 0;
            m_nodes = new Node[m_capacity];

            AddToFreeList(0);
        }

        public void Dispose()
        {
            m_nodes = null;
        }

        public int Insert(ref AABB aabb, object userData)
        {
            int id = AllocateNode();

            // Fatten AABB and set height/userdata
            m_nodes[id].aabb = aabb;
            FattenAABB(ref m_nodes[id].aabb);
            m_nodes[id].userData = userData;
            m_nodes[id].height = 0;

            InsertLeaf(id);

            return id;
        }

        public void Remove(int id)
        {

            Debug.Assert(id >= 0 && id < m_capacity);
            Debug.Assert(m_nodes[id].IsLeaf());

            RemoveLeaf(id);
            DeallocateNode(id);
        }

        public bool Update(int id, ref AABB aabb)
        {
            Debug.Assert(id >= 0 && id < m_capacity);
            Debug.Assert(m_nodes[id].IsLeaf());

            if (m_nodes[id].aabb.Contains(aabb))
                return false;

            RemoveLeaf(id);

            m_nodes[id].aabb = aabb;
            FattenAABB(ref m_nodes[id].aabb);

            InsertLeaf(id);

            return true;
        }

        public object GetUserData(int id)
        {
            Debug.Assert(id >= 0 && id < m_capacity);
                
            return m_nodes[id].userData;
        }

        public AABB GetFatAABB(int id)
        {
            Debug.Assert(id >= 0 && id < m_capacity);

            return m_nodes[id].aabb;
        }

        public void Query(IQuerable cb, ref AABB aabb)
        {
            const int k_stackCapacity = 256;
            int[] stack = new int[k_stackCapacity];
            int sp = 1;

            stack[0] = m_root;

            while (sp != 0)
            {
                // k_stackCapacity too small
                Debug.Assert(sp < k_stackCapacity);

                int id = stack[--sp];

                Node n = m_nodes[id];
                if (AABB.CollisionAABBtoAABB(aabb, n.aabb))
                {
                    if (n.IsLeaf())
                    {
                        if (!cb.TreeCallBack(id))
                            return;
                    }
                    else
                    {
                        stack[sp++] = n.left;
                        stack[sp++] = n.right;
                    }
                }
            }
        }

        public void Query(IQuerable cb, ref RaycastData rayCast)
        {
            const float k_epsilon = (float)(1.0e-6);
            const int k_stackCapacity = 256;
            int[] stack=  new int[k_stackCapacity];
            int sp = 1;

            stack[0] = m_root;

            Vector3 p0 = rayCast.Start;
            Vector3 p1 = p0 + rayCast.Direction * rayCast.Time;

            while (sp != 0)
            {
                // k_stackCapacity too small
                Debug.Assert(sp < k_stackCapacity);

                int id = stack[--sp];

                if (id == Null)
                    continue;

                Node n = m_nodes[id];

                Vector3 e = n.aabb.Max - n.aabb.Min;
                Vector3 d = p1 - p0;
                Vector3 m = p0 + p1 - n.aabb.Min - n.aabb.Max;

                float adx = System.Math.Abs(d.X);

                if (System.Math.Abs(m.X) > e.X + adx)
                    continue;

                float ady = System.Math.Abs(d.Y);

                if (System.Math.Abs(m.Y) > e.Y + ady)
                    continue;

                float adz = System.Math.Abs(d.Z);

                if (System.Math.Abs(m.Z) > e.Z + adz)
                    continue;

                adx += k_epsilon;
                ady += k_epsilon;
                adz += k_epsilon;

                if (System.Math.Abs(m.Y * d.Z - m.Z * d.Y) > e.Y * adz + e.Z * ady)
                    continue;

                if (System.Math.Abs(m.Z * d.X - m.X * d.Z) > e.X * adz + e.Z * adx)
                    continue;

                if (System.Math.Abs(m.X * d.Y - m.Y * d.X) > e.X * ady + e.Y * adx)
                    continue;

                if (n.IsLeaf())
                {
                    if (!cb.TreeCallBack(id))
                        return;
                }

                else
                {
                    stack[sp++] = n.left;
                    stack[sp++] = n.right;
                }
            }
        }

        public void Validate()
        {
            // Verify free list
            int freeNodes = 0;
            int index = m_freeList;

            while (index != Null)
            {
                Debug.Assert(index >= 0 && index < m_capacity);
                index = m_nodes[index].next;
                ++freeNodes;
            }

            Debug.Assert(m_count + freeNodes == m_capacity);

            // Validate tree structure
            if (m_root != Null)
            {
                Debug.Assert(m_nodes[m_root].parent == Null);
                ValidateStructure(m_root);

            }
        }

    }

    public interface IQuerable
    {
        bool TreeCallBack(int id);
    }
}




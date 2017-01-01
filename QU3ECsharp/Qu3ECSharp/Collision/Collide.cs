

//--------------------------------------------------------------------------------------------------

/*
Ported to CSharp by Madhawa Vidanapathirana 
https://github.com/madhawav
*/
//--------------------------------------------------------------------------------------------------




using Qu3ECSharp.Dynamics;
/**
@file	Settings.h
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
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Qu3ECSharp.Collision
{
    public class Collide
    {
        private Collide() { }

        private static bool TrackFaceAxis(ref int axis, int n, float s, ref float sMax, Vector3 normal, ref Vector3 axisNormal)
        {
            //Find the least penetration????

            if (s > 0.0f)
                return true;

            if (s > sMax)
            {
                sMax = s;
                axis = n;
                axisNormal = normal;
            }

            return false;
        }

        private static bool TrackEdgeAxis(ref int axis, int n, float s, ref float sMax, Vector3 normal, ref Vector3 axisNormal)
        {
            if (s > 0.0f)
                return true;

            float l = 1.0f / Vector3.Length(normal);
            s *= l;

            if (s > sMax)
            {
                sMax = s;
                axis = n;
                axisNormal = normal * l;
            }

            return false;
        }

        struct ClipVertex
        {
            public Vector3 v;
            public FeaturePair f;

            public static ClipVertex Default
            {
                get
                {
                    FeaturePair f = new FeaturePair();
                    f.Key = ~0;
                    ClipVertex c = new ClipVertex();
                    c.f = f;
                    return c;
                }
            }

            public Vector3 Vector { get { return v; } set { v = value; } }
            public FeaturePair FeaturePair { get { return f; } set { f = value; } }
        }


        private static void ComputeReferenceEdgesAndBasis(Vector3 eR, Transform rtx, Vector3 n, int axis, ref byte[] result, ref Matrix3 basis, ref Vector3 e)
        {
            n = Transform.MultiplyTranspose(rtx.Rotation, n);
            if (axis >= 3)
                axis -= 3;

            switch (axis)
            {
                case 0:
                    if (n.X > 0.0f)
                    {
                        result[0] = 1;
                        result[1] = 8;
                        result[2] = 7;
                        result[3] = 9;

                        e.Set(eR.Y, eR.Z, eR.X);
                        basis.SetRows(rtx.Rotation.eY, rtx.Rotation.eZ, rtx.Rotation.eX);
                    }

                    else
                    {
                        result[0] = 11;
                        result[1] = 3;
                        result[2] = 10;
                        result[3] = 5;

                        e.Set(eR.Z, eR.Y, eR.X);
                        basis.SetRows(rtx.Rotation.eZ, rtx.Rotation.eY, -rtx.Rotation.eX);
                    }
                    break;

                case 1:
                    if (n.Y > 0.0f)
                    {
                        result[0] = 0;
                        result[1] = 1;
                        result[2] = 2;
                        result[3] = 3;

                        e.Set(eR.Z, eR.X, eR.Y);
                        basis.SetRows(rtx.Rotation.eZ, rtx.Rotation.eX, rtx.Rotation.eY);
                    }
                    else
                    {
                        result[0] = 4;
                        result[1] = 5;
                        result[2] = 6;
                        result[3] = 7;

                        e.Set(eR.Z, eR.X, eR.Y);
                        basis.SetRows(rtx.Rotation.eZ, -rtx.Rotation.eX, -rtx.Rotation.eY);
                    }
                    break;

                case 2:
                    if (n.Z > 0.0f)
                    {
                        result[0] = 11;
                        result[1] = 4;
                        result[2] = 8;
                        result[3] = 0;

                        e.Set(eR.Y, eR.X, eR.Z);
                        basis.SetRows(-rtx.Rotation.eY, rtx.Rotation.eX, rtx.Rotation.eZ);
                    }

                    else
                    {
                        result[0] = 6;
                        result[1] = 10;
                        result[2] = 2;
                        result[3] = 9;

                        e.Set(eR.Y, eR.X, eR.Z);
                        basis.SetRows(-rtx.Rotation.eY, -rtx.Rotation.eX, -rtx.Rotation.eZ);
                    }
                    break;
            }
        }

        private static void ComputeIncidentFace(Transform itx, Vector3 e, Vector3 n, ref ClipVertex[] result)
        {
            n = -Transform.MultiplyTranspose(itx.Rotation, n);
            Vector3 absN = Vector3.Abs(n);

            if (absN.X > absN.Y && absN.X > absN.Z)
            {
                if (n.X > 0.0f)
                {
                    result[0].v.Set(e.X, e.Y, -e.Z);
                    result[1].v.Set(e.X, e.Y, e.Z);
                    result[2].v.Set(e.X, -e.Y, e.Z);
                    result[3].v.Set(e.X, -e.Y, -e.Z);

                    result[0].f.InI = 9;
                    result[0].f.OutI = 1;
                    result[1].f.InI = 1;
                    result[1].f.OutI = 8;
                    result[2].f.InI = 8;
                    result[2].f.OutI = 7;
                    result[3].f.InI = 7;
                    result[3].f.OutI = 9;
                }
                else
                {
                    result[0].v.Set(-e.X, -e.Y, e.Z);
                    result[1].v.Set(-e.X, e.Y, e.Z);
                    result[2].v.Set(-e.X, e.Y, -e.Z);
                    result[3].v.Set(-e.X, -e.Y, -e.Z);

                    result[0].f.InI = 5;
                    result[0].f.OutI = 11;
                    result[1].f.InI = 11;
                    result[1].f.OutI = 3;
                    result[2].f.InI = 3;
                    result[2].f.OutI = 10;
                    result[3].f.InI = 10;
                    result[3].f.OutI = 5;
                }
            }

            else if (absN.Y > absN.X && absN.Y > absN.Z)
            {
                if (n.Y > 0.0f)
                {
                    result[0].v.Set(-e.X, e.Y, e.Z);
                    result[1].v.Set(e.X, e.Y, e.Z);
                    result[2].v.Set(e.X, e.Y, -e.Z);
                    result[3].v.Set(-e.X, e.Y, -e.Z);

                    result[0].f.InI = 3;
                    result[0].f.OutI = 0;
                    result[1].f.InI = 0;
                    result[1].f.OutI = 1;
                    result[2].f.InI = 1;
                    result[2].f.OutI = 2;
                    result[3].f.InI = 2;
                    result[3].f.OutI = 3;
                }
                else
                {
                    result[0].v.Set(e.X, -e.Y, e.Z);
                    result[1].v.Set(-e.X, -e.Y, e.Z);
                    result[2].v.Set(-e.X, -e.Y, -e.Z);
                    result[3].v.Set(e.X, -e.Y, -e.Z);

                    result[0].f.InI = 7;
                    result[0].f.OutI = 4;
                    result[1].f.InI = 4;
                    result[1].f.OutI = 5;
                    result[2].f.InI = 5;
                    result[2].f.OutI = 6;
                    result[3].f.InI = 5;
                    result[3].f.OutI = 6;
                }
            }

            else
            {
                if (n.Z > 0.0f)
                {
                    result[0].v.Set(-e.X, e.Y, e.Z);
                    result[1].v.Set(-e.X, -e.Y, e.Z);
                    result[2].v.Set(e.X, -e.Y, e.Z);
                    result[3].v.Set(e.X, e.Y, e.Z);

                    result[0].f.InI = 0;
                    result[0].f.OutI = 11;
                    result[1].f.InI = 11;
                    result[1].f.OutI = 4;
                    result[2].f.InI = 4;
                    result[2].f.OutI = 8;
                    result[3].f.InI = 8;
                    result[3].f.OutI = 0;
                }

                else
                {
                    result[0].v.Set(e.X, -e.Y, -e.Z);
                    result[1].v.Set(-e.X, -e.Y, -e.Z);
                    result[2].v.Set(-e.X, e.Y, -e.Z);
                    result[3].v.Set(e.X, e.Y, -e.Z);

                    result[0].f.InI = 9;
                    result[0].f.OutI = 6;
                    result[1].f.InI = 6;
                    result[1].f.OutI = 10;
                    result[2].f.InI = 10;
                    result[2].f.OutI = 2;
                    result[3].f.InI = 2;
                    result[3].f.OutI = 9;
                }
            }

            for (int i = 0; i < 4; ++i)
                result[i].v = Transform.Multiply(itx, result[i].v);
        }

        private static bool InFront(float a)
        {
            return a < 0.0f;
        }

        private static bool Behind(float a)
        {
            return a >= 0.0f;
        }

        private static bool On(float a)
        {
            return a < 0.005f && a > -0.005f;
        }


        private static int Orthographic(float sign, float e, int axis, int clipEdge, ClipVertex[] input, ref ClipVertex[] output)
        {
            int outCount = 0;
            ClipVertex a = input[input.Length - 1];
            for (int i = 0; i < input.Length; ++i)
            {
                ClipVertex b = input[i];

                float da = sign * a.v[axis] - e;
                float db = sign * b.v[axis] - e;

                ClipVertex cv = new ClipVertex();

                // B
                if (((InFront(da) && InFront(db)) || On(da) || On(db)))
                {
                    Debug.Assert(outCount < 8);
                    output[outCount++] = b;
                }

                // I
                else if (InFront(da) && Behind(db))
                {
                    cv.f = b.f;
                    cv.v = a.v + (b.v - a.v) * (da / (da - db));
                    cv.f.OutR = (byte)clipEdge;
                    cv.f.OutI = 0;
                    Debug.Assert(outCount < 8);
                    output[outCount++] = cv;
                }

                // I, B
                else if (Behind(da) && InFront(db))
                {
                    cv.f = a.f;
                    cv.v = a.v + (b.v - a.v) * (da / (da - db));
                    cv.f.InR = (byte)clipEdge;
                    cv.f.InI = 0;
                    Debug.Assert(outCount < 8);
                    output[outCount++] = cv;

                    Debug.Assert(outCount < 8);
                    output[outCount++] = b;
                }

                a = b;
            }

            return outCount;

        }


        //--------------------------------------------------------------------------------------------------
        // Resources (also see q3BoxtoBox's resources):
        // http://www.randygaul.net/2013/10/27/sutherland-hodgman-clipping/
        private static int Clip(Vector3 rPos, Vector3 e, byte[] clipEdges, Matrix3 basis, ClipVertex[] incident, ClipVertex[] outVerts, float[] outDepths)
        {
            int inCount = 4;
            int outCount;
            ClipVertex[] input = new ClipVertex[8];
            ClipVertex[] output = new ClipVertex[8];
            for (int i = 0; i < output.Length; i++)
            {
                input[i] = new ClipVertex();
                output[i] = new ClipVertex();
            }

            for (int i = 0; i < 4; ++i)
                input[i].v = Transform.MultiplyTranspose(basis, incident[i].v - rPos);

            outCount = Orthographic(1.0f, e.X, 0, clipEdges[0], input, ref output);

            if (outCount == 0)
                return 0;

            inCount = Orthographic(1.0f, e.Y, 1, clipEdges[1], output, ref input);

            if (inCount == 0)
                return 0;

            outCount = Orthographic(-1.0f, e.X, 0, clipEdges[2], input, ref output);

            if (outCount == 0)
                return 0;

            inCount = Orthographic(-1.0f, e.Y, 1, clipEdges[3], output, ref input);

            // Keep incident vertices behind the reference face
            outCount = 0;
            for (int i = 0; i < inCount; ++i)
            {
                float d = input[i].v.Z - e.Z;

                if (d <= 0.0f)
                {
                    outVerts[outCount].v = Transform.Multiply(basis, input[i].v) + rPos;
                    outVerts[outCount].f = input[i].f;
                    outDepths[outCount++] = d;
                }
            }

            Debug.Assert(outCount <= 8);

            return outCount;

        }

        private static void EdgesContact(out Vector3 CA, out Vector3 CB, Vector3 PA, Vector3 QA, Vector3 PB, Vector3 QB)
        {
            Vector3 DA = QA - PA;
            Vector3 DB = QB - PB;
            Vector3 r = PA - PB;
            float a = Vector3.Dot(DA, DA);
            float e = Vector3.Dot(DB, DB);
            float f = Vector3.Dot(DB, r);
            float c = Vector3.Dot(DA, r);

            float b = Vector3.Dot(DA, DB);
            float denom = a * e - b * b;

            float TA = (b * f - c * e) / denom;
            float TB = (b * TA + f) / e;

            CA = PA + DA * TA;
            CB = PB + DB * TB;
        }


        private static void SupportEdge(Transform tx, Vector3 e, Vector3 n, out Vector3 aOut, out Vector3 bOut)
        {
            n = Transform.MultiplyTranspose(tx.Rotation, n);
            Vector3 absN = Vector3.Abs(n);
            Vector3 a = new Vector3();
            Vector3 b = new Vector3();

            // x > y
            if (absN.X > absN.Y)
            {
                // x > y > z
                if (absN.Y > absN.Z)
                {
                    a.Set(e.X, e.Y, e.Z);
                    b.Set(e.X, e.Y, -e.Z);
                }

                // x > z > y || z > x > y
                else
                {
                    a.Set(e.X, e.Y, e.Z);
                    b.Set(e.X, -e.Y, e.Z);
                }
            }

            // y > x
            else
            {
                // y > x > z
                if (absN.X > absN.Z)
                {
                    a.Set(e.X, e.Y, e.Z);
                    b.Set(e.X, e.Y, -e.Z);
                }

                // z > y > x || y > z > x
                else
                {
                    a.Set(e.X, e.Y, e.Z);
                    b.Set(-e.X, e.Y, e.Z);
                }
            }

            float signx = Math.Math.Sign(n.X);
            float signy = Math.Math.Sign(n.Y);
            float signz = Math.Math.Sign(n.Z);

            a.X *= signx;
            a.Y *= signy;
            a.Z *= signz;
            b.X *= signx;
            b.Y *= signy;
            b.Z *= signz;

            aOut = Transform.Multiply(tx, a);
            bOut = Transform.Multiply(tx, b);
        }

        //--------------------------------------------------------------------------------------------------
        // Resources:
        // http://www.randygaul.net/2014/05/22/deriving-obb-to-obb-intersection-sat/
        // https://box2d.googlecode.com/files/GDC2007_ErinCatto.zip
        // https://box2d.googlecode.com/files/Box2D_Lite.zip

        public static void BoxtoBox(Manifold m, Box a, Box b)
        {
            Transform atx = a.Body.Transform;
            Transform btx = b.Body.Transform;
            Transform aL = a.LocalTransform;
            Transform bL = b.LocalTransform;
            atx = Transform.Multiply(atx, aL);
            btx = Transform.Multiply(btx, bL);

            Vector3 eA = a.Extent;
            Vector3 eB = b.Extent;

            Matrix3 C = Matrix3.Transpose(atx.Rotation) * btx.Rotation;

            Matrix3 absC = new Matrix3();
            bool parallel = false;

            float kCosTol = (float) (1.0e-6);
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    float val = System.Math.Abs(C[i][j]);

                    absC.SetCell(i, j, val); //TODO: Check whether precedence is correct here
                    //absC[i][j] = val;

                    if (val + kCosTol >= 1.0f)
                        parallel = true;
                }
            }

            // Vector from center A to center B in A's space
            Vector3 t = Transform.MultiplyTranspose(atx.Rotation, btx.Position - atx.Position);

            // Query states
            float s;
            float aMax = -float.MaxValue;
            float bMax = -float.MaxValue;
            float eMax = -float.MaxValue;
            int aAxis = ~0;
            int bAxis = ~0;
            int eAxis = ~0;
            Vector3 nA = new Vector3();
            Vector3 nB = new Vector3();
            Vector3 nE = new Vector3();

            // Face axis checks

            // a's x axis
            s = System.Math.Abs(t.X) - (eA.X + Vector3.Dot(absC.Column0(), eB));
            if (TrackFaceAxis(ref aAxis, 0, s, ref aMax, atx.Rotation.eX, ref nA))
                return;

            // a's y axis
            s = System.Math.Abs(t.Y) - (eA.Y + Vector3.Dot(absC.Column1(), eB));
            if (TrackFaceAxis(ref aAxis, 1, s, ref aMax, atx.Rotation.eY, ref nA))
                return;

            // a's z axis
            s = System.Math.Abs(t.Z) - (eA.Z + Vector3.Dot(absC.Column2(), eB));
            if (TrackFaceAxis(ref aAxis, 2, s, ref aMax, atx.Rotation.eZ, ref nA))
                return;

            // b's x axis
            s = System.Math.Abs(Vector3.Dot(t, C.eX)) - (eB.X + Vector3.Dot(absC.eX, eA));
            if (TrackFaceAxis(ref bAxis, 3, s, ref bMax, btx.Rotation.eX, ref nB))
                return;

            // b's y axis
            s = System.Math.Abs(Vector3.Dot(t, C.eY)) - (eB.Y + Vector3.Dot(absC.eY, eA));
            if (TrackFaceAxis(ref bAxis, 4, s, ref bMax, btx.Rotation.eY, ref nB))
                return;

            // b's z axis
            s = System.Math.Abs(Vector3.Dot(t, C.eZ)) - (eB.Z + Vector3.Dot(absC.eZ, eA));
            if (TrackFaceAxis(ref bAxis, 5, s, ref bMax, btx.Rotation.eZ, ref nB))
                return;


            if (!parallel)
            {
                // Edge axis checks
                float rA;
                float rB;

                // Cross( a.x, b.x )
                rA = eA.Y * absC[0][2] + eA.Z * absC[0][1];
                rB = eB.Y * absC[2][0] + eB.Z * absC[1][0];
                s = System.Math.Abs(t.Z * C[0][1] - t.Y * C[0][2]) - (rA + rB);
                if (TrackEdgeAxis(ref eAxis, 6, s, ref eMax, new Vector3(0.0f, -C[0][2], C[0][1]), ref nE))
                    return;

                // Cross( a.x, b.y )
                rA = eA.Y * absC[1][2] + eA.Z * absC[1][1];
                rB = eB.X * absC[2][0] + eB.Z * absC[0][0];
                s = System.Math.Abs(t.Z * C[1][1] - t.Y * C[1][2]) - (rA + rB);
                if (TrackEdgeAxis(ref eAxis, 7, s, ref eMax, new Vector3(0.0f, -C[1][2], C[1][1]), ref nE))
                    return;

                // Cross( a.x, b.z )
                rA = eA.Y * absC[2][2] + eA.Z * absC[2][1];
                rB = eB.X * absC[1][0] + eB.Y * absC[0][0];
                s = System.Math.Abs(t.Z * C[2][1] - t.Y * C[2][2]) - (rA + rB);
                if (TrackEdgeAxis(ref eAxis, 8, s, ref eMax, new Vector3(0.0f, -C[2][2], C[2][1]), ref nE))
                    return;

                // Cross( a.y, b.x )
                rA = eA.X * absC[0][2] + eA.Z * absC[0][0];
                rB = eB.Y * absC[2][1] + eB.Z * absC[1][1];
                s = System.Math.Abs(t.X * C[0][2] - t.Z * C[0][0]) - (rA + rB);
                if (TrackEdgeAxis(ref eAxis, 9, s, ref eMax, new Vector3(C[0][2], 0.0f, -C[0][0]), ref nE))
                    return;

                // Cross( a.y, b.y )
                rA = eA.X * absC[1][2] + eA.Z * absC[1][0];
                rB = eB.X * absC[2][1] + eB.Z * absC[0][1];
                s = System.Math.Abs(t.X * C[1][2] - t.Z * C[1][0]) - (rA + rB);
                if (TrackEdgeAxis(ref eAxis, 10, s, ref eMax, new Vector3(C[1][2], 0.0f, -C[1][0]), ref nE))
                    return;

                // Cross( a.y, b.z )
                rA = eA.X * absC[2][2] + eA.Z * absC[2][0];
                rB = eB.X * absC[1][1] + eB.Y * absC[0][1];
                s = System.Math.Abs(t.X * C[2][2] - t.Z * C[2][0]) - (rA + rB);
                if (TrackEdgeAxis(ref eAxis, 11, s, ref eMax, new Vector3(C[2][2], 0.0f, -C[2][0]), ref nE))
                    return;

                // Cross( a.z, b.x )
                rA = eA.X * absC[0][1] + eA.Y * absC[0][0];
                rB = eB.Y * absC[2][2] + eB.Z * absC[1][2];
                s = System.Math.Abs(t.Y * C[0][0] - t.X * C[0][1]) - (rA + rB);
                if (TrackEdgeAxis(ref eAxis, 12, s, ref eMax, new Vector3(-C[0][1], C[0][0], 0.0f), ref nE))
                    return;

                // Cross( a.z, b.y )
                rA = eA.X * absC[1][1] + eA.Y * absC[1][0];
                rB = eB.X * absC[2][2] + eB.Z * absC[0][2];
                s = System.Math.Abs(t.Y * C[1][0] - t.X * C[1][1]) - (rA + rB);
                if (TrackEdgeAxis(ref eAxis, 13, s, ref eMax, new Vector3(-C[1][1], C[1][0], (0.0f)), ref nE))
                    return;

                // Cross( a.z, b.z )
                rA = eA.X * absC[2][1] + eA.Y * absC[2][0];
                rB = eB.X * absC[1][2] + eB.Y * absC[0][2];
                s = System.Math.Abs(t.Y * C[2][0] - t.X * C[2][1]) - (rA + rB);
                if (TrackEdgeAxis(ref eAxis, 14, s, ref eMax, new Vector3(-C[2][1], C[2][0], 0.0f), ref nE))
                    return;
            }

            // Artificial axis bias to improve frame coherence
            float kRelTol = 0.95f;
            float kAbsTol = 0.01f;
            int axis;
            float sMax;
            Vector3 n;

            float faceMax = System.Math.Max(aMax, bMax);

            if (kRelTol * eMax > faceMax + kAbsTol)
            {
                axis = eAxis;
                sMax = eMax;
                n = nE;
            }

            else
            {
                if (kRelTol * bMax > aMax + kAbsTol)
                {
                    axis = bAxis;
                    sMax = bMax;
                    n = nB;
                }

                else
                {
                    axis = aAxis;
                    sMax = aMax;
                    n = nA;
                }
            }

            if (Vector3.Dot(n, btx.Position - atx.Position) < 0.0f)
                n = -n;

            Debug.Assert(axis != ~0);

            if (axis < 6)
            {
                Transform rtx;
                Transform itx;
                Vector3 eR;
                Vector3 eI;
                bool flip;

                if (axis < 3)
                {
                    rtx = atx;
                    itx = btx;
                    eR = eA;
                    eI = eB;
                    flip = false;
                }

                else
                {
                    rtx = btx;
                    itx = atx;
                    eR = eB;
                    eI = eA;
                    flip = true;
                    n = -n;
                }

                // Compute reference and incident edge information necessary for clipping
                ClipVertex[] incident = new ClipVertex[4];
                ComputeIncidentFace(itx, eI, n, ref incident);
                byte[] clipEdges = new byte[4];
                Matrix3 basis = new Matrix3();
                Vector3 e = new Vector3();
                ComputeReferenceEdgesAndBasis(eR, rtx, n, axis, ref clipEdges, ref basis, ref e);

                // Clip the incident face against the reference face side planes
                ClipVertex[] outputs = new ClipVertex[8];
                float[] depths = new float[8];
                int outNum;
                outNum = Clip(rtx.Position, e, clipEdges, basis, incident, outputs, depths);

                if (outNum != 0)
                {
                    m.ContactCount = outNum;
                    m.Normal = flip ? -n : n;

                    for (int i = 0; i < outNum; ++i)
                    {
                        Contact c = m.Contacts[i];

                        FeaturePair pair = outputs[i].f;

                        if (flip)
                        {
                            byte temp = pair.InI;
                            pair.InI = pair.InR;
                            pair.InR = temp;

                            temp = pair.OutI;
                            pair.OutI = pair.OutR;
                            pair.OutR = temp;
                        }
                        outputs[i].f = pair;
                        c.FeaturePair = outputs[i].f;

                        c.Position = outputs[i].v;
                        c.Penetration = depths[i];
                    }
                }
            }
            else
            {
                n = atx.Rotation * n;

                if (Vector3.Dot(n, btx.Position - atx.Position) < (0.0f))
                    n = -n;

                Vector3 PA, QA;
                Vector3 PB, QB;
                SupportEdge(atx, eA, n, out PA, out QA);
                SupportEdge(btx, eB, -n, out PB, out QB);

                Vector3 CA, CB;
                EdgesContact(out CA, out CB, PA, QA, PB, QB);

                m.Normal = n;
                m.ContactCount = 1;

                Contact[] c = m.Contacts;
                FeaturePair pair = new FeaturePair();
                pair.Key = axis;
                c[0].FeaturePair = pair;
                c[0].Penetration = sMax;
                c[0].Position = (CA + CB) * (0.5f);
            }

        }

    }
}

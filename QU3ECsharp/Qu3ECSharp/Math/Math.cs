//--------------------------------------------------------------------------------------------------
/**
@file	q3Math.inl
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
/*
Ported to CSharp by Madhawa Vidanapathirana 
https://github.com/madhawav
*/
//--------------------------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Qu3ECSharp.Math
{
    public class Math
    {
        public static Random random = new Random();
        public static float Abs(float a)
        {
            if (a < 0.0f)
                return -a;

            return a;
        }

        public static float Min(float a, float b)
        {
            if (a < b)
                return a;

            return b;
        }
        public static float Max(float a, float b)
        {
            if (a < b)
                return b;

            return a;
        }

        public static float Sign(float a)
        {
            if (a >= 0.0f)
            {
                return 1.0f;
            }

            else
            {
                return -1.0f;
            }
        }


        public static float Clamp01(float val)
        {
            if (val >= 1.0f)
                return 1.0f;

            if (val <= 0.0f)
                return 0.0f;

            return val;
        }

        public static float Clamp(float min, float max, float a)
        {
            if (a < min)
                return min;

            if (a > max)
                return max;

            return a;
        }
        public static float Lerp(float a, float b, float t)
        {
            return a * (1.0f - t) + b * t;
        }

        
        public static Vector3 Lerp( Vector3 a, Vector3 b, float t )
        {
	        return a* (1.0f  - t) + b* t;
        }

        public static float RandomFloat(float l, float h)
        {
            float a = (float)random.NextDouble();
            a = (h - l) * a + l;
            return a;
        }

        public static int RandomInt(int low, int high)
        {
            return random.Next(low, high);
        }
    }
}

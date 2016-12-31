//--------------------------------------------------------------------------------------------------
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
/*
Ported to CSharp by Madhawa Vidanapathirana 
https://github.com/madhawav
*/
//--------------------------------------------------------------------------------------------------

namespace Qu3ECSharp.Common
{
    public class Settings
    {
        public static float SLEEP_LINEAR = 0.01f;
        public static float SLEEP_ANGULAR = (3.0f / 180.0f) * (float)System.Math.PI;
        public static float SLEEP_TIME = 0.5f;
        public static float BAUMGARTE = 0.2f;
        public static float PENETRATION_SLOP = 0.05f;
    }
}

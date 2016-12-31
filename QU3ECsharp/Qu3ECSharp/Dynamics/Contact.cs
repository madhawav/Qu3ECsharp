//--------------------------------------------------------------------------------------------------
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
 

    public class Contact
    {
    }
}

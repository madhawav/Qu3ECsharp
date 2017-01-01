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
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Qu3ECSharp.Dynamics;

namespace Qu3ECSharp.Scene
{
    class Scene
    {
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

//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2022, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar

    \author    <pkunjam1@jhu.edu>
    \author    Punit Kunjam
*/
//==============================================================================

#include "collision.h"

using namespace std;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

string g_current_filepath;

afCollision::afCollision()
{
}

int afCollision::init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs)
{

    cerr << "INFO! LOADING COLLISION PLUGIN \n";

    m_afRigidBody = (afRigidBodyPtr)a_afObjectPtr;
    m_startTime = m_afRigidBody->getSimulationTime();
    m_original_scaling =  m_afRigidBody->m_bulletCollisionShape->getLocalScaling();

    return 1;
}

void afCollision::graphicsUpdate()
{

}

void afCollision::physicsUpdate(double dt)
{
    double curent_time = m_afRigidBody->getSimulationTime();
    if (curent_time - m_startTime > 0.2 ){
        m_startTime = curent_time;
        cerr << "INFO! Changinc Collision Shape \n";
        btVector3 current_scaling = m_original_scaling + btVector3(1,1,1) * sin(curent_time);
        m_afRigidBody->m_bulletCollisionShape->setLocalScaling(current_scaling / 2.0);
        m_afRigidBody->m_afWorld->m_bulletWorld->updateSingleAabb(m_afRigidBody->m_bulletRigidBody);

        m_afRigidBody->m_afWorld->m_bulletWorld->removeRigidBody(m_afRigidBody->m_bulletRigidBody);

         m_afRigidBody->m_afWorld->m_bulletWorld->addRigidBody(m_afRigidBody->m_bulletRigidBody);
        cerr << "\t INFO! Reducing to " << current_scaling[0] << ", " << current_scaling[1] << ", " << current_scaling[2]  << endl;
    }
}

void afCollision::reset(){

}

bool afCollision::close()
{
    return true;
}

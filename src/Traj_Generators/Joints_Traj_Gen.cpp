/*

    Joint Trajectory Generator Class
    This class is an interface to generate arbitrary Joint trajectory in the robot's joint space

    Copyright 2018 Università della Campania Luigi Vanvitelli

    Author: Marco Costanzo <marco.costanzo@unicampania.it>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "Traj_Generators/Joints_Traj_Gen.h"

using namespace TooN;

    /*======CONSTRUCTORS=========*/
    
        /*
            Constructor with number of joints and total time as input
        */
        Joints_Traj_Gen::Joints_Traj_Gen( int num_joints, double final_time ):
            _num_joints(num_joints),
            Traj_Generator_Interface(final_time){}

        //Joints_Traj_Gen( const Joints_Traj_Gen& traj );

    /*======END CONSTRUCTORS=========*/

    /*====== GETTERS =========*/

    /*
        Get the number of joints
    */
     double Joints_Traj_Gen::getNumJoints() const{
        return _num_joints;
    }

    /*====== END GETTERS =========*/

    /*
        Get the mask at time secs, if mask[i]=0 then the i-th joint should not be taken into account
    */
     Vector<Dynamic,int> Joints_Traj_Gen::getMask(double secs) const{
        return Ones(_num_joints);
    }
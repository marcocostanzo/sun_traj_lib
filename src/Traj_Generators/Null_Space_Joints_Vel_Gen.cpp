/*

    Joint Velocity Trajectory Generator For Using In The Null Space - Class
    This class is an interface to generate arbitrary Joint Velocity Trajectory in the robot's joint space
    to use into the null space

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


#include "Traj_Generators/Null_Space_Joints_Vel_Gen.h"

using namespace TooN;

    /*======CONSTRUCTORS=========*/
    
        /*
            Constructor with number of joints and total time as input
        */
        Null_Space_Joints_Vel_Gen::Null_Space_Joints_Vel_Gen( int num_joints):
            _num_joints(num_joints),
            Traj_Generator_Interface(-INFINITY){}

        //Null_Space_Joints_Vel_Gen( const Null_Space_Joints_Vel_Gen& traj );

        /*
            Clone the object in the heap
        */
        Null_Space_Joints_Vel_Gen* Null_Space_Joints_Vel_Gen::clone() const{
            return new Null_Space_Joints_Vel_Gen(*this);
        }

    /*======END CONSTRUCTORS=========*/

    /*====== GETTERS =========*/

    /*
        Get the number of joints
    */
     double Null_Space_Joints_Vel_Gen::getNumJoints() const{
        return _num_joints;
    }

    /*====== END GETTERS =========*/

    /*
        Get Joints Velocity at time secs, usefull inputs are: q_DH and the robot pointer
    */
    Vector<> Null_Space_Joints_Vel_Gen::getJointsVelocity(double secs, const Vector<>& q_DH, const RobotPtr& robot ) const{
        return Zeros(_num_joints);
    }

    /*
        Get the mask at time secs, if mask[i]=0 then the i-th joint should not be taken into account
    */
     Vector<Dynamic,int> Null_Space_Joints_Vel_Gen::getMask(double secs) const{
        return Ones(_num_joints);
    }

    /*
        initialize the trajectory
    */
    void Null_Space_Joints_Vel_Gen::initialize(const TooN::Vector<>& qDH_k, const RobotPtr& _robot){}
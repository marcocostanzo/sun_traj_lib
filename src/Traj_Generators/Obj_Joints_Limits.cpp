/*

    Joints Speed Generator for the objective "away from the limits"

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


#include "Traj_Generators/Obj_Joints_Limits.h"

using namespace TooN;
using namespace std;

    /*======CONSTRUCTORS=========*/
    
        /*
            Constructor with number of joints:
            This generate a constant trajectory of ZERO velocity
        */
        Obj_Joints_Limits::Obj_Joints_Limits(  int num_joints,
                            Vector<> desired_configuration,
                            Vector<> desired_configuration_joint_weights
                        ):
                Null_Space_Joints_Vel_Gen(num_joints),
                _desired_configuration(desired_configuration),
                _desired_configuration_joint_weights(desired_configuration_joint_weights)
                {}

        Obj_Joints_Limits::Obj_Joints_Limits(  int num_joints,
                            TooN::Vector<> desired_configuration
                        ):
                        Obj_Joints_Limits(num_joints, desired_configuration, Zeros(num_joints))
                        {}

        Obj_Joints_Limits::Obj_Joints_Limits(  int num_joints ):
                        Obj_Joints_Limits(num_joints, Zeros(num_joints), Zeros(num_joints))
                        {}

        /*
            Clone the object in the heap
        */
        Obj_Joints_Limits* Obj_Joints_Limits::clone() const{
            return new Obj_Joints_Limits(*this);
        }

    /*======END CONSTRUCTORS=========*/

    /*====== GETTERS =========*/

    /*====== END GETTERS =========*/

    /*
        Get Joints Velocity at time secs, usefull inputs are: q_DH and the robot pointer
    */
    Vector<> Obj_Joints_Limits::getJointsVelocity(double secs, const Vector<>& q_DH, const RobotPtr& robot ) const{
        return robot->grad_fcst_target_configuration( q_DH, _desired_configuration , _desired_configuration_joint_weights );
    }

    Vector<> Obj_Joints_Limits::getJointsVelocity(const Vector<>& q_DH, const RobotPtr& robot ) const{
        return robot->grad_fcst_target_configuration( q_DH, _desired_configuration , _desired_configuration_joint_weights );
    }

    /*
        weigths for the joints in the cst function
    */
    Vector<Dynamic,int> Obj_Joints_Limits::getMask(double secs) const{
        return _desired_configuration_joint_weights;
    }

    Vector<Dynamic,int> Obj_Joints_Limits::getMask() const{
        return _desired_configuration_joint_weights;
    }
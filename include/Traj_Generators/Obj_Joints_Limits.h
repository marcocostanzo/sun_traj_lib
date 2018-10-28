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


#ifndef OBJ_JOINTS_LIMITS_H
#define OBJ_JOINTS_LIMITS_H

#include "Traj_Generators/Null_Space_Joints_Vel_Gen.h"

class Obj_Joints_Limits : public Null_Space_Joints_Vel_Gen {

    private:

        /*
            Avoid Default constructor
        */
        Obj_Joints_Limits();

    protected:

        /*
            points to be considered as center of joints
        */
        TooN::Vector<> _desired_configuration;

        /*
            weigths for the joints in the cst function
        */
        TooN::Vector<> _desired_configuration_joint_weights;
    public:

    /*======CONSTRUCTORS=========*/
    
        /*
            Constructor with number of joints:
            This generate a constant trajectory of ZERO velocity
        */
        Obj_Joints_Limits(  int num_joints,
                            TooN::Vector<> desired_configuration,
                            TooN::Vector<> desired_configuration_joint_weights
                        );

        Obj_Joints_Limits(  int num_joints,
                            TooN::Vector<> desired_configuration
                        );

        Obj_Joints_Limits(  int num_joints
                        );

        Obj_Joints_Limits( const Obj_Joints_Limits& traj ) = default;

        /*
            Clone the object in the heap
        */
        virtual Obj_Joints_Limits* clone() const override;

    /*======END CONSTRUCTORS=========*/

    /*====== GETTERS =========*/

    /*====== END GETTERS =========*/

    /*
        Get Joints Velocity at time secs, usefull inputs are: q_DH and the robot pointer
    */
    virtual TooN::Vector<> getJointsVelocity(double secs, const TooN::Vector<>& q_DH, const RobotPtr& robot ) const override;

    virtual TooN::Vector<> getJointsVelocity(const TooN::Vector<>& q_DH, const RobotPtr& robot ) const;

    /*
        weigths for the joints in the cst function
    */
    virtual TooN::Vector<TooN::Dynamic,int> getMask(double secs) const override;

    virtual TooN::Vector<TooN::Dynamic,int> getMask() const;

};//END CLASS

using Obj_Joints_Limits_Ptr = std::unique_ptr<Obj_Joints_Limits>;

#endif
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


#ifndef NULL_SPACE_JOINTS_VEL_GEN_H
#define NULL_SPACE_JOINTS_VEL_GEN_H

#include "TooN/TooN.h"
#include "Traj_Generators/Traj_Generator_Interface.h"
#include "Robot.h"

class Null_Space_Joints_Vel_Gen : public Traj_Generator_Interface {

    private:

        /*
            Avoid Default constructor
        */
        Null_Space_Joints_Vel_Gen();

    protected:

        /*
            number of joints
        */
        int _num_joints;

    public:

    /*======CONSTRUCTORS=========*/
    
        /*
            Constructor with number of joints:
            This generate a constant trajectory of ZERO velocity
        */
        Null_Space_Joints_Vel_Gen( int num_joints);

        Null_Space_Joints_Vel_Gen( const Null_Space_Joints_Vel_Gen& traj ) = default;

        /*
            Clone the object in the heap
        */
        virtual Null_Space_Joints_Vel_Gen* clone() const;

    /*======END CONSTRUCTORS=========*/

    /*====== GETTERS =========*/

    /*
        Get the number of joints
    */
    virtual double getNumJoints() const;

    /*====== END GETTERS =========*/

    /*
        Get Joints Velocity at time secs, usefull inputs are: q_DH and the robot pointer
    */
    virtual TooN::Vector<> getJointsVelocity(double secs, const TooN::Vector<>& q_DH, const RobotPtr& robot ) const;

    /*
        Get the mask at time secs, if mask[i]=0 then the i-th joint should not be taken into account
    */
    virtual TooN::Vector<TooN::Dynamic,int> getMask(double secs) const;

    /*
        initialize the trajectory
    */
    virtual void initialize(const TooN::Vector<>& qDH_k, const RobotPtr& _robot);

};//END CLASS

using Null_Space_Joints_Vel_Gen_Ptr = std::unique_ptr<Null_Space_Joints_Vel_Gen>;

#endif
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

#ifndef JOINTS_TRAJ_GEN_H
#define JOINTS_TRAJ_GEN_H

#include "TooN/TooN.h"
#include "Robot.h"
#include "Traj_Generators/Traj_Generator_Interface.h"

class Joints_Traj_Gen : public Traj_Generator_Interface {

    private:

        /*
            Avoid Default constructor
        */
        Joints_Traj_Gen();

    protected:

        /*
            number of joints
        */
        int _num_joints;

    public:

    /*======CONSTRUCTORS=========*/
    
        /*
            Constructor with number of joints and total time as input
        */
        Joints_Traj_Gen( int num_joints, double final_time );

        //Joints_Traj_Gen( const Joints_Traj_Gen& traj );

        /*
            Clone the object in the heap
        */
        virtual Joints_Traj_Gen* clone() const = 0;

    /*======END CONSTRUCTORS=========*/

    /*====== GETTERS =========*/

    /*
        Get the number of joints
    */
    virtual double getNumJoints() const;

    /*====== END GETTERS =========*/

    /*
        Get Joints Position at time secs
    */
    virtual TooN::Vector<> getJointsPosition(double secs) const = 0;

    /*
        Get Joints Velocity at time secs
    */
    virtual TooN::Vector<> getJointsVelocity(double secs) const = 0;

    /*
        Get the mask at time secs, if mask[i]=0 then the i-th joint should not be taken into account
    */
    virtual TooN::Vector<TooN::Dynamic,int> getMask(double secs) const;

    /*
        initialize the trajectory
    */
    virtual void initialize(const TooN::Vector<>& qDH_k, const RobotPtr& _robot) = 0;

};//END CLASS

using Joints_Traj_Gen_Ptr = std::unique_ptr<Joints_Traj_Gen>;

#endif
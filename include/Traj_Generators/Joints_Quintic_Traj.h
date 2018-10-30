/*

    Quintic Joint Trajectory Generator Class
    This class generates a quintic Joint trajectory in the robot's joint space

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

#ifndef JOINTS_QUINTIC_TRAJ_GEN_H
#define JOINTS_QUINTIC_TRAJ_GEN_H

#include "Traj_Generators/Joints_Traj_Gen.h"
#include "Traj_Generators/Quintic_Poly_Gen.h"

class Joints_Quintic_Traj : public Joints_Traj_Gen {

    private:

        /*
            Avoid Default constructor
        */
        Joints_Quintic_Traj();

    protected:

        std::vector<Quintic_Poly_Gen> quintic_joints;

        TooN::Vector<> _desired_configutation;
        double _initial_time;

    public:

    /*======CONSTRUCTORS=========*/
    
        /*
            Constructor with number of joints and total time as input
        */
        Joints_Quintic_Traj( 
                                const TooN::Vector<>& desired_configutation,
                                double initial_time, 
                                double final_time
                                 );

        Joints_Quintic_Traj( const Joints_Quintic_Traj& traj ) = default;
            

        /*
            Clone the object in the heap
        */
        virtual Joints_Quintic_Traj* clone() const;

    /*======END CONSTRUCTORS=========*/

    /*====== GETTERS =========*/

    /*====== END GETTERS =========*/

    /*
        Get Joints Position at time secs
    */
    virtual TooN::Vector<> getJointsPosition(double secs) const;

    /*
        Get Joints Velocity at time secs
    */
    virtual TooN::Vector<> getJointsVelocity(double secs) const;

    /*
        initialize the trajectory
    */
    virtual void initialize(const TooN::Vector<>& qDH_k, const RobotPtr& _robot);

};//END CLASS

using Joints_Quintic_Traj_Ptr = std::unique_ptr<Joints_Quintic_Traj>;

#endif
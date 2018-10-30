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


#include "Traj_Generators/Joints_Quintic_Traj.h"

using namespace TooN;
using namespace std;


    /*======CONSTRUCTORS=========*/
    
        /*
            Constructor with number of joints and total time as input
        */
        Joints_Quintic_Traj::Joints_Quintic_Traj( 
                                const Vector<>& desired_configutation,
                                double initial_time, 
                                double final_time
                                 ):
            Joints_Traj_Gen( desired_configutation.size(), final_time ),
            _desired_configutation(desired_configutation),
            _initial_time(initial_time){

                for( int i = 0; i < _num_joints; i++ ){
                    quintic_joints.push_back( 
                                                Quintic_Poly_Gen(
                                                    initial_time,
                                                    final_time,
                                                    desired_configutation[i],
                                                    desired_configutation[i],
                                                    0.0, 
                                                    0.0,
                                                    0.0, 
                                                    0.0
                                                )
                                            );
                }

            }

        /*
            Clone the object in the heap
        */
        Joints_Quintic_Traj* Joints_Quintic_Traj::clone() const{
            return new Joints_Quintic_Traj(*this);
        }

    /*======END CONSTRUCTORS=========*/

    /*====== GETTERS =========*/

    /*====== END GETTERS =========*/

    /*
        Get Joints Position at time secs
    */
    Vector<> Joints_Quintic_Traj::getJointsPosition(double secs) const{
        Vector<> out = Zeros(_num_joints);
        for( int i = 0; i < _num_joints; i++ ){
            out[i] = quintic_joints[i].getPosition(secs);
        }
        return out;
    }

    /*
        Get Joints Velocity at time secs
    */
    Vector<> Joints_Quintic_Traj::getJointsVelocity(double secs) const{
        Vector<> out = Zeros(_num_joints);
        for( int i = 0; i < _num_joints; i++ ){
            out[i] = quintic_joints[i].getVelocity(secs);
        }
        return out;
    }

    /*
        initialize the trajectory
    */
    void Joints_Quintic_Traj::initialize(const Vector<>& qDH_k, const RobotPtr& _robot){
        for( int i = 0; i < _num_joints; i++ ){
            quintic_joints[i].setInitialPosition( qDH_k[i] );
        }
    }
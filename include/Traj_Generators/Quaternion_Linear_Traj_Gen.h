/*

    Quaternion Linear Trajectory Generator Class
    This class is generates a linear orientation trajectory using quaternions

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

//TODO Try w. UnitQuaternion.interp(...)

#ifndef QUATERNION_LINEAR_TRAJ_GEN_H
#define QUATERNION_LINEAR_TRAJ_GEN_H

#include "Traj_Generators/Quaternion_Traj_Gen.h"
#include "Traj_Generators/Traj_Gen_common.h"
#include "Traj_Generators/Quintic_Poly_Gen.h"

class Quaternion_Linear_Traj_Gen : public Quaternion_Traj_Gen {

    private:

        /*
            No default Constructor
        */
        Quaternion_Linear_Traj_Gen();

    protected:

        /*
            Initial Time
        */
        double _initial_time;

        /*
            Initial Quaternion in {base} frame
        */
        UnitQuaternion _b_quat_init;

        /*
            AngVec of the final rotation in the {initial} Frame
        */
        AngVec _init_angvec_desired;

        /*
            Rotation axis in the {base} frame
        */
        TooN::Vector<3> _b_rotax;

        /*
            Mag of initial and final angular velocity (direction is the rotation axis)
        */
        double _wi, _wf;

        /*
            Mag of initial and final angular acceleration (direction is the rotation axis)
        */
        double _dwi, _dwf;

        /*
            mask
            if mask[i]=0 then the i-th cartesian coordinate should not be taken into account
        */      
        TooN::Vector<3,int> _mask;

        /*
            Quintic poly generator for the angle variable
        */
        Quintic_Poly_Gen _quintic_angle;

        /*
            Mode
        */
        Robot_Motion_Mode _mode;   

    public:

    /*======CONSTRUCTORS=========*/
    
        /*
            Constructor
        */
        Quaternion_Linear_Traj_Gen( 
                                    Robot_Motion_Mode mode,
                                    double initial_time,
                                    double final_time,
                                    const AngVec& ang_vec,
                                    double final_angular_velocity = 0.0,
                                    double initial_angular_velocity = 0.0,
                                    double initial_angular_acceleration = 0.0,
                                    double final_angular_acceleration = 0.0,
                                    const TooN::Vector<3,int>& mask = TooN::Ones                                     
                                    );


        /*
            Copy Constructor
        */
        Quaternion_Linear_Traj_Gen( const Quaternion_Linear_Traj_Gen& traj ) = default;

        /*
            Clone the object in the heap
        */
        virtual Quaternion_Linear_Traj_Gen* clone() const override;

    /*======END CONSTRUCTORS=========*/

    /*====== GETTERS =========*/

    /*====== END GETTERS =========*/

    /*
        Get AngVec at time secs
    */
    virtual AngVec getAngVec(double secs) const;

    /*
        Get Quaternion at time secs
    */
    virtual UnitQuaternion getQuaternion(double secs) const override;

    /*
        Get Angular Velocity at time secs
    */
    virtual TooN::Vector<3> getAngularVelocity(double secs) const override;

    /*
        Get the mask at time secs, if mask[i]=0 then the i-th cartesian coordinate should not be taken into account
        Warning: these are UnitQuaternion !!
    */
    virtual TooN::Vector<3,int> getMask(double secs) const override;

    /*
        initialize the trajectory
    */
    virtual void initialize(const TooN::Matrix<4,4>& b_T_init) override;

};//END CLASS

using Quaternion_Linear_Traj_Gen_Ptr = std::unique_ptr<Quaternion_Linear_Traj_Gen>;

#endif
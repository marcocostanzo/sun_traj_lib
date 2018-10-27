/*

    Position Line Trajectory Generator Class
    This class is generates a line trajectory in the cartesian space

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

#ifndef POSITION_LINE_TRTAJ_GEN_H
#define POSITION_LINE_TRTAJ_GEN_H

#include "Traj_Generators/Position_Traj_Gen.h"
#include "Traj_Generators/Traj_Gen_common.h"
#include "Traj_Generators/Quintic_Poly_Gen.h"


class Position_Line_Traj_Gen : public Position_Traj_Gen{


    private:

        /*
            No default Constructor
        */
        Position_Line_Traj_Gen();

    protected:

        /*
            Initial Time
        */
        double _initial_time;
        
        /*
            Initial Position
        */
        TooN::Vector<3> _pi;

        /*
            Final Position
        */
        TooN::Vector<3> _pf;

        /*
            Mag of initial velocity (direction is _pf-_pi)
        */
        double _vi;

        /*
            Mag of final velocity (direction is _pf-_pi)
        */
        double _vf;

        /*
            Mag of initial acceleration (direction is _pf-_pi)
        */
        double _aci;

        /*
            Mag of final acceleration (direction is _pf-_pi)
        */
        double _acf;

        /*
            mask
            if mask[i]=0 then the i-th cartesian coordinate should not be taken into account
        */
        TooN::Vector<3,int> _mask;

        /*
            Quintic poly generator for the s variable
        */
        Quintic_Poly_Gen _quintic_s;

        /*
            Mode
        */
        Robot_Motion_Mode _mode;

        /*
            Direction unit vector
        */
        TooN::Vector<3> _direction;


    public:

    /*======CONSTRUCTORS========*/

        /*
            Constructor
        */
        Position_Line_Traj_Gen( 
                                Robot_Motion_Mode mode,
                                double initial_time, 
                                double final_time, 
                                const TooN::Vector<3>& final_position, 
                                double final_velocity = 0.0, //direction pf-pi 
                                double initial_velocity = 0.0, //direction pf-pi 
                                double initial_acceleration = 0.0, //direction pf-pi 
                                double final_acceleration = 0.0, //direction pf-pi 
                                const TooN::Vector<3,int>& mask = TooN::Ones 
                                );

        /*
            Copy Constructor
        */
        Position_Line_Traj_Gen( const Position_Line_Traj_Gen& traj ) = default;

        /*
            Clone the object in the heap
        */
        virtual Position_Line_Traj_Gen* clone() const override;

    /*======END CONSTRUCTORS========*/

    /*
        Get Position at time secs
    */
    virtual TooN::Vector<3> getPosition(double secs) const override;

    /*
        Get Velocity at time secs
    */
    virtual TooN::Vector<3> getLinearVelocity(double secs) const override;

    /*
        Get the mask at time secs, if mask[i]=0 then the i-th cartesian coordinate should not be taken into account
    */
    virtual TooN::Vector<3,int> getMask(double secs) const override;

     /*
        initialize the trajectory
    */
    virtual void initialize(const TooN::Matrix<4,4>& b_T_init) override;

};

using Position_Line_Traj_Gen_Ptr = std::unique_ptr<Position_Line_Traj_Gen>;

#endif
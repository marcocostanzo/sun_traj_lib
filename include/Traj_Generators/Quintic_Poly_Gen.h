/*

    Quintic Poly Generator Class
    This class generates a scalar trajectory w. a quintic poly

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

#ifndef QUINTIC_POLY_GEN_H
#define QUINTIC_POLY_GEN_H

#include "Traj_Generator_Interface.h"
#include "GeometryHelper.h"


class Quintic_Poly_Gen : public Traj_Generator_Interface{

    private:

        /*
            No default Constructor
        */
        Quintic_Poly_Gen();

    protected:

        /*
            initial time
        */
        double _initial_time;

        /*
            Poly coeff of p(t)
        */
        TooN::Vector<6> _poly_coeff; 

        /*
            Poly coeff of dp(t)
        */
        TooN::Vector<5> _vel_poly_coeff; 

        /*
            Poly coeff of ddp(t)
        */
        TooN::Vector<4> _acc_poly_coeff; 

        /*
            initial position, final position
            initial velocity, final velocity
            initial acceleration, final acceleration
        */
        double _pi, _pf, _vi, _vf, _aci, _acf; 

    public:

    /*=======CONSTRUCTORS======*/

        /*
            Constructor
        */
        Quintic_Poly_Gen(   
                            double initial_time,
                            double final_time,
                            double initial_position,
                            double final_position,
                            double initial_velocity = 0.0, 
                            double final_velocity = 0.0,
                            double initial_acceleration = 0.0, 
                            double final_acceleration = 0.0
                        );


        /*
            Copy Constructor
        */
        Quintic_Poly_Gen( const Quintic_Poly_Gen& quint ) = default;

        /*
            Clone the object in the heap
        */
        virtual Quintic_Poly_Gen* clone() const override;

    /*=======END CONSTRUCTORS======*/

    /*
        Get Position at time secs
    */
    virtual double getPosition(double secs) const;

    /*
        Get Velocity at time secs
    */
    virtual double getVelocity(double secs) const;

    /*
        Get Acceleration at time secs
    */
    virtual double getAcceleration(double secs) const;

};

using Quintic_Poly_Gen_Ptr = std::unique_ptr<Quintic_Poly_Gen>;

#endif
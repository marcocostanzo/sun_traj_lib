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

#include "Traj_Generators/Quintic_Poly_Gen.h"

using namespace TooN;
using namespace std;



    /*=======CONSTRUCTORS======*/

        /*
            Constructor
        */
        Quintic_Poly_Gen::Quintic_Poly_Gen(   
                                            double initial_time,
                                            double final_time,
                                            double initial_position,
                                            double final_position,
                                            double initial_velocity, 
                                            double final_velocity,
                                            double initial_acceleration, 
                                            double final_acceleration
                        ):
                        Traj_Generator_Interface(final_time),
                        _initial_time(initial_time),
                        _pi(initial_position),
                        _pf(final_position),
                        _vi(initial_velocity),
                        _vf(final_velocity),
                        _aci(initial_acceleration),
                        _acf(final_acceleration)
        {
            //This is the effective total time of motion
            double t = _final_time - _initial_time;
            
            //This coefficients are known
            _poly_coeff[5] = _pi;
            _poly_coeff[4] = _vi;
            _poly_coeff[3] = _aci/2;

            //Prepare pow
            double t_2 = pow(t,2);
            double t_3 = pow(t,3);
            double t_4 = pow(t,4);
            double t_5 = pow(t,5);

            //Construct inverse of A ( A*poly_coeff = B )
            Matrix<3,3> A_inv = Data(
                  6.0/t_5, -3.0/t_4, 1.0/(2.0*t_3),
                -15.0/t_4,  7.0/t_3,      -1.0/t_2,
                 10.0/t_3, -4.0/t_2,   1.0/(2.0*t)
            );
            //Construct B
            Vector<3> B = makeVector(
                 _pf - _pi -  _vi*t - (_aci/2.0)*t_2,
                 _vf - _vi - _aci*t,
                _acf - _aci
            );

            //Calculate poly coeff
            _poly_coeff.slice<0,3>() = A_inv*B;

            //calculate coeff of dp and ddp as polydiff
            _vel_poly_coeff = polydiff(_poly_coeff);
            _acc_poly_coeff = polydiff(_vel_poly_coeff);

        }


        /*
            Clone the object in the heap
        */
        Quintic_Poly_Gen* Quintic_Poly_Gen::clone() const{
            return new Quintic_Poly_Gen(*this);
        }

    /*=======END CONSTRUCTORS======*/

    /*
        Get Position at time secs
    */
    double Quintic_Poly_Gen::getPosition(double secs) const{
        if( secs <= _initial_time ){
            return _pi;
        }
        if( secs >= _final_time ){
            return _pf;
        }
        return polyval(_poly_coeff,secs - _initial_time);
    }

    /*
        Get Velocity at time secs
    */
    double Quintic_Poly_Gen::getVelocity(double secs) const{
        if( secs < _initial_time ){
            return 0.0;
        }
        if( secs > _final_time ){
            return 0.0;
        }
        return polyval(_vel_poly_coeff,secs - _initial_time);
    }

    /*
        Get Acceleration at time secs
    */
    double Quintic_Poly_Gen::getAcceleration(double secs) const{
        if( secs < _initial_time ){
            return 0.0;
        }
        if( secs > _final_time ){
            return 0.0;
        }
        return polyval(_acc_poly_coeff,secs - _initial_time);
    }
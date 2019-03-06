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

#include "Traj_Generators/Line_Segment_Traj.h"

using namespace TooN;
using namespace std;

    /*======CONSTRUCTORS========*/

        /*
            Constructor
        */
        Line_Segment_Traj::Line_Segment_Traj( 
                                                        Robot_Motion_Mode mode,
                                                        double initial_time, 
                                                        double final_time, 
                                                        const Vector<3>& final_position, 
                                                        double final_velocity , //direction pf-pi 
                                                        double initial_velocity, //direction pf-pi 
                                                        double initial_acceleration, //direction pf-pi 
                                                        double final_acceleration, //direction pf-pi 
                                                        const Vector<3,int>& mask 
                                                        ):
                    Position_Traj_Gen( final_time ),
                    _mode(mode),
                    _initial_time(initial_time),
                    _pf(final_position),
                    _vf(final_velocity),
                    _vi(initial_velocity),
                    _aci(initial_acceleration),
                    _acf(final_acceleration),
                    _mask(mask),

                    _direction(unit(_pf-_pi)),

                    //TODO: Use a default constructor here ??
                    _quintic_s(
                        _initial_time,
                        _final_time,
                        0.0,
                        norm(_pf - _pi),
                        _vi, 
                        _vf,
                        _aci, 
                        _acf
                    )
                    {}

        /*
            Clone the object in the heap
        */
        Line_Segment_Traj* Line_Segment_Traj::clone() const{
            return new Line_Segment_Traj(*this);
        }

    /*======END CONSTRUCTORS========*/

    /*
        Get Position at time secs
    */
    Vector<3> Line_Segment_Traj::getPosition(double secs) const{
        return _pi + _quintic_s.getPosition(secs) * _direction;
    }

    /*
        Get Velocity at time secs
    */
    Vector<3> Line_Segment_Traj::getLinearVelocity(double secs) const{
        return _quintic_s.getVelocity(secs) * _direction;
    }

    /*
        Get the mask at time secs, if mask[i]=0 then the i-th cartesian coordinate should not be taken into account
    */
    Vector<3,int> Line_Segment_Traj::getMask(double secs) const{
        return _mask;
    }

     /*
        initialize the trajectory
    */
    void Line_Segment_Traj::initialize(const Matrix<4,4>& b_T_init){

        //get initial position
        _pi = b_T_init.T()[3].slice<0,3>();

        //adjust the final position
        switch (_mode)
        {
            case Robot_Motion_Mode::REL_TOOL : {

                Vector<4> pos_tilde = Ones;
                pos_tilde.slice<0,3>() = _pf;

                pos_tilde = b_T_init * pos_tilde;
                _pf = pos_tilde.slice<0,3>();
                break;
            }
            case Robot_Motion_Mode::REL_BASE : {
                _pf = _pf + _pi;
                break;
            }
            case Robot_Motion_Mode::ABS_BASE : {
                //_pf is ok here
                break;
            }
            
            default:{
                cout << TRAJ_ERROR_COLOR << "[Line_Segment_Traj] Error in initialize() - invalid MODE" CRESET << endl;
                exit(-1);
            }
        }
        
        //reconstruct the quintic object
        /*
        _quintic_s = Quintic_Poly_Gen(
                                    _initial_time,
                                    _final_time,
                                    0.0,
                                    norm(_pf - _pi),
                                    _vi, 
                                    _vf,
                                    _aci, 
                                    _acf
                                );
        */
        _quintic_s.setFinalPosition(norm(_pf - _pi));
        _direction = unit(_pf-_pi);
    }
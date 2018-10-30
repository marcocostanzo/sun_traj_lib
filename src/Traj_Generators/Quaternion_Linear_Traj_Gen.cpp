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

#include "Traj_Generators/Quaternion_Linear_Traj_Gen.h"


using namespace TooN;
using namespace std;

    /*======CONSTRUCTORS=========*/
    
        /*
            Constructor
        */
        Quaternion_Linear_Traj_Gen::Quaternion_Linear_Traj_Gen( 
                                    Robot_Motion_Mode mode,
                                    double initial_time,
                                    double final_time,
                                    const AngVec& ang_vec,
                                    double final_angular_velocity,
                                    double initial_angular_velocity,
                                    double initial_angular_acceleration,
                                    double final_angular_acceleration,
                                    const Vector<3,int>& mask                                     
                                    ):
            Quaternion_Traj_Gen(final_time),
            _mode(mode),
            _initial_time(initial_time),
            _init_angvec_desired(ang_vec),
            _wi(initial_angular_velocity),
            _wf(final_angular_velocity),
            _dwi(initial_angular_acceleration),
            _dwf(final_angular_acceleration),
            _mask(mask),
            _b_rotax( _init_angvec_desired.getVec() ),
            _b_quat_init(/*Unit*/),

            //TODO: Use a default constructor here ??
            _quintic_angle(
                _initial_time,
                _final_time,
                0.0,
                _init_angvec_desired.getAng(),
                _wi, 
                _wf,
                _dwi, 
                _dwf
            )
            {}

        /*
            Clone the object in the heap
        */
        Quaternion_Linear_Traj_Gen* Quaternion_Linear_Traj_Gen::clone() const{
            return new Quaternion_Linear_Traj_Gen(*this);
        }

    /*======END CONSTRUCTORS=========*/

    /*====== GETTERS =========*/

    /*====== END GETTERS =========*/

    /*
        Get AngVec at time secs
    */
    AngVec Quaternion_Linear_Traj_Gen::getAngVec(double secs) const{
        return AngVec( _quintic_angle.getPosition(secs) , _init_angvec_desired.getVec() );
    }

    /*
        Get Quaternion at time secs
    */
    UnitQuaternion Quaternion_Linear_Traj_Gen::getQuaternion(double secs) const{

        UnitQuaternion _init_quat_actual( getAngVec(secs) );
        return _b_quat_init * _init_quat_actual;

    }

    /*
        Get Angular Velocity at time secs
    */
    Vector<3> Quaternion_Linear_Traj_Gen::getAngularVelocity(double secs) const{
        return _quintic_angle.getVelocity(secs) * _b_rotax;
    }

    /*
        Get the mask at time secs, if mask[i]=0 then the i-th cartesian coordinate should not be taken into account
        Warning: these are UnitQuaternion !!
    */
    Vector<3,int> Quaternion_Linear_Traj_Gen::getMask(double secs) const{
        return _mask;
    }

    /*
        initialize the trajectory
    */
    void Quaternion_Linear_Traj_Gen::initialize(const Matrix<4,4>& b_T_init){

        _b_quat_init = UnitQuaternion(b_T_init);

        switch (_mode)
        {
            case Robot_Motion_Mode::REL_TOOL : {

                //In this case the _init_angvec_desired is already in the correct frame
                //_b_rotax is the rotation axis of the _init_angvec_desired rotated in the {base} frame

                _b_rotax = b_T_init.slice<0,0,3,3>() * _init_angvec_desired.getVec();
            
                break;
            }
            case Robot_Motion_Mode::REL_BASE : {

                //In this case the _init_angvec_desired has the axis in the {base} frame
                //  but the angle is already a delta_angle
                
                //_b_rotax is the axis used as input in the constructor
                _b_rotax = _init_angvec_desired.getVec();

                //The correct axis of _init_angvec_desired is _b_rotax rotated in the {initial} frame
                _init_angvec_desired.setVec( b_T_init.slice<0,0,3,3>().T() * _b_rotax );

                break;
            }
            case Robot_Motion_Mode::ABS_BASE : {

                //In this case the _init_angvec_desired has the axis in the {base} frame
                //  and the angle is a absolute angle w.r.t. base frame

                //_init_angvec_desired at the left of this line has to been read as _b_angvec_desired
                UnitQuaternion init_quat_desired = inv(_b_quat_init) * UnitQuaternion(_init_angvec_desired);
                //Here the correct _init_angvec_desired is computed
                _init_angvec_desired = init_quat_desired.toangvec();

                //Now it is necessary reconstruct the _quintic_angle generator using the new delta_angle
                /*
                _quintic_angle = Quintic_Poly_Gen(
                                                    _initial_time,
                                                    _final_time,
                                                    0.0,
                                                    _init_angvec_desired.getAng(),
                                                    _wi, 
                                                    _wf,
                                                    _dwi, 
                                                    _dwf
                                                );*/
                _quintic_angle.setFinalPosition(_init_angvec_desired.getAng());

                //Now the situation is the same of the case Robot_Motion_Mode::REL_TOOL
                _b_rotax = b_T_init.slice<0,0,3,3>() * _init_angvec_desired.getVec();

                break;
            }
            
            default:{
                cout << TRAJ_ERROR_COLOR << "[Quaternion_Linear_Traj_Gen] Error in initialize() - invalid MODE" CRESET << endl;
                exit(-1);
            }
        }

    }
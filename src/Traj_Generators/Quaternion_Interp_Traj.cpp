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
    Full Constructor
*/
Quaternion_Interp_Traj::Quaternion_Interp_Traj(const UnitQuaternion& qi, const UnitQuaternion& qf, const Scalar_Traj_Interface& traj_s)
    :Quaternion_Traj_Interface( NAN, NAN ),
    _qi(qi),
    _qf(qf),
    _traj_s( traj_s.clone() )
    {}

/*
    Copy Constructor
*/
Quaternion_Interp_Traj( const Quaternion_Interp_Traj& traj )
    :Quaternion_Traj_Interface( traj ),
    Quaternion_Interp_Traj(
        traj._qi, 
        traj._qf, 
        traj._traj_s->clone()
        )
    {}

/*
    Clone the object in the heap
*/
Quaternion_Interp_Traj* clone() const {
    return new Quaternion_Interp_Traj(*this);
}

/*======END CONSTRUCTORS=========*/

/*
    TODO
*/
UnitQuaternion getInitialQuaternion() const{
    return _qi;
}

/*
    TODO
*/
AngVec getInitialAngVec() const{
    return getInitialQuaternion().toangvec();
}

/*
    TODO
*/
UnitQuaternion getFinalQuaternion() const{
    return _qf;
}

/*
    TODO
*/
AngVec getFinalAngVec() const{
    return getFinalQuaternion().toangvec();
}

/*
    Get the final time instant
*/
double getFinalTime() const{
    return _traj_s->getFinalTime();
}

/*
    Get the initial time instant
*/
double getInitialTime() const{
    return _traj_s->getInitialTime();
}

/*====== END GETTERS =========*/

/*====== SETTERS =========*/

/*
    Change the initial time instant (translate the trajectory in the time)
*/
void changeInitialTime(double initial_time) {
    _traj_s->changeInitialTime(initial_time);
} 

/*====== END SETTERS =========*/

/*
    Get Position at time secs
*/
UnitQuaternion getQuaternion(double secs) const {
    return _qi.interp( _qf, _traj_s->getPosition(secs), true);
}


/*
    Get AngVec at time secs
*/
AngVec getAngVec(double secs) const{
    return getQuaternion(secs).toangvec();
}

/*
    Get Velocity at time secs
*/
Vector<3> getVelocity(double secs) const {
    return _traj_s->getVelocity(secs)
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
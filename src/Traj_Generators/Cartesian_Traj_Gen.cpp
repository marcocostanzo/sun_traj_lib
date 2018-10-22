/*

    Cartesian Trajectory Generator Class
    This class is an interface to generate arbitrary cartesian trajectory in the cartesian space
    the angular position is giver as UnitQuaterion

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


#include "Traj_Generators/Cartesian_Traj_Gen.h"

using namespace TooN;
using namespace std;

    /*======CONSTRUCTORS=========*/
    
        /*
            Constructor with low level position and quaterion generators as input
        */
        Cartesian_Traj_Gen::Cartesian_Traj_Gen( const Position_Traj_Gen& pos_gen, const Quaternion_Traj_Gen& ang_gen ):
            Traj_Generator_Interface(NAN)
        {
            _pos_gen = Position_Traj_Gen_Ptr( pos_gen.clone() );
            _ang_gen = Quaternion_Traj_Gen_Ptr( ang_gen.clone() );
        }

        /*
            Copy Constructor
        */
        Cartesian_Traj_Gen::Cartesian_Traj_Gen( const Cartesian_Traj_Gen& traj ):
            Traj_Generator_Interface(NAN)
        {
            _pos_gen = Position_Traj_Gen_Ptr( traj._pos_gen->clone() );
            _ang_gen = Quaternion_Traj_Gen_Ptr( traj._ang_gen->clone() );
        }

        /*
            Clone the object in the heap
        */
        Cartesian_Traj_Gen* Cartesian_Traj_Gen::clone() const{
            return new Cartesian_Traj_Gen( *this );
        }

    /*======END CONSTRUCTORS=========*/

    /*====== GETTERS =========*/

    /*
        Get the total time
    */
    double Cartesian_Traj_Gen::getFinalTime() const{
        return max( _pos_gen->getFinalTime(), _ang_gen->getFinalTime() );
    }

    /*
        Get the time left
    */
    double Cartesian_Traj_Gen::getTimeLeft(double secs) const{
        return max( _pos_gen->getTimeLeft(secs), _ang_gen->getTimeLeft(secs) );
    }

    /*====== END GETTERS =========*/

    /*
        Get Position at time secs
    */
    Vector<3> Cartesian_Traj_Gen::getPosition(double secs) const{
        return _pos_gen->getPosition(secs);
    }

    /*
        Get Linear Velocity at time secs
    */
    Vector<3> Cartesian_Traj_Gen::getLinearVelocity(double secs) const{
        return _pos_gen->getLinearVelocity(secs);
    }

    /*
        Get Quaternion at time secs
    */
    UnitQuaternion Cartesian_Traj_Gen::getQuaternion(double secs) const{
        return _ang_gen->getQuaternion(secs);
    }

    /*
        Get Angular Velocity at time secs
    */
    Vector<3> Cartesian_Traj_Gen::getAngularVelocity(double secs) const{
        return _ang_gen->getAngularVelocity(secs);
    }

    /*
        Get the mask at time secs, if mask[i]=0 then the i-th cartesian coordinate should not be taken into account
        Warning: UnitQuaternion are used for angular position !!
    */
    Vector<6,int> Cartesian_Traj_Gen::getMask(double secs) const{
        Vector<6,int> outmask;
        outmask.slice<0,3>() = _pos_gen->getMask(secs);
        outmask.slice<3,3>() = _ang_gen->getMask(secs);
        return outmask;
    }

    /*
        return true if the trajectory is compleate at time secs
    */
    bool Cartesian_Traj_Gen::isCompleate(double secs) const{
        return ( _pos_gen->isCompleate(secs) && _ang_gen->isCompleate(secs));
    }

    /*
        initialize the trajectory
    */
    void Cartesian_Traj_Gen::initialize(const TooN::Matrix<4,4>& b_T_init){
        _pos_gen->initialize( b_T_init );
        _ang_gen->initialize( b_T_init );
    }
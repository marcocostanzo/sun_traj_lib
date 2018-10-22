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


#ifndef CARTESIAN_TRAJ_GEN_H
#define CARTESIAN_TRAJ_GEN_H

#include "Traj_Generators/Position_Traj_Gen.h"
#include "Traj_Generators/Quaternion_Traj_Gen.h"

class Cartesian_Traj_Gen : public Traj_Generator_Interface {

    private:

        /*
            Avoid Default constructor
        */
        Cartesian_Traj_Gen();

    protected:

        /*
            Position Generator
        */
        Position_Traj_Gen_Ptr _pos_gen;

        /*
            Quaternion Generator
        */
        Quaternion_Traj_Gen_Ptr _ang_gen;

    public:

    /*======CONSTRUCTORS=========*/
    
        /*
            Constructor with low level position and quaterion generators as input
        */
        Cartesian_Traj_Gen( const Position_Traj_Gen& pos_gen, const Quaternion_Traj_Gen& ang_gen );

        /*
            Copy Constructor
        */
        Cartesian_Traj_Gen( const Cartesian_Traj_Gen& traj );

        /*
            Clone the object in the heap
        */
        virtual Cartesian_Traj_Gen* clone() const;

    /*======END CONSTRUCTORS=========*/

    /*====== GETTERS =========*/

    /*
        Get the total time
    */
    virtual double getFinalTime() const;

    /*
        Get the time left
    */
    virtual double getTimeLeft(double secs) const;

    /*====== END GETTERS =========*/

    /*
        Get Position at time secs
    */
    virtual TooN::Vector<3> getPosition(double secs) const;

    /*
        Get Linear Velocity at time secs
    */
    virtual TooN::Vector<3> getLinearVelocity(double secs) const;

    /*
        Get Quaternion at time secs
    */
    virtual UnitQuaternion getQuaternion(double secs) const;

    /*
        Get Angular Velocity at time secs
    */
    virtual TooN::Vector<3> getAngularVelocity(double secs) const;

    /*
        Get the mask at time secs, if mask[i]=0 then the i-th cartesian coordinate should not be taken into account
        Warning: UnitQuaternion are used for angular position !!
    */
    virtual TooN::Vector<6,int> getMask(double secs) const;

    /*
        return true if the trajectory is compleate at time secs
    */
    virtual bool isCompleate(double secs) const;

    /*
        initialize the trajectory
    */
    virtual void initialize(const TooN::Matrix<4,4>& b_T_init);

};//END CLASS

using Cartesian_Traj_Gen_Ptr = std::unique_ptr<Cartesian_Traj_Gen>;

#endif
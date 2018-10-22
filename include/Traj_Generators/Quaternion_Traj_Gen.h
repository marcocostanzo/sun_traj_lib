/*

    Quaternion Trajectory Generator Class
    This class is an interface to generate arbitrary UnitQuaternion trajectory in the cartesian space

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

#ifndef QUATERNION_TRAJ_GEN_H
#define QUATERNION_TRAJ_GEN_H

#include "TooN/TooN.h"
#include "UnitQuaternion.h"
#include "Traj_Generators/Traj_Generator_Interface.h"

class Quaternion_Traj_Gen : public Traj_Generator_Interface {

    private:

        /*
            Avoid Default constructor
        */
        Quaternion_Traj_Gen();

    protected:

    public:

    /*======CONSTRUCTORS=========*/
    
        /*
            Constructor with total time as input
        */
        Quaternion_Traj_Gen( double final_time );

        /*
            Clone the object in the heap
        */
        virtual Quaternion_Traj_Gen* clone() const = 0;

        //Quaternion_Traj_Gen( const Quaternion_Traj_Gen& traj );

    /*======END CONSTRUCTORS=========*/

    /*====== GETTERS =========*/

    /*====== END GETTERS =========*/

    /*
        Get Quaternion at time secs
    */
    virtual UnitQuaternion getQuaternion(double secs) const = 0;

    /*
        Get Angular Velocity at time secs
    */
    virtual TooN::Vector<3> getAngularVelocity(double secs) const = 0;

    /*
        Get the mask at time secs, if mask[i]=0 then the i-th cartesian coordinate should not be taken into account
        Warning: these are UnitQuaternion !!
    */
    virtual TooN::Vector<3,int> getMask(double secs) const;

    /*
        initialize the trajectory
    */
    virtual void initialize(const TooN::Matrix<4,4>& b_T_init) = 0;

};//END CLASS

using Quaternion_Traj_Gen_Ptr = std::unique_ptr<Quaternion_Traj_Gen>;

#endif
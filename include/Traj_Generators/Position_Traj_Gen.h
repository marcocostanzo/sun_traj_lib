/*

    Position Trajectory Generator Class
    This class is an interface to generate arbitrary position trajectory in the cartesian space

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

#ifndef POSITION_TRAJ_GEN_H
#define POSITION_TRAJ_GEN_H

#include "TooN/TooN.h"
#include "Traj_Generators/Traj_Generator_Interface.h"

class Position_Traj_Gen : public Traj_Generator_Interface {

    private:

        /*
            Avoid Default constructor
        */
        Position_Traj_Gen();

    protected:

    public:

    /*======CONSTRUCTORS=========*/
    
        /*
            Constructor with total time as input
        */
        Position_Traj_Gen( double final_time );

        //Position_Traj_Gen( const Position_Traj_Gen& traj );

        /*
            Clone the object in the heap
        */
        virtual Position_Traj_Gen* clone() const = 0;

    /*======END CONSTRUCTORS=========*/

    /*====== GETTERS =========*/

    /*====== END GETTERS =========*/

    
    /*
        Get Position at time secs
    */
    virtual TooN::Vector<3> getPosition(double secs) const = 0;

    /*
        Get Velocity at time secs
    */
    virtual TooN::Vector<3> getLinearVelocity(double secs) const = 0;

    /*
        Get the mask at time secs, if mask[i]=0 then the i-th cartesian coordinate should not be taken into account
    */
    virtual TooN::Vector<3,int> getMask(double secs) const;

    /*
        initialize the trajectory
    */
    virtual void initialize(const TooN::Matrix<4,4>& b_T_init) = 0;

};//END CLASS

using Position_Traj_Gen_Ptr = std::unique_ptr<Position_Traj_Gen>;

#endif
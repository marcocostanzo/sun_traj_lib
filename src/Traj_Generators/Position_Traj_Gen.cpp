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

#include "Traj_Generators/Position_Traj_Gen.h"

using namespace TooN;

    /*======CONSTRUCTORS=========*/
    
        /*
            Constructor with total time as input
        */
        Position_Traj_Gen::Position_Traj_Gen( double final_time ):
            Traj_Generator_Interface(final_time){}

        //Position_Traj_Gen( const Position_Traj_Gen& traj );

    /*======END CONSTRUCTORS=========*/

    /*====== GETTERS =========*/

    /*====== END GETTERS =========*/

    /*
        Get the mask at time secs, if mask[i]=0 then the i-th cartesian coordinate should not be taken into account
    */
    Vector<3,int> Position_Traj_Gen::getMask(double secs) const{
        return Ones;
    }
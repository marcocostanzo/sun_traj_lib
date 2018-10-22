/*

    Trajectory Generator Interface Class
    This class is a general interface to trajectory generators classes

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

#include "Traj_Generators/Traj_Generator_Interface.h"

    /*======CONSTRUCTORS=========*/
    
        /*
            Constructor with total time as input
        */
        Traj_Generator_Interface::Traj_Generator_Interface( double final_time ):
            _final_time(_final_time){}

        //Traj_Generator_Interface( const Traj_Generator_Interface& traj );

    /*======END CONSTRUCTORS=========*/

    /*====== GETTERS =========*/

    /*
        Get the total time
    */
    double Traj_Generator_Interface::getFinalTime() const{
        return _final_time;
    }

    /*
        Get the time left
    */
    double Traj_Generator_Interface::getTimeLeft(double secs) const{
        return (_final_time - secs);
    }

    /*====== END GETTERS =========*/

    /*
        return true if the trajectory is compleate at time secs
    */
    bool Traj_Generator_Interface::isCompleate(double secs) const{
        return ( secs >=  _final_time);
    }
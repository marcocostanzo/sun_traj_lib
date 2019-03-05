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

using namespace std;


/*======CONSTRUCTORS=========*/

/*
    Constructor with duration and initial time as input
*/
Traj_Generator_Interface::Traj_Generator_Interface( double duration, double initial_time ):
    _initial_time(initial_time){
        if(duration < 0.0){
            cout << TRAJ_ERROR_COLOR "Error in Traj_Generator_Interface( double duration, double initial_time) duration has to be >= 0" CRESET << endl;
            exit(-1);
        }
        _final_time = _initial_time + duration;
    }
 
    /*======END CONSTRUCTORS=========*/

    /*====== GETTERS =========*/

    /*
        Get the final time instant
    */
    double Traj_Generator_Interface::getFinalTime() const{
        return _final_time;
    }

    /*
        Get the initial time instant
    */
    double Traj_Generator_Interface::getInitialTime() const{
        return _initial_time;
    }

    /*
        Get the duration
    */
    double Traj_Generator_Interface::getDuration() const{
        return getFinalTime() - getInitialTime();
    }

    /*
        Get the time left
    */
    double Traj_Generator_Interface::getTimeLeft(double secs) const{
        return (getFinalTime() - secs);
    }

    /*====== END GETTERS =========*/

    /*====== SETTERS =========*/

     /*
        Change the initial time instant (translate the trajectory in the time)
    */
    void Traj_Generator_Interface::changeInitialTime(double initial_time){
        _final_time = initial_time + getDuration();
        _initial_time = initial_time;
    }

    /*====== END SETTERS =========*/

    /*
        return true if the trajectory is compleate at time secs
    */
    bool Traj_Generator_Interface::isCompleate(double secs) const{
        return ( secs >=  _final_time);
    }

    /*
        return true if the trajectory is started at time secs
    */
    bool Traj_Generator_Interface::isStarted(double secs) const{
        return ( secs < _initial_time );
    }
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

#ifndef TRAJ_GEN_INTERFACE_H
#define TRAJ_GEN_INTERFACE_H

#include "Traj_Gen_common.h"
#include<iostream>
#include <memory>

class Traj_Generator_Interface{

    private:

        /*
            No default Constructor
        */
        Traj_Generator_Interface();

    protected:

        /*
            initial time of the trajectory
        */
        double _initial_time;

        /*
            final time of the trajectory
        */
        double _final_time;

    public:

    /*======CONSTRUCTORS=========*/
    
        /*
            Constructor with duration and initial time as input
        */
        Traj_Generator_Interface( double duration, double initial_time = 0.0 );

        //Traj_Generator_Interface( const Traj_Generator_Interface& traj );

        /*
            Clone the object in the heap
        */
        virtual Traj_Generator_Interface* clone() const = 0;

    /*======END CONSTRUCTORS=========*/

    /*====== GETTERS =========*/

    /*
        Get the final time instant
    */
    virtual double getFinalTime() const;

    /*
        Get the initial time instant
    */
    virtual double getInitialTime() const;

    /*
        Get the duration
    */
    virtual double getDuration() const;

    /*
        Get the time left
    */
    virtual double getTimeLeft(double secs) const;

    /*====== END GETTERS =========*/

    /*====== SETTERS =========*/

     /*
        Change the initial time instant (translate the trajectory in the time)
    */
    virtual void changeInitialTime(double initial_time); 

    /*====== END SETTERS =========*/

    /*
        return true if the trajectory is compleate at time secs
    */
    virtual bool isCompleate(double secs) const;

    /*
        return true if the trajectory is started at time secs
    */
    virtual bool isStarted(double secs) const;

};//END CLASS

using Traj_Generator_Interface_Ptr = std::unique_ptr<Traj_Generator_Interface>;

#endif
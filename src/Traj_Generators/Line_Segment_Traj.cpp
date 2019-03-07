/*

    Position Line Trajectory Generator Class
    This class is generates a line trajectory in the cartesian space

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

#include "Traj_Generators/Line_Segment_Traj.h"

using namespace TooN;
using namespace std;

/*======CONSTRUCTORS========*/

/*
    Full Constructor
*/
Line_Segment_Traj::Line_Segment_Traj(const TooN::Vector<3>& pi, const TooN::Vector<3>& pf, const Scalar_Traj_Interface& traj_s)
    :Position_Traj_Interface(NAN, NAN),
    _pi(pi),
    _pf(pf),
    _traj_s( traj_s.clone() )
    {}

/*
    Copy Constructor
*/
Line_Segment_Traj::Line_Segment_Traj( const Line_Segment_Traj& traj )
    :Position_Traj_Interface( traj ),
    _traj_s(traj._traj_s->clone()){}

/*
    Clone the object in the heap
*/
Line_Segment_Traj* Line_Segment_Traj::Line_Segment_Traj::clone() const {
    return new Line_Segment_Traj(*this);
}

/*======END CONSTRUCTORS========*/

/*====== GETTERS ========*/

/*
    TODO
*/
Vector<3> Line_Segment_Traj::getInitialPoint() const{
    return _pi;
}

/*
    TODO
*/
Vector<3> Line_Segment_Traj::getFinalPoint() const{
    return _pf;
}

/*
    Get the final time instant
*/
double Line_Segment_Traj::getFinalTime() const{
    return _traj_s->getFinalTime();
}

/*
    Get the initial time instant
*/
double Line_Segment_Traj::getInitialTime() const{
    return _traj_s->getFinalTime();
}

/*====== END GETTERS ========*/

/*====== SETTERS =========*/

/*
    Change the initial time instant (translate the trajectory in the time)
*/
void Line_Segment_Traj::changeInitialTime(double initial_time) {
    _traj_s->changeInitialTime(initial_time);
} 

/*====== END SETTERS =========*/

/*====== RUNNERS =========*/

/*
    Get Position at time secs
*/
Vector<3> Line_Segment_Traj::getPosition(double secs) const{
    return _pi + (_traj_s->getPosition(secs) * (_pf - _pi));
}

/*
    Get Velocity at time secs
*/
Vector<3> Line_Segment_Traj::getVelocity(double secs) const{
    return _traj_s->getVelocity(secs) * (_pf - _pi);
}

/*
    Get Acceleration at time secs
*/
Vector<3> Line_Segment_Traj::getAcceleration(double secs) const{
    return _traj_s->getAcceleration(secs) * (_pf - _pi);
}

/*====== END RUNNERS =========*/
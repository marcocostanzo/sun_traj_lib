/*

    Robot Prismatic Link

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

#ifndef ROBOTLINKPRISMATIC_H
#define ROBOTLINKPRISMATIC_H

#include "RobotLink.h"


class RobotLinkPrismatic : public RobotLink{

    private:

        RobotLinkPrismatic(); //No Default Constructor

    protected:

    public:

        /*=============CONSTRUCTORS===========*/

        //Full Constructor
        RobotLinkPrismatic( double a, double alpha, double d, double theta, 
                            double offset, bool flip, 
                            double robot2dh_offset, double robot2dh_sign, 
                            double DHJoint_limit_lower, double DHJoint_limit_higher, 
                            double RobotJoint_limit_lower, double RobotJoint_limit_higher, 
                            double velocity_limit_lower,
                            std::string name );

        RobotLinkPrismatic( double a, double alpha, double d, double theta, double offset, bool flip);

        RobotLinkPrismatic( double a, double alpha, double d, double theta);

        /*=======END CONSTRUCTORS===========*/

        /*
            Clone the object
        */
        virtual RobotLinkPrismatic* clone() const override;

        /*
            return the link offset
            Note:
                - For prismatic link this is the joint variable,
                  in that case this function returns NaN
        */
        virtual double getDH_d() const override;

        /*
            TODO
            ERROR IF LINK IS PRISMATIC
        */
        virtual void setDH_d( double d ) override;

        /*
            TODO
        */
        virtual void display() const override;

        /*
                Retrun the joint type
                'p' = prismatic
                'r' = revolute
        */
        virtual char type() const override;

        /*
            Compute the link transform matrix
        */
        virtual TooN::Matrix<4,4> A( double q_DH ) const override;



};//end class

using RobotLinkPrismaticPtr = std::unique_ptr<RobotLinkPrismatic>;

bool isPrismatic( const RobotLink& l );

#endif

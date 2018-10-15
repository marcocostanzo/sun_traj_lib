
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
                            double velocity_limit_lower, double velocity_limit_higher,
                            std::string name );

        RobotLinkPrismatic( double a, double alpha, double d, double theta, double offset, bool flip);

        RobotLinkPrismatic( double a, double alpha, double d, double theta);

        /*=======END CONSTRUCTORS===========*/

        /*
            return the link offset
            Note:
                - For prismatic link this is the joint variable,
                  in that case this function returns NaN
        */
        virtual double getDH_d() const;

        /*
            TODO
            ERROR IF LINK IS PRISMATIC
        */
        virtual void setDH_d( double d );

        /*
            TODO
        */
        virtual void display() const;

        /*
                Retrun the joint type
                'p' = prismatic
                'r' = revolute
        */
        virtual char type() const;

        /*
            Compute the link transform matrix
        */
        virtual TooN::Matrix<4,4> A( double q_DH ) const;



};//end class

bool isPrismatic( const RobotLink& l );

#endif

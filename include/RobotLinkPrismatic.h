
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


#ifndef ROBOTLINK_H
#define ROBOTLINK_H

#include "TooN/TooN.h"
#include <memory>
 
#define ROBOT_ERROR_COLOR       "\033[1m\033[31m"      /* Bold Red */
#define ROBOT_WARNING_COLOR     "\033[1m\033[33m"      /* Bold Yellow */
#define ROBOT_CRESET            "\033[0m"

class RobotLink{

    private:

        RobotLink(); //No Default Constructor

    protected:

        //Kinematic parameters (DH)
        double _a;      //link length
        double _alpha;  //link twist
        double _d;      //link offset
        double _theta;  //link angle

        double _offset; //joint coordinate offset
        bool _flip;     //joint moves in opposite direction
        //////////////////////////////////////////////////

        //Robot-DH Conversion
        double _robot2dh_offset;    //Offset between robot and DH convention
        double _robot2dh_flip;      //joint in Robot convention moves in opposite direction
        ///////////////////////////////////////////////

        //Safety Vars
        double _DHJoint_limit_lower, _DHJoint_limit_higher;       //SoftLimits in DH convention
        double _RobotJoint_limit_lower, _RobotJoint_limit_higher; //HardLimits in Robot convention
        double _velocity_limit; //HardLimits in Robot convention
        //////////////////////////////////////////////

        std::string _name;  //joint name

        /*======CONSTRUCTORS======*/

        //Full Constructor
        RobotLink(  double a, double alpha, double d, double theta, 
                    double offset, bool flip, 
                    double robot2dh_offset, bool _robot2dh_flip, 
                    double DHJoint_limit_lower, double DHJoint_limit_higher, 
                    double RobotJoint_limit_lower, double RobotJoint_limit_higher, 
                    double velocity_limit,
                    std::string name );

        RobotLink( double a, double alpha, double d, double theta, double offset, bool flip);

        RobotLink( double a, double alpha, double d, double theta);

        //RobotLink( const RobotLink& rl) = default;

        /*======END CONSTRUCTORS======*/

        //========Varie=======//
        
        static void checkLowerHigher( double lower, double higher );

        virtual TooN::Matrix<4,4> A_internal( double theta, double d ) const;

        virtual double DH_revert_offset( double q_DH ) const;

        //===================//

    public:

        //========GETTERS==============//

        /*
            return the link length
        */
        virtual double getDH_a() const;

        /*
            return the link twist
        */
        virtual double getDH_alpha() const;

        /*
            return the link offset
            Note:
                - For prismatic link this is the joint variable,
                  in that case this function returns NaN
        */
        virtual double getDH_d() const;

        /*
            return the link angle
            Note:
                - For revolute link this is the joint variable,
                  in that case this function returns NaN
        */
        virtual double getDH_theta() const;

        /*
            Return the offset in the joint variable
            Note:
                -This is NOT the offset in the robot convention!
        */
        virtual double getDH_offset() const;

        /*
            Return true joint moves in opposite direction in the DH convention
            Note:
                -This is NOT the sign in the robot convention!
        */
        virtual bool getDH_flip() const;

        /*
            Return the offset between the robot and DH convention
        */
        virtual double getRobot2DH_offset() const;

        /*
            Return the sign between robot and DH conventions
        */
        virtual bool getRobot2DH_flip() const;

        /*
            TODO
        */
        virtual TooN::Vector<2> getDHJoint_limits() const;

        /*
            TODO
        */
        virtual TooN::Vector<2> getRobotJoint_limits() const;
        
        /*
            TODO
        */
        virtual double getVelocity_limit() const;

        /*
            Return the Joint Name
        */
        virtual std::string getName() const;

        /*
            Clone the object
        */
        virtual RobotLink* clone() const = 0;

        //======END GETTERS===========//

        //========SETTERS==============//

        /*
            TODO
        */
        virtual void setDH_a( double a );

        /*
            TODO
        */
        virtual void setDH_alpha( double alpha );

        /*
            TODO
            ERROR IF LINK IS PRISMATIC
        */
        virtual void setDH_d( double d );

        /*
            TODO
            ERROR IF LINK IS REVOLUTE
        */
        virtual void setDH_theta( double theta );

        /*
            TODO
        */
        virtual double setDH_offset( double offset );

        /*
            TODO
        */
        virtual void setDH_flip( bool flip );

        /*
            TODO
        */
        virtual void setRobot2DH_offset( double offset );

        /*
            TODO
        */
        virtual void setRobot2DH_flip( bool flip );

        /*
            TODO
        */
        virtual void setDHJoint_limits( double lower, double higher);

        /*
            TODO
        */
        virtual void setDHJoint_limits( TooN::Vector<2> limits );

        /*
            TODO
        */
        virtual void setRobotJoint_limits( double lower, double higher);

        /*
            TODO
        */
        virtual void setRobotJoint_limits( TooN::Vector<2> limits );

        /*
            TODO
        */    
        virtual void setVelocity_limit( double velocity_limit);

        /*
            TODO
        */
        virtual void setName( std::string name );

        //======END SETTERS===========//

        /*
            TODO
        */
        virtual void display() const;

        /*
            Retrun the joint type
            'p' = prismatic
            'r' = revolute
        */
        virtual char type() const = 0;

        /*
            Compute the link transform matrix
        */
        virtual TooN::Matrix<4,4> A( double q_DH ) const = 0;

        /*
            return True if the input q_DH (in DH convention) exceeds the softDHLimits
        */
        virtual bool exceededJointDHLimits(double q_DH) const;

        /*
            return True if the input q_Robot (in Robot convention) exceeds the HardRobotLimits
        */
        virtual bool exceededJointRobotLimits(double q_Robot) const;

        /*
            return True if the input q_vel exceeds the velocity limit
        */
        virtual bool exceededJointVelocity(double q_vel) const;

        /*
            TODO
        */
        virtual double joint_Robot2DH( double q_Robot ) const;

        /*
            TODO
        */
        virtual double joint_DH2Robot( double q_DH ) const;

        /*
            TODO
        */
        virtual double jointvel_Robot2DH( double q_vel_Robot ) const;

        /*
            TODO
        */
        virtual double jointvel_DH2Robot( double q_vel_DH ) const;


};//end class

using RobotLinkPtr = std::unique_ptr<RobotLink>;

#endif
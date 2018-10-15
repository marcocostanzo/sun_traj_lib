
#include "RobotLink.h"

using namespace TooN;
using namespace std;


        /*======CONSTRUCTORS======*/

        //Full Constructor
        RobotLink::RobotLink(  double a, double alpha, double d, double theta, 
                    double offset, bool flip, 
                    double robot2dh_offset, bool robot2dh_flip, 
                    double DHJoint_limit_lower, double DHJoint_limit_higher, 
                    double RobotJoint_limit_lower, double RobotJoint_limit_higher, 
                    double velocity_limit_lower, double velocity_limit_higher,
                    string name )
                    {
            _a = a;
            _alpha = alpha;
            _d = d;
            _theta = theta;
            _offset = offset;
            _flip = flip;
            _robot2dh_offset = robot2dh_offset;
            setRobot2DH_flip(robot2dh_flip);
            setDHJoint_limits( DHJoint_limit_lower, DHJoint_limit_higher );
            setRobotJoint_limits( RobotJoint_limit_lower, RobotJoint_limit_higher );
            setVelocity_limits( velocity_limit_lower, velocity_limit_higher );
            _name = name;
        }

        RobotLink::RobotLink( double a, double alpha, double d, double theta, double offset, bool flip):
            RobotLink(  a, alpha, d, theta, 
                        offset, flip, 
                        0.0, false, 
                        -INFINITY, INFINITY, 
                        -INFINITY, INFINITY, 
                        -INFINITY, INFINITY,
                        "Joint unnamed" )
                        {}

        RobotLink::RobotLink( double a, double alpha, double d, double theta):
            RobotLink( a, alpha, d, theta, 0.0, false)
            {}

        /*======END CONSTRUCTORS======*/

        //========Varie=======//
        
        void RobotLink::checkLowerHigher( double lower, double higher ){
            if( lower > higher ){
                cout << ROBOT_ERROR_COLOR "[RobotLink] Error in checkLowerHigher( double lower, double higher ): lower > higher " ROBOT_CRESET << endl;
            }
        }

        Matrix<4,4> RobotLink::A_internal( double theta, double d ) const{

            double sa = sin(_alpha); 
            double ca = cos(_alpha);
            
            double st = sin(theta);
            double ct = cos(theta);
            
            //standard DH
             return Data(
                             ct, -st*ca,  st*sa, _a*ct,
                             st,  ct*ca, -ct*sa, _a*st,
                            0.0,     sa,     ca,    d,
                            0.0,    0.0,    0.0,   1.0
             );
        }

        double RobotLink::DH_revert_offset( double q_DH ) const{
            if(_flip)
                return (-q_DH - _offset);
            else
                return ( q_DH - _offset);
        }

        //===================//

        //========GETTERS==============//

        /*
            return the link length
        */
        double RobotLink::getDH_a() const{
            return _a;
        }

        /*
            return the link twist
        */
        double RobotLink::getDH_alpha() const{
            return _alpha;
        }

        /*
            return the link offset
            Note:
                - For prismatic link this is the joint variable,
                  in that case this function returns NaN
        */
        double RobotLink::getDH_d() const{
            return _d;
        }

        /*
            return the link angle
            Note:
                - For revolute link this is the joint variable,
                  in that case this function returns NaN
        */
        double RobotLink::getDH_theta() const{
            return _theta;
        }

        /*
            Return the offset in the joint variable
            Note:
                -This is NOT the offset in the robot convention!
        */
        double RobotLink::getDH_offset() const{
            return _offset;
        }

        /*
            Return true joint moves in opposite direction in the DH convention
            Note:
                -This is NOT the sign in the robot convention!
        */
        bool RobotLink::getDH_flip() const{
            return _flip;
        }

        /*
            Return the offset between the robot and DH convention
        */
        double RobotLink::getRobot2DH_offset() const{
            return _robot2dh_offset;
        }

        /*
            Return the sign between robot and DH conventions
        */
        bool RobotLink::getRobot2DH_flip() const{
            return _robot2dh_flip;
        }

        /*
            TODO
        */
        Vector<2> RobotLink::getDHJoint_limits() const{
            return makeVector( _DHJoint_limit_lower, _DHJoint_limit_higher );
        }

        /*
            TODO
        */
        Vector<2> RobotLink::getRobotJoint_limits() const{
            return makeVector( _RobotJoint_limit_lower, _RobotJoint_limit_higher );
        }
        
        /*
            TODO
        */
        Vector<2> RobotLink::getVelocity_limits() const{
            return makeVector( _velocity_limit_lower, _velocity_limit_higher );
        }

        /*
            Return the Joint Name
        */
        string RobotLink::getName() const{
            return _name;
        }

        //======END GETTERS===========//

        //========SETTERS==============//

        /*
            TODO
        */
        void RobotLink::setDH_a( double a ){
            _a = a;
        }

        /*
            TODO
        */
        void RobotLink::setDH_alpha( double alpha ){
            _alpha = alpha;
        }

        /*
            TODO
            ERROR IF LINK IS PRISMATIC
        */
        void RobotLink::setDH_d( double d ){
            _d = d;
        }

        /*
            TODO
            ERROR IF LINK IS REVOLUTE
        */
        void RobotLink::setDH_theta( double theta ){
            _theta = theta;
        }

        /*
            TODO
        */
        double RobotLink::setDH_offset( double offset ){
            _offset = offset;
        }

        /*
            TODO
        */
        void RobotLink::setDH_flip( bool flip ){
            _flip = flip;
        }

        /*
            TODO
        */
        void RobotLink::setRobot2DH_offset( double offset ){
            _robot2dh_offset = offset;
        }

        /*
            TODO
        */
        void RobotLink::setRobot2DH_flip( bool flip ){
            _robot2dh_flip = flip;
        }

        /*
            TODO
        */
        void RobotLink::setDHJoint_limits( double lower, double higher) {
            checkLowerHigher(lower, higher);
            _DHJoint_limit_lower = lower;
            _DHJoint_limit_higher = higher;
        }

        /*
            TODO
        */
        void RobotLink::setDHJoint_limits( Vector<2> limits ) {
            setDHJoint_limits( limits[0], limits[1]);
        }

        /*
            TODO
        */
        void RobotLink::setRobotJoint_limits( double lower, double higher) {
            checkLowerHigher(lower, higher);
            _RobotJoint_limit_lower = lower;
            _RobotJoint_limit_higher = higher;
        }

        /*
            TODO
        */
        void RobotLink::setRobotJoint_limits( Vector<2> limits ) {
            setRobotJoint_limits( limits[0], limits[1]);
        }

        /*
            TODO
        */    
        void RobotLink::setVelocity_limits( double lower, double higher) {
            checkLowerHigher(lower, higher);
            _velocity_limit_lower = lower;
            _velocity_limit_higher = higher;
        }

        /*
            TODO
        */
        void RobotLink::setVelocity_limits( Vector<2> limits ) {
            setVelocity_limits( limits[0], limits[1]);
        }

        /*
            TODO
        */
        void RobotLink::setName( string name ){
            _name = name;
        }

        //======END SETTERS===========//

        /*
            TODO
        */
        void RobotLink::display() const{
            /*TODO*/
            cout << _name << "Display TO DO" << endl;
        }

        /*
            return True if the input q_DH (in DH convention) exceeds the softDHLimits
        */
        bool RobotLink::exceededJointDHLimit(double q_DH) const{
            return ( q_DH <= _DHJoint_limit_lower || q_DH >= _DHJoint_limit_higher );
        }

        /*
            return True if the input q_Robot (in Robot convention) exceeds the HardRobotLimits
        */
        bool RobotLink::exceededJointRobotLimit(double q_Robot) const{
            return ( q_Robot <= _RobotJoint_limit_lower || q_Robot >= _RobotJoint_limit_higher );
        }

        /*
            return True if the input q_vel exceeds the velocity limits
        */
        bool RobotLink::exceededJointVelocity(double q_vel) const{
            return ( q_vel <= _velocity_limit_lower || q_vel >= _velocity_limit_higher );
        }

        /*
            TODO
        */
        double RobotLink::joint_Robot2DH( double q_Robot ) const{
            if(_robot2dh_flip)
                return (-q_Robot + _robot2dh_offset);
            else
                return ( q_Robot + _robot2dh_offset);
        }

        /*
            TODO
        */
        double RobotLink::joint_DH2Robot( double q_DH ) const{
            if(_robot2dh_flip)
                return -( q_DH - _robot2dh_offset);
            else
                return ( q_DH - _robot2dh_offset);
        }

        /*
            TODO
        */
        double RobotLink::jointvel_Robot2DH( double q_vel_Robot ) const{
            if(_robot2dh_flip)
                return -q_vel_Robot;
            else
                return q_vel_Robot;
        }

        /*
            TODO
        */
        double RobotLink::jointvel_DH2Robot( double q_vel_DH ) const{
            if(_robot2dh_flip)
                return -q_vel_DH;
            else
                return q_vel_DH;
        }
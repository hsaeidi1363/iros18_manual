#include<ros/ros.h>
#include<omni_msgs/OmniFeedback.h>
#include<omni_msgs/OmniButtonEvent.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/PoseStamped.h>
#include<sensor_msgs/JointState.h>
#include<trajectory_msgs/JointTrajectory.h> 
#include<trajectory_msgs/JointTrajectoryPoint.h> 
#include<kdl/chain.hpp>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>

#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>





double x_d = 0.0;
double y_d = 0.0;
double z_d = 0.0;

double kp_x = 50.0;
double kd_x = 200.0;
double kp_y = 50.0;
double kd_y = 200.0;
double kp_z = 150.0;
double kd_z = 600.0;


//Frame KDL::Frame::DH_Craig1989 (double a, double alpha, double d, double theta)

KDL::Chain LWR(){

  KDL::Chain chain;

  //base
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),
        KDL::Frame::DH_Craig1989(0,0,0.33989,0)));

  //joint 1
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, -M_PI_2,0,0)));

  //joint 2 
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0,M_PI_2,0.40011,0)));

  //joint 3
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0,M_PI_2,0,0)));

  //joint 4
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, -M_PI_2,0.40003,0)));

  //joint 5
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, -M_PI_2,0,0)));

  //joint 6
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, M_PI_2,0,0)));

  //joint 7 (with flange adapter)
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        //KDL::Frame::DH_Craig1989(0,0,0.098,0)));
//    KDL::Frame::DH_Craig1989(0,0,0.088,0))); //AS
KDL::Frame::DH_Craig1989(0,0,0.12597,0)));

  return chain;

}



//reading the kuka lwr joint positions
sensor_msgs::JointState joints;
bool initialized = false;
//callback for reading joint values
void get_joints(const sensor_msgs::JointState & data){
	for (int i = 0; i < data.position.size();++i){
		// if this is not the first time the callback function is read, obtain the joint positions
		if(initialized){
			joints.position[i] = data.position[i];	
		// otherwise initilize them with 0.0 values
		}
	}	
	initialized = true;
}



// initialize the joint positions with a non-zero value to be used in the solvers
void initialize_joints(KDL::JntArray & _jointpositions, int _nj, float _init){
	for (int i = 0; i < _nj; ++i)
		_jointpositions(i) = _init;
}

void initialize_joints(sensor_msgs::JointState & _jointpositions, int _nj, float _init){
	for (int i = 0; i < _nj; ++i)
		_jointpositions.position.push_back(_init);
}


// initialize a joint command point
void initialize_points(trajectory_msgs::JointTrajectoryPoint & _pt, int _nj, float _init){
	for (int i = 0; i < _nj; ++i)
		_pt.positions.push_back(_init);
}

//defines the joint names for the robot (used in the jointTrajectory messages)
void name_joints(trajectory_msgs::JointTrajectory & _cmd, int _nj){
	for (int i = 1; i <= _nj; ++i){
		std::ostringstream joint_name;		
		joint_name << "iiwa_joint_";
		joint_name << i;
		_cmd.joint_names.push_back(joint_name.str());
	}
}

void eval_points(trajectory_msgs::JointTrajectoryPoint & _point, KDL::JntArray & _jointpositions, int _nj){
	// joints can move between -+: 172,120,172,120,172,120,170
	//double joint_bounds[] = {3.002, 2.0944,3.002, 2.0944,3.002, 2.0944, 3.002};
	for (int i = 0; i < _nj; ++i){
		 while(_jointpositions(i) > M_PI)
				_jointpositions(i) -= 2*M_PI;
		 while(_jointpositions(i) < -M_PI)
				_jointpositions(i) += 2*M_PI;
		_point.positions[i] = _jointpositions(i);
	}	
}






trajectory_msgs::JointTrajectory auto_cmd;

geometry_msgs::PoseStamped pos;
omni_msgs::OmniFeedback centering_force;
omni_msgs::OmniFeedback centering_force_prev;

omni_msgs::OmniButtonEvent button;
omni_msgs::OmniButtonEvent prev_button;
geometry_msgs::Twist dbg;

std_msgs::Bool test_start;

bool autonomous_mode = false; // manual is 1 and 0 is auto

void get_auto(const trajectory_msgs::JointTrajectory & _data){
	auto_cmd = _data;
}

void get_pos(const geometry_msgs::PoseStamped & _data){
	pos = _data;

}

bool ref_change = false;

void get_button(const omni_msgs::OmniButtonEvent & _data){
	button = _data;
	if(button.grey_button == 1 && prev_button.grey_button == 0){
		//x_d =  pos.pose.position.x; 
		//y_d =  pos.pose.position.y; 
		//z_d =  pos.pose.position.z; 
		ref_change = true;
		autonomous_mode = !autonomous_mode;
	}		
	if(button.white_button == 1 && prev_button.white_button == 0){
		test_start.data = !test_start.data;
	}		
	prev_button = button;
}

double e_x_prev = 0.0, e_y_prev = 0.0, e_z_prev = 0.0;


void calc_center_force(void){
	double e_x, e_y, e_z, de_x, de_y, de_z;
	e_x = x_d - pos.pose.position.x; 
	e_y = y_d - pos.pose.position.y; 
	e_z = z_d - pos.pose.position.z; 
	de_x = e_x - e_x_prev;
	de_y = e_y - e_y_prev;
	de_z = e_z - e_z_prev;
	double tau = 0.8;
	if (autonomous_mode){
		centering_force.force.x = (kp_x*e_x + kd_x*de_x)*(1-tau) + tau*centering_force_prev.force.x;
		centering_force.force.y = (kp_y*e_y + kd_y*de_y)*(1-tau) + tau*centering_force_prev.force.y;
        }else{
		
		centering_force.force.x = 0.0;
		centering_force.force.y = 0.0;
	}
	centering_force.force.z = (kp_z*e_z + kd_z*de_z)*(1-tau) + tau*centering_force_prev.force.z;
	if(ref_change){
		ref_change = false;
		centering_force.force.x += centering_force_prev.force.x;
		centering_force.force.y += centering_force_prev.force.y;
		centering_force.force.z += centering_force_prev.force.z;
	}
	dbg.linear.x = e_x;
	dbg.linear.y = de_x;
	dbg.linear.z = centering_force.force.x;
	e_x_prev = e_x;
	e_y_prev = e_y;
	e_z_prev = e_z;
	centering_force_prev = centering_force;
}


int main(int argc, char * argv[]){
    
    
    
    
    // define the kinematic chain
	KDL::Chain chain = LWR();
	// define the forward kinematic solver via the defined chain
	KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
	// define the inverse kinematics solver
	KDL::ChainIkSolverVel_pinv iksolverv = KDL::ChainIkSolverVel_pinv(chain);//Inverse velocity solver
	KDL::ChainIkSolverPos_NR iksolver(chain,fksolver,iksolverv,100,1e-4);//Maximum 100 iterations, stop at accuracy 1e-6

	// get the number of joints from the chain
	unsigned int nj = chain.getNrOfJoints();
	// define a joint array in KDL format for the joint positions
        KDL::JntArray jointpositions = KDL::JntArray(nj);
	// define a joint array in KDL format for the next joint positions
	KDL::JntArray jointpositions_new = KDL::JntArray(nj);
	// define a manual joint command array for debugging	
	KDL::JntArray manual_joint_cmd = KDL::JntArray(nj);
        
        
        
    
	ros::init(argc, argv, "tele_op");
	ros::NodeHandle nh_;
	ros::NodeHandle home("~");

	bool semi_auto = false;
	home.getParam("semi_auto",semi_auto) ;
	
	int loop_freq = 100;
        float dt = (float) 1/loop_freq;
	ros::Rate loop_rate(loop_freq);	

	ros::Publisher force_pub =nh_.advertise<omni_msgs::OmniFeedback>("/phantom/force_feedback",10);
        
        // defining the puilsher that accepts joint position commands and applies them to the simulator
	std::string command_topic = "iiwa/PositionJointInterface_trajectory_controller/command";


	ros::Publisher cmd_pub = nh_.advertise<trajectory_msgs::JointTrajectory>(command_topic,10);


	ros::Publisher control_mode_pub = nh_.advertise<std_msgs::UInt8>("iiwa/control_mode",10);

	ros::Publisher test_start_pub = nh_.advertise<std_msgs::Bool>("/test_start",10);
        
       
	ros::Publisher dbg_pub = nh_.advertise<geometry_msgs::Twist>("/hapticdbg",10);

	

	// subscriber for reading the joint angles from the gazebo simulator
        ros::Subscriber joints_sub = nh_.subscribe("/iiwa/joint_states",10, get_joints);

        ros::Subscriber auto_sub = nh_.subscribe("/iiwa/auto/command",10, get_auto);
        

	ros::Subscriber pos_sub = nh_.subscribe("/phantom/pose",10, get_pos);
	
	ros::Subscriber button_sub = nh_.subscribe("/phantom/button",10, get_button);

        trajectory_msgs::JointTrajectory joint_cmd;
	trajectory_msgs::JointTrajectoryPoint pt,pt2;

	initialize_points(pt,nj,0.0);
	initialize_points(pt2,nj,0.0);
	
	double roll, pitch, yaw, x, y, z;
        
        // define the joint names, e.g. iiwa_joint_1 up to iiwa_joint_7
	name_joints(joint_cmd, nj);
	

	initialize_joints(jointpositions, nj, 0.2);
        initialize_joints(joints, nj, 0.0);
        
	KDL::Frame cartpos;    
        KDL::Rotation rpy = KDL::Rotation::RPY(roll,pitch,yaw); //Rotation built from Roll-Pitch-Yaw angles

	KDL::Frame current_cartpos;    
	
	eval_points(pt, manual_joint_cmd, nj); 
        pt.time_from_start = ros::Duration(1.0);
        joint_cmd.points.push_back(pt);
        geometry_msgs::Twist ref;
        geometry_msgs::Twist xyz;
        // for debugging: Calculate forward position kinematics
        bool kinematics_status;
        bool start_loc_available = false;
        bool all_zero = true;

	std_msgs::UInt8 control_mode;

        test_start.data = false;
        while (ros::ok()){
		if (initialized){
			// update the joint positions with the most recent readings from the joints
			for (int k = 0; k<7; ++k){
				jointpositions(k) = joints.position[k];
                                all_zero &= (fabs(jointpositions(k)) <.01);
                                    
			}				
                        if(!all_zero){
                        //find where the robot is
                            if(!start_loc_available){
                                kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
                                if(kinematics_status>=0){
                                        xyz.linear.x = cartpos.p[0];
                                        xyz.linear.y = cartpos.p[1];
                                        xyz.linear.z = cartpos.p[2];
                                        cartpos.M.GetRPY(roll,pitch, yaw);
                                        xyz.angular.x = roll;
                                        xyz.angular.y = pitch;
                                        xyz.angular.z = yaw;
                                }
                                start_loc_available = true;
                                
                            }else{
                            
  
                                ref.linear.x = xyz.linear.x - pos.pose.position.y;
                                ref.linear.y = xyz.linear.y + pos.pose.position.x;
                                ref.linear.z = xyz.linear.z + pos.pose.position.z*0.2;
                                //keep the same orientation
                                ref.angular.x = xyz.angular.x;
                                ref.angular.y = xyz.angular.y;
                                ref.angular.z = xyz.angular.z;
                                // update the reference cartesian positions
                                cartpos.p[0]=ref.linear.x;
                                cartpos.p[1]=ref.linear.y;
                                cartpos.p[2]=ref.linear.z;			
                                rpy = KDL::Rotation::RPY(ref.angular.x,ref.angular.y,ref.angular.z);
                                cartpos.M = rpy;
                                int ret = iksolver.CartToJnt(jointpositions,cartpos,jointpositions_new);
                                eval_points(pt, jointpositions_new, nj);
                                pt.time_from_start = ros::Duration(dt);
                                joint_cmd.points[0] = pt;
                                joint_cmd.header.stamp = ros::Time::now();

				if (autonomous_mode){
					joint_cmd = auto_cmd;                                
                                	kinematics_status = fksolver.JntToCart(jointpositions,current_cartpos);
					y_d = -(current_cartpos.p[0] -xyz.linear.x);
					x_d = current_cartpos.p[1]-xyz.linear.y;
					z_d = current_cartpos.p[2]-xyz.linear.z;
				}
                                cmd_pub.publish(joint_cmd);
                            }
                        }
                                        
                }
		if(autonomous_mode)
			control_mode.data = 0;
		else
			control_mode.data = 1;
                dbg_pub.publish(dbg);
		control_mode_pub.publish(control_mode);

		test_start_pub.publish(test_start);

                calc_center_force();
		force_pub.publish(centering_force);
		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}

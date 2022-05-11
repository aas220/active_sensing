 //
// Created by tipakorng on 3/3/16.
//
//Modified by Lexi Scott in 2022

#include <visualization_msgs/MarkerArray.h>
#include "simulator.h"
#include "active_sensing_continuous/action_message.h"
#include "rng.h"

unsigned int sensing_action_global;
double observation_global;
int obversation_back_flag = 0;
int continue_flag = 0;
double update_location_x;
double update_location_y;
double update_location_z;
int communication_count = 0;
//int globalcheckflag = 0;

Simulator::Simulator(Model &model, BeliefSpacePlanner &planner, unsigned int sensing_interval) :
        model_(model),
        planner_(planner),
        sensing_interval_(sensing_interval)
{
    has_publisher_ = false;
}

Simulator::Simulator(Model &model, BeliefSpacePlanner &planner, ros::NodeHandle *node_handle,
                     unsigned int sensing_interval) :
        model_(model),
        planner_(planner),
        sensing_interval_(sensing_interval),
        node_handle_(node_handle)
{
    publisher_ = node_handle_->advertise<visualization_msgs::Marker>("simulator", 1);
    has_publisher_ = true;
    //ros::NodeHandle n_h;
    //ros::Publisher sensing_action_pub = n_h.advertise<active_sensing_continuous::ReqObsrv>("req_obsrv", 1000);
    
}

Simulator::~Simulator()
{}

void Simulator::initSimulator()
{
    // Clear stuff
    states_.clear();
    task_actions_.clear();
    sensing_actions_.clear();
    observations_.clear();
    cumulative_reward_ = 0;
    // Reset planner
    planner_.reset();
    // Reset active sensing time
    active_sensing_time_ = 0;
}

void Simulator::updateSimulator(unsigned int sensing_action, Eigen::VectorXd observation, Eigen::VectorXd task_action)
{	globalcheckflag = 1;
	nextflag=1;
    ROS_INFO("update simulator");
    Eigen::VectorXd new_state = model_.sampleNextState(states_.back(), task_action);
   ROS_INFO("new state0 is %f",new_state(0));
    ROS_INFO("new state1 is %f",new_state(1));
	nextflag=0;
    states_.push_back(new_state);
    sensing_actions_.push_back(sensing_action);
    observations_.push_back(observation);
    task_actions_.push_back(task_action);
    cumulative_reward_ += model_.getReward(new_state, task_action);
	globalcheckflag = 0;
}

void Simulator::updateSimulator(Eigen::VectorXd task_action)
{
	globalcheckflag = 1;
	nextflag=1;
    ROS_INFO("updateelse simulator");
    Eigen::VectorXd new_state = model_.sampleNextState(states_.back(), task_action);
   ROS_INFO("new state0 is %f",new_state(0));
    ROS_INFO("new state1 is %f",new_state(1));
	nextflag=0;
    states_.push_back(new_state);
    task_actions_.push_back(task_action);
    cumulative_reward_ += model_.getReward(new_state, task_action);
	globalcheckflag = 0;
}


/*
void observationBackCallback(const active_sensing_continuous::ObsrvBack& msg)
{
  //ROS_INFO("observation: [%f]", msg.observation_back);
  observation_global = msg.observation_back;
  obversation_back_flag = 1;
  //ROS_INFO("flag is %d", obversation_back_flag);
}

void continueCallback(const active_sensing_continuous::CT& msg)
{
	continue_flag= 1;
}



/*
void Simulator::pulishReqobsrv(ROS::NodeHandle nodehandle)
{
  ros::Publisher sensing_action_pub = nodehandle.advertise<active_sensing_continuous::ReqObsrv>("req_obsrv", 1000);

}

*/




void Simulator::simulate(const Eigen::VectorXd &init_state, unsigned int num_steps, unsigned int verbosity)
{
    initSimulator();
    unsigned int n = 0;
    unsigned int sensing_action;
    double active_sensing_time = 0;

    Eigen::VectorXd observation(1);
    Eigen::VectorXd task_action(3);
    states_.push_back(init_state);
    std::chrono::high_resolution_clock::time_point active_sensing_start;
    std::chrono::high_resolution_clock::time_point active_sensing_finish;
    std::chrono::duration<double> active_sensing_elapsed_time;


    if (verbosity > 0)
        std::cout << "state = \n" << states_.back().transpose() << std::endl;

    planner_.publishParticles();
    
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<active_sensing_continuous::action_message>("new_active_sensing");
    active_sensing_continuous::action_message srv;

    while (!model_.isTerminal(states_.back()) && n < num_steps)
    {
	
        // Normalize particle weights.
        planner_.normalizeBelief();
        ROS_INFO("communication round is %d",communication_count);
	    ROS_INFO("n is %d",n);
        //This is a check to make sure it's still communicating with roscore'
        if(!ros::master::check()){
            ROS_ERROR("Failed to access roscore");
            ros::Duration(2).sleep();

        }
        // If sensing is allowed in this step.
        if (n % (sensing_interval_ + 1) == 0)
        {

            // Get sensing action.
            active_sensing_start = std::chrono::high_resolution_clock::now();
            sensing_action = planner_.getSensingAction();
            active_sensing_finish = std::chrono::high_resolution_clock::now();
            active_sensing_elapsed_time = std::chrono::duration_cast<std::chrono::duration<double> >
                    (active_sensing_finish-active_sensing_start);
            active_sensing_time += active_sensing_elapsed_time.count();
            ROS_INFO("GENERATE SENSING ACTION");
            srv.request.x = sensing_action;
            srv.request.y = 0;
            srv.request.z = 0;
            srv.request.type = 1;

            /*
                code below pub observation request to local
            */

            //sensing_action_global = sensing_action;
	        observation = model_.sampleObservation(states_.back(), sensing_action);
            while(ros::ok)
            {
            int Break_Loop = 0;
            if(client.call(srv))
            {
                switch(srv.response.type1)
                {
                    case 2:
                     ROS_INFO("RECEIVE OBSERVATION.");
                     observation(0) = srv.response.x1;
                     ROS_INFO("GETTING TASK ACTION");
                     planner_.updateBelief(sensing_action, observation);
                     task_action = planner_.getTaskAction();
                     srv.request.x = task_action(0);
                     srv.request.y = task_action(1);
                     srv.request.z = task_action(2);
                     srv.request.type = 3;
                     break;
                    case 4:
                     ROS_INFO("CONTINUE.");
                     planner_.predictBelief(task_action);
                     ROS_INFO("UPDATE BELIEF");
                     updateSimulator(sensing_action, observation, task_action);
                     Break_Loop=1;
                     break;
                }
                 
            }
            else
            {
              ROS_ERROR("Failed to call service");
            }
            if(Break_Loop==1)
            {
                break;
            }
            }
            if (verbosity > 0)
            {
                std::cout << "n = " << n << std::endl;
                std::cout << "sensing_action = " << sensing_action << std::endl;
                std::cout << "observation = " << observation.transpose() << std::endl;
                std::cout << "most_likely_state = " << planner_.getMaximumLikelihoodState().transpose() << std::endl;
                std::cout << "task_action = " << task_action.transpose() << std::endl;
                std::cout << "state = " << states_.back().transpose() << std::endl;
             }
        }

            // Otherwise, set sensing action to DO NOTHING.
        else
        {
            task_action = planner_.getTaskAction();           
	        ROS_INFO("task_action(0) is %f",task_action(0));
            ROS_INFO("task_action(1) is %f",task_action(1));
            //ROS_INFO("task_action(2) is %f",task_action(2));
	        //ROS_INFO("statesback0 before predict is %f",states_.back()(0));
	        //ROS_INFO("statesback1 before predice is %f",states_.back()(1));
            planner_.predictBelief(task_action);
	        //ROS_INFO("statesback0 before update is %f",states_.back()(0));
	        //ROS_INFO("statesback1 before update is %f",states_.back()(1));
            updateSimulator(task_action);
	        //ROS_INFO("statesback0 after update is %f",states_.back()(0));
	        //ROS_INFO("statesback1 adter update is %f",states_.back()(1));  
            if (verbosity > 0)
            {
                std::cout << "n = " << n << std::endl;
                std::cout << "sensing_action = " << "n/a" << std::endl;
                std::cout << "observation = " << "n/a" << std::endl;
                std::cout << "most_likely_state = " << planner_.getMaximumLikelihoodState().transpose() << std::endl;
                std::cout << "task_action = " << task_action.transpose() << std::endl;
                std::cout << "state = " << states_.back().transpose() << std::endl;
            }
        }

        planner_.publishParticles();
        model_.publishMap();
        if (has_publisher_)
        {
            publishState();
        }
        n++;
	    communication_count++;
	    ROS_INFO("!model_.isTerminal(states_.back()) after loop is %d",!model_.isTerminal(states_.back()));
        ROS_INFO("\n");
        //ros::Duration(1.0).sleep();

    }

    int num_sensing_steps = (n + 1) / (sensing_interval_ + 1);

    if (num_sensing_steps > 0)
        active_sensing_time_ = active_sensing_time / num_sensing_steps;
    else
        active_sensing_time_ = 0;
}

std::vector<Eigen::VectorXd> Simulator::getStates()
{
    return states_;
}

std::vector<unsigned int> Simulator::getSensingActions()
{
    return sensing_actions_;
}

std::vector<Eigen::VectorXd> Simulator::getTaskActions()
{
    return task_actions_;
}

std::vector<Eigen::VectorXd> Simulator::getObservations()
{
    return observations_;
}

double Simulator::getCumulativeReward()
{
    return cumulative_reward_;
}

double Simulator::getAverageActiveSensingTime()
{
    return active_sensing_time_;
}

void Simulator::publishState()
{
    if (has_publisher_)
    {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0, 0, 0));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "my_frame"));

        visualization_msgs::Marker marker;

        uint32_t shape = visualization_msgs::Marker::CUBE;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;

        Eigen::VectorXd state = states_.back();

        model_.fillMarker(state, marker);

        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        publisher_.publish(marker);
    }
}

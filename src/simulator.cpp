 // //
// Created by tipakorng on 3/3/16.
//
//Modified by Lexi Scott in 2022

#include <visualization_msgs/MarkerArray.h>
#include "simulator.h"
#include "active_sensing_continuous/action_message.h"
#include "rng.h"
#include<iostream>
#include<fstream>

Eigen::VectorXd observation(1);
Eigen::VectorXd task_action(3);


int obversation_back_flag = 0;
int continue_flag = 0;
double update_location_x;
double update_location_y;
double update_location_z;
using namespace std;
int communication_count = 0;
double task_action_time = 0;
std::vector<double> all_rewards;
unsigned int round_ = 0;
fstream file;
fstream file2;
Eigen::VectorXd task_action_backup(3);
unsigned int n = 0;
unsigned int sensing_action;
double active_sensing_time = 0;
std::chrono::high_resolution_clock::time_point active_sensing_start;
std::chrono::high_resolution_clock::time_point active_sensing_finish;
std::chrono::duration<double> active_sensing_elapsed_time;
int Break_Loop = 0;
int ncount = 0;

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
    task_action_time = 0;
    states_.clear();
    task_actions_.clear();
    sensing_actions_.clear();
    observations_.clear();
    cumulative_reward_ = 0;
    // Reset planner
    planner_.reset();
    // Reset active sensing time
    active_sensing_time_ = 0;
    n = 0;
    active_sensing_time = 0;
}

void Simulator::updateSimulator(unsigned int sensing_action, Eigen::VectorXd observation, Eigen::VectorXd task_action)
{	globalcheckflag = 1;
	nextflag=1;
    ROS_INFO("update simulator");
    Eigen::VectorXd new_state = model_.sampleNextState(states_.back(), task_action);
   ROS_INFO("server new state0 is %f at round %d",new_state(0), communication_count);
    ROS_INFO("server new state1 is %f at round %d ",new_state(1), communication_count);
    ROS_INFO("server new state1 is %f at round %d ", new_state(2), communication_count);
	nextflag=0;
    states_.push_back(new_state);
    sensing_actions_.push_back(sensing_action);
    observations_.push_back(observation);
    task_actions_.push_back(task_action);
    double currentReward = model_.getReward(new_state, task_action);
    cumulative_reward_ += currentReward;
    all_rewards.push_back(currentReward);
	globalcheckflag = 0;
 
}

void Simulator::updateSimulator(Eigen::VectorXd task_action)
{
	globalcheckflag = 1;
	nextflag=1;
    ROS_INFO("updateelse simulator");
    Eigen::VectorXd new_state = model_.sampleNextState(states_.back(), task_action);
    ROS_INFO("server new state0 is %f at round %d", new_state(0), communication_count);
    ROS_INFO("server new state1 is %f at round %d ", new_state(1), communication_count);
    ROS_INFO("server new state1 is %f at round %d ", new_state(2), communication_count);
	nextflag=0;
    states_.push_back(new_state);
    task_actions_.push_back(task_action);
    cumulative_reward_ += model_.getReward(new_state, task_action);
	globalcheckflag = 0;
}


bool Simulator::localmachine(active_sensing_continuous::action_message2::Request& req,
    active_sensing_continuous::action_message2::Response& res) {
    //Make it party in here
   
    ROS_INFO("Server thinks it got, %d", req.type1);
    
    if (req.recovery == 1) {
        ROS_ERROR("UPDATING RECOVERY INFO");
        //res.type = 7;
        //return true;
        if (req.serverCCReply > 0) {
            communication_count = req.serverCCReply;
            n = req.n;

            Eigen::VectorXd old_state(3);
            old_state[0] = req.state1;
            old_state[1] = req.state2;
            old_state[2] = req.state3;
           // states_.push_back(old_state);
         
            
            observation(0) = req.observation;
            sensing_action = req.sense;
            sensing_actions_.push_back(sensing_action);
            observations_.push_back(observation);


            task_action_backup[0] = req.t1;
            task_action_backup[1] = req.y1;
            task_action_backup[2] = req.z1;
            task_actions_.push_back(task_action_backup);
            task_action = task_action_backup;
         

            std::cout << "state = " << states_.back().transpose() << std::endl;

            //updateSimulator(sensing_action, observation, task_action);
            //updateSimulator(task_action);
            ROS_ERROR("NEW COMMUNICATION COUNT IS %d", communication_count);
            ROS_ERROR("NEW N COUNT IS %d", n);
            ROS_INFO("server new state0 from recovery is is %f at round %d", states_.back()(0), communication_count);
            ROS_INFO("server new state1 is %f at round %d ", states_.back()(1), communication_count);
            ROS_INFO("server new state2 is %f at round %d ", states_.back()(2), communication_count);
            planner_.publishParticles();
            planner_.normalizeBelief();
            for (int i = 0; i < communication_count; i++) {

                if (i % (sensing_interval_ + 1) == 0) {
                    sensing_action = planner_.getSensingAction();
                    observation = model_.sampleObservation(states_.back(), sensing_action);
                    planner_.updateBelief(sensing_action, observation);
                    task_action = planner_.getTaskAction();
                    planner_.predictBelief(task_action);
                    updateSimulator(sensing_action, observation, task_action);

                }
                else{
                    planner_.getTaskAction();
                    planner_.predictBelief(task_action);
                
                    updateSimulator(task_action);
                    ROS_INFO("Rigging the noise");
                }
            }
          
            /* 
            if (communication_count % (sensing_interval_ + 1) == 0) {
                planner_.getSensingAction();
                model_.sampleObservation(states_.back(), sensing_action);
                planner_.updateBelief(sensing_action, observation);

            }
            planner_.publishParticles();
            planner_.normalizeBelief();
            */
            ROS_INFO("server new after nosie state0 from recovery is is %f at round %d", states_.back()(0), communication_count);
            ROS_INFO("server new state1 is %f at round %d ", states_.back()(1), communication_count);
            ROS_INFO("server new state2 is %f at round %d ", states_.back()(2), communication_count);
            
            //res.type = 5;
            //return true;
           
        }
        if (req.type1 == 2) {
            ROS_INFO("GENERATE SENSING ACTION");
            active_sensing_start = std::chrono::high_resolution_clock::now();
            sensing_action = planner_.getSensingAction();
            active_sensing_finish = std::chrono::high_resolution_clock::now();
            active_sensing_elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>
                (active_sensing_finish - active_sensing_start);
            active_sensing_time += active_sensing_elapsed_time.count();
            observation = model_.sampleObservation(states_.back(), sensing_action);

            res.type = 1;
            res.x = sensing_action;
            res.y = 0;
            res.z = 0;
            res.serverCC = communication_count;
            res.servern = n;
        }
    }
    
        switch (req.type1)
        {
        case 1:
            
            // Get sensing action.
         
            ROS_INFO("GENERATE SENSING ACTION");
                active_sensing_start = std::chrono::high_resolution_clock::now();
                sensing_action = planner_.getSensingAction();
                active_sensing_finish = std::chrono::high_resolution_clock::now();
                active_sensing_elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>
                    (active_sensing_finish - active_sensing_start);
                active_sensing_time += active_sensing_elapsed_time.count();
                observation = model_.sampleObservation(states_.back(), sensing_action);
            
            res.type = 1;
            res.x = sensing_action;
            res.y = 0;
            res.z = 0;
            res.serverCC = communication_count;
            res.servern = n ;
            ROS_INFO("finished generating sensing action");
          
            
             //observation = model_.sampleObservation(states_.back(), sensing_action);
            return true;
            break;
        case 2:
            ROS_INFO("RECEIVE OBSERVATION.");
            observation(0) = req.x1;
            ROS_INFO("GETTING TASK ACTION");
            planner_.updateBelief(sensing_action, observation);
            task_action = planner_.getTaskAction();
            res.type = 3;
            res.x = task_action(0);
            res.y = task_action(1);
            res.z = task_action(2);
            res.serverCC = communication_count;
            res.servern = n;
            return true;
            break;
        case 4:
            
            //ROS_ERROR("CONTINUE.");

            planner_.predictBelief(task_action);
            
            ROS_INFO("UPDATE BELIEF");
            updateSimulator(sensing_action, observation, task_action);
         
    //Just do something to indicate that the continue was executed
         
            Break_Loop = 1;
            res.type = 0;
            res.serverCC = communication_count;
            res.servern = n;
            //                task_action_time = srv.response.task_action_time;
            return true;
            break;
        }

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




void Simulator::simulate(const Eigen::VectorXd& init_state, unsigned int num_steps, unsigned int verbosity, unsigned int round)
{
    initSimulator();

    states_.push_back(init_state);
  
    if (verbosity > 0)
        std::cout << "state = \n" << states_.back().transpose() << std::endl;

    planner_.publishParticles();

    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("cloud_active_sensing_recovery", &Simulator::localmachine, this);
   // ros::ServiceClient client = nh.serviceClient<active_sensing_continuous::action_message>("new_active_sensing");
    active_sensing_continuous::action_message2 srv2;
    cout << "advertised service";
    ros::spinOnce();
    while (!model_.isTerminal(states_.back()) && n < num_steps)
    {
         // Normalize particle weights.
        planner_.normalizeBelief();
        ROS_INFO("server communication round is %d", communication_count);
        ROS_INFO("server n is %d", n);
       // ros::Duration(2).sleep();
        //is is a check to make sure it's still communicating with roscore'
        if (!ros::master::check()) {
            ROS_ERROR("Failed to access roscore");
            //ros::Duration(5).sleep();
            system("cd ..");
            system("sudo ./runlocal.sh");

        }
        // If sensing is allowed in this step.
       
        if (n % (sensing_interval_ + 1) == 0)
        {
            //ros::spinOnce();
           // observation = model_.sampleObservation(states_.back(), sensing_action);
    
            while (ros::ok)
            {
                Break_Loop = 0;
                ros::spinOnce();

                if (Break_Loop == 1)
                {
                    break;
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
        }

            // Otherwise, set sensing action to DO NOTHING.
            else
            {
                task_action = planner_.getTaskAction();
                ROS_INFO("server task_action(0) in else is %f at round %d", task_action(0), communication_count);
                ROS_INFO("server task_action(1)in else  is %f at round %d", task_action(1), communication_count);
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
            ROS_INFO("!model_.isTerminal(states_.back()) after loop is %d", !model_.isTerminal(states_.back()));
            ROS_INFO("\n");
            //ros::Duration(1.0).sleep();

        }

        int num_sensing_steps = (n + 1) / (sensing_interval_ + 1);

        if (num_sensing_steps > 0) {
            active_sensing_time_ = active_sensing_time / num_sensing_steps;
            task_action_time = task_action_time / num_sensing_steps;
        }
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
    std::time_t raw_time;
    struct tm* time_info;
    char buffer[90];
    time(&raw_time);
    time_info = localtime(&raw_time);
    strftime(buffer, 80, "%Y-%m-%d-%H:%M:%S", time_info);
    std::string file_name(buffer);
    std::string output_path = round_ +"averaged_rewards_cloud_sever" + file_name + ".txt";
    std::string output_path2 = round_ + "task_action_average" + file_name + ".txt";
    file.open(output_path, ios_base::out);

    for (int i = 0; i < all_rewards.size(); i++)
    {
        file << all_rewards[i] << endl;
    }
    file.close();
    file2.open(output_path2, ios_base::out);
    file2 << "Task Action Average";
    file2 << task_action_time;
    file2.close();
   
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

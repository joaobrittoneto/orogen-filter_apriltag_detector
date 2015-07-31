/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace filter_apriltag_detector;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    else
    {
        thresh = _threshold.get();
        rbs_vector.clear();

    	return true;
    }
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    base::samples::RigidBodyState sample;
    while (_pose_sample.read(sample) == RTT::NewData)
    {
        if (rbs_vector.size() < 3)
        {
            rbs_vector.push_back(sample);
            _output.write(sample);
        }
        else
        {
            rbs_vector.push_back(sample);
            std::vector<double> sec_diff(6);
            base::samples::RigidBodyState rbs_out;
            outlierFilter(rbs_vector, sec_diff, rbs_out);
            _output.write(rbs_out);
            _out_sec_diff.write(sec_diff);
            rbs_vector.erase(rbs_vector.begin());
        }
    }

}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

void Task::outlierFilter(std::vector<base::samples::RigidBodyState> &rbs_vector, std::vector<double> &sec_diff, base::samples::RigidBodyState &rbs_out)
{
    double diff1, diff2;
	double step1 = (rbs_vector[1].time - rbs_vector[0].time).toSeconds();
	double step2 = (rbs_vector[2].time - rbs_vector[1].time).toSeconds();

    for( int i = 0; i<3; ++i)
    {
    diff1 = (rbs_vector[1].position[i]-rbs_vector[0].position[i])/step1;
    diff2 = (rbs_vector[2].position[i]-rbs_vector[1].position[i])/step2;

    sec_diff[i] = (diff2-diff1)/step2;
    }
    
    //second derivative of Roll
    diff1 = ((base::getRoll(rbs_vector[1].orientation)) - (base::getRoll(rbs_vector[0].orientation)))/step1;
    diff2 = ((base::getRoll(rbs_vector[2].orientation)) - (base::getRoll(rbs_vector[1].orientation)))/step2;
    sec_diff[3] = (diff2-diff1)/step2;

    //second derivative of Pitch
    diff1 = ((base::getPitch(rbs_vector[1].orientation)) - (base::getPitch(rbs_vector[0].orientation)))/step1;
    diff2 = ((base::getPitch(rbs_vector[2].orientation)) - (base::getPitch(rbs_vector[1].orientation)))/step2;
    sec_diff[4] = (diff2-diff1)/step2;

    //second derivative of Yaw
    //diff1 = fabs(((base::getYaw(rbs_vector[1].orientation)) - (base::getYaw(rbs_vector[0].orientation))));
    diff1 = ((base::getYaw(rbs_vector[1].orientation)) - (base::getYaw(rbs_vector[0].orientation)))/step1;
    diff2 = ((base::getYaw(rbs_vector[2].orientation)) - (base::getYaw(rbs_vector[1].orientation)))/step2;
    sec_diff[5] = (diff2-diff1)/step2;

    if (sec_diff[5] >= thresh[5])
    {
        rbs_out = rbs_vector[2]; 
        rbs_out.orientation = rbs_vector[1].orientation; 
    }
    else
        rbs_out = rbs_vector[2];
}


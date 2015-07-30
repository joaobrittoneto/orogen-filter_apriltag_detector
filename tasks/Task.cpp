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
    	gThreshold = _threshold.get();
    	first_time = true;

    	while(!queueSamples.empty())
    	{
    		queueSamples.pop();
    	}

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
    base::samples::RigidBodyState new_sample;
    std::vector<bool> outlier;
    outlier.resize(6);
    for(int i=0; i<6; i++)
    {
    	outlier[i] = false;
    }

    while (_pose_sample.read(sample) == RTT::NewData)
    {
    	if(first_time)
    	{
    		lastSample = sample;
    		first_time = false;
    	}

    	if(outlierDetected(sample, outlier))
		{
    		removeOutlier(sample, new_sample, outlier);
		}
    	else
    		new_sample = sample;

    	lastSample = new_sample;
		queueSamples.push(new_sample);
		_output.write(new_sample);
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

bool Task::outlierDetected(base::samples::RigidBodyState &sample, std::vector<bool> &outlier)
{
	base::samples::RigidBodyState aux_sample = sample;
	double step = (sample.time - lastSample.time).toSeconds();

	bool outlierDetected = false;
	double derivative;

	for(int i=0; i<3; i++)
	{
		if(step != 0)
			derivative = (sample.position[i] - lastSample.position[i])/step;
		else
			derivative = 1000;

		filteredSample.velocity[i] = derivative;
		double change =  fabs(derivative - lastSample.velocity[i])/fabs(lastSample.velocity[i]);

		if( change > gThreshold)
		{
			outlier[i] = true;
			outlierDetected = true;
		}
	}

	if(step != 0)
		derivative = ((base::getRoll(sample.orientation)) - (base::getRoll(lastSample.orientation)))/step;
	else
		derivative = 1000;
	filteredSample.angular_velocity[0] = derivative;
	double change =  fabs(derivative - lastSample.angular_velocity[0])/fabs(lastSample.angular_velocity[0]);
	if(change > gThreshold)
	{
		outlier[3] = true;
		outlierDetected = true;
	}

	if(step != 0)
		derivative = ((base::getPitch(sample.orientation)) - (base::getPitch(lastSample.orientation)))/step;
	else
		derivative = 1000;
	filteredSample.angular_velocity[1] = derivative;
	change =  fabs(derivative - lastSample.angular_velocity[1])/fabs(lastSample.angular_velocity[1]);
	if(change > gThreshold)
	{
		outlier[4] = true;
		outlierDetected = true;
	}

	if(step != 0)
		derivative = ((base::getYaw(sample.orientation)) - (base::getYaw(lastSample.orientation)))/step;
	else
		derivative = 1000;
	filteredSample.angular_velocity[2] = derivative;
	change =  fabs(derivative - lastSample.angular_velocity[2])/fabs(lastSample.angular_velocity[2]);
	if(change > gThreshold)
	{
		outlier[5] = true;
		outlierDetected = true;
	}

	return outlierDetected;
}

bool Task::removeOutlier(base::samples::RigidBodyState &input, base::samples::RigidBodyState &ouput, std::vector<bool> &outlier)
{
	return true;
}

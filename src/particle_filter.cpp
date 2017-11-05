/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) 
{
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	num_particles = 100;

	std::default_random_engine dre;
	std::mt19937 eng(dre());
	std::normal_distribution<double> n_distrib_x(0, std_pos[0]);
	std::normal_distribution<double> n_distrib_y(0, std_pos[1]);
	std::normal_distribution<double> n_distrib_theta(0, std_pos[2]);

	for (i=0; i<num_particles; i++)
	{
		particles[i].x = x + n_distrib_x(gen);
		particles[i].y = y + n_distrib_y(gen);
		particles[i].theta = theta + n_distrib_theta(gen);
	}





	/*
	std::random_device rd; // obtain a random number from hardware
    std::mt19937 eng(rd()); // seed the generator
    std::uniform_int_distribution<> distr(25, 63); // define the range

    for(int n=0; n<40; ++n)
        std::cout << distr(eng) << ' '; // generate numbers
	*/




	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) 
{
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Prediction utilizing the bicycle model
	
	// Preparing the error terms:
	std::default_random_engine dre;
	std::mt19937 eng(dre());
	std::normal_distribution<double> n_distrib_x(0, std_pos[0]);
	std::normal_distribution<double> n_distrib_y(0, std_pos[1]);
	std::normal_distribution<double> n_distrib_theta(0, std_pos[2]);
	// Two ways to create new normal distributions:
	// 1.) n_distr = std::normal_distribution<double>(0, 0);
	// 2.) n_distr.param(std::normal_distribution<double>::param_type(0, 0);
	
	double err_x, err_y, err_theta;

	for (i=0; i<num_particles; i++)
	{	
		// Generate new error terms
		err_x = n_distrib_x(eng);
		err_y = n_distrib_y(eng);
		err_theta = n_distrib_theta(eng);

		// Temporary variables to simplify the expressions on the right hand side
	    double x = particles[i].x;
	    double y = particles[i].y;
	    double theta = particles[i].theta;

	    // Don't forget to exclude division by zero!    
	    if (yaw_rate != 0.0f)
	    {
			particles[i].x = x + velocity/yaw_rate * (sin(theta+yaw_rate*delta_t)-sin(theta)) + err_y; 
			particles[i].y = y + velocity/yaw_rate * (-cos(theta+yaw_rate*delta_t)+cos(theta)) + err_y;
			particles[i].theta = theta + yaw_rate * delta_t + 0.5f * delta_t*delta_t * nu_psidd + err_theta;
	    }
	    else if (yaw_rate == 0.0f)
	    {
			particles[i].x = x + velocity * cos(theta) * delta_t + err_x;
			particles[i].y = y + velocity * sin(theta) * delta_t + err_y;
			particles[i].theta = err_theta;
	    }
	}
}





void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) 
{
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	for (int i=0; i<; i++)
	{
		for (int j=0; j<; j++)
		{
			// std::map someone?!?
			dist(i,j);
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) 
{
	// TODO: Update the weights of each particle using a multi-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	dataAssociation();

	for (int i=0; i<num_landmarks; i++)
	{
		float factor = 1. / (sqrt(2 * M_PI) * sigma_x * sigma_y);
		factor * exp(-0.5f * pow((x-mu_x)/sigma_x, 2) -0.5f * pow((y-mu_y)/sigma_y, 2) );	
	}

	// A simple rotation
	x_prime = x*cos(phi) - y*sin(phi);
	y_prime = x*sin(phi) - y*cos(phi);
		
	// A rotation
	x_prime = x + t_x;
	y_prime = y + t_y;

	// Now use homogeneous coordinates:
	// Translation matrix
	MatrixXd translation = MatrixXd(3, 3);
	translation << 1, 0, t_x,
				   0, 1, t_y
				   0, 0, 1; 

	// Rotation matrix
	MatrixXd rotation = MatrixXd(3, 3);
	rotation << cos(phi), -sin(phi), 0,
				sin(phi),  cos(phi), 0,
				0, 0, 1;		

	// Initialize a vector:
	VectorXd vec = VectorXd(3);
	vec << x, y, 1;			 

	vec_prime = translation * vec;


	MatrixXd transformation = MatrixXd(3, 3);
	transformation << cos(phi), -sin(phi), t_x,
					  sin(phi), cos(phi), t_y
					  0, 0, 1;

	// How to concatenate homogeneous transformations?


	for (int i=0; i<num_particles; i++)
	{
		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;

		// iterate through the list of detected objects and
		// 1) transform detected object coordinates from vehicle to world coordinates (first rotation, then translation)
		// 2) perform a nearest neighbor search
		// 3) ...
		// 4) profit?

		for (int j; j<num_objects; j++)
		{
			double obj_x;
			double obj_y;

			obj_x_world = obj_x*cos(theta) - obj_y*sin(theta) - x;
			obj_y_world = obj_x*sin(theta) + obj_y*cos(theta) - y;
		}

	}


}


void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	// Multiply the exponential functions
	double weight = 1.0f;
	for (int i=0; i<N; i++)
	{
		weight *= P(x[i],mu_x[i]);
	}
	


}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

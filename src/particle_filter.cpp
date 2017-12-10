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
#include "map.h"

// Included it for coordinate transformations
#include "Eigen/Dense"


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
	std::normal_distribution<double> n_distrib_x(0, std[0]);
	std::normal_distribution<double> n_distrib_y(0, std[1]);
	std::normal_distribution<double> n_distrib_theta(0, std[2]);


	for (int i=0; i<num_particles; ++i)
	{
		Particle tmp_particle;
		tmp_particle.x = x + n_distrib_x(eng);
		tmp_particle.y = y + n_distrib_y(eng);
		tmp_particle.theta = theta + n_distrib_theta(eng);
		tmp_particle.weight = 1.0;

		particles.push_back(tmp_particle);
	}


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

	for (int i=0; i<num_particles; ++i)
	{	
		// Generate new error terms
		err_x = n_distrib_x(eng);
		err_y = n_distrib_y(eng);
		err_theta = n_distrib_theta(eng);

		// Temporary variables to simplify the expressions on the right hand side
	    double x = particles[i].x;
	    double y = particles[i].y;
	    double theta = particles[i].theta;

	    // Implementation of bicycle model.
	    // Don't forget to exclude division by zero!    
	    if (yaw_rate != 0.0f)
	    {
			particles[i].x = x + velocity/yaw_rate * (sin(theta+yaw_rate*delta_t)-sin(theta)) + err_y; 
			particles[i].y = y + velocity/yaw_rate * (-cos(theta+yaw_rate*delta_t)+cos(theta)) + err_y;
			particles[i].theta = theta + yaw_rate * delta_t + 0.5f * delta_t*delta_t * err_theta;
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
	/*
	for (auto i : predicted)
	{
		for (auto j : observations)
		{
			cout << "Distance between predicted and observed points: " << dist(i.x, i.y, j.x , j.y) << endl;
		}
	}*/
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], const std::vector<LandmarkObs> &observations, const Map &map_landmarks) 
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


	// The vector 'observations' comes without assigned ids.  (Which makes sense for today's sensors!)


	// Iterate over all particles.
	// Ignore the sensor's range for a moment.
	// Also ignore the sensor's error.
	for (int i=0; i < num_particles; ++i)
	{
		// Initialize the transformation matrices (World->Car)
		double theta = particles[i].theta;
		Eigen::MatrixXd rotation = Eigen::MatrixXd(3, 3);
		rotation << cos(theta), sin(theta), 0,
					-sin(theta), cos(theta), 0,
					0, 0, 1;

		double t_x = particles[i].x;
		double t_y = particles[i].y;
		Eigen::MatrixXd translation = Eigen::MatrixXd(3, 3);
		translation << 1, 0, -t_x,
					   0, 1, -t_y,
					   0, 0, 1;


		const double sigma_x = std_landmark[0];
		const double sigma_y = std_landmark[1];
		const double gauss_weight = 1. / (sqrt(2 * M_PI) * std_landmark[0] * std_landmark[1]);


		vector<LandmarkObs> predicted_landmarks;
		
		// TODO steps:
		// 0.) Determine whether the observation is in range or not
		// 1.) Predict positions of landmarks
		// 2.) Data association
		// 3.) Calculate the weights


		// 0.) Determine whether the observation is in range or not
		//
		// Iterate over all landmarks in the map:
		for (auto landmark_iterator : map_landmarks.landmark_list)
		{
			// Initialize the observed position of landmark:
			Eigen::VectorXd landmark_pos = Eigen::VectorXd(3);
			landmark_pos << landmark_iterator.x_f, landmark_iterator.y_f, 1;	

			// Calculate the car coordinates of the landmarks
			landmark_pos = rotation * translation * landmark_pos;

			// 1.) Predict positions of landmarks
			//
			// Save the world coordinates of the observations and associate them!
			if (dist(landmark_pos[0], 0., landmark_pos[1], 0.) <= sensor_range)
			{
				LandmarkObs tmp;
				tmp.x = landmark_pos[0];
				tmp.y = landmark_pos[1];
				tmp.id = landmark_iterator.id_i;
				predicted_landmarks.push_back(tmp);
			}
		}

		
		// 2.) Data association
		//
		// The Euclidean distance between two objects is Galileo invariant. 
		// Hence, we can either transform the landmarks of the map into car-coordinates and call them 'predictions' or 
		// transform the observations into world-coordinates.  My guess is that dataAssociation was designed for the former case.

		for (auto obs_it : observations)
		{
			double new_distance;
			double min_distance = 4.0; 
			LandmarkObs next_landmark;

			for (auto landmark_it : predicted_landmarks)
			{
				new_distance = dist(obs_it.x, landmark_it.x, obs_it.y, landmark_it.y);

				if (new_distance < min_distance) {
					min_distance = new_distance;	
					next_landmark = landmark_it;
				}
			}
			cout << "Distance between predicted and observed points: " << min_distance << endl;

			// 3.) Calculate the weights
			double exponential = pow((obs_it.x-next_landmark.x)/sigma_x, 2) + pow((obs_it.y-next_landmark.y)/sigma_y, 2);
			particles[i].weight *= gauss_weight * exp( -0.5f * exponential );
		}

	}
}




void ParticleFilter::resample() 
{
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	// http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	std::default_random_engine dre;
	std::mt19937 eng(dre());

	std::discrete_distribution<> ddist(weights.begin(), weights.end());

	//std::map<int, int> m;

	std::vector<Particle> particles_tmp = particles;

	for (int i=0; i<num_particles; ++i)
	{
		//++m[ddist(gen)];
		particles[i] = particles_tmp[ddist(eng)];
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







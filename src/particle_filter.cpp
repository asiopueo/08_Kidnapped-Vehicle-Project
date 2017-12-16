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

	num_particles = 50;

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
			particles[i].theta = theta + yaw_rate * delta_t + err_theta * 0.5f * delta_t*delta_t;   // Where did this factor come from?
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
		const double p_x = particles[i].x;
		const double p_y = particles[i].y;
		const double p_theta = particles[i].theta;

		const double sigma_x = std_landmark[0];
		const double sigma_y = std_landmark[1];
		const double gauss_weight = 1. / (2 * M_PI * std_landmark[0] * std_landmark[1]);

		// The following matrices shall be initialized when using 'Eigen'.		
		// Initialize the transformation matrices (World->Car)
		/*Eigen::MatrixXd rotation = Eigen::MatrixXd(3, 3);
		rotation << cos(theta), -sin(theta), 0,
					sin(theta), cos(theta), 0,
					0, 0, 1;

		Eigen::MatrixXd translation = Eigen::MatrixXd(3, 3);
		translation << 1, 0, p_x,
					   0, 1, p_y,
					   0, 0, 1;*/



		// TODO steps:
		// 0.) Determine whether the observation is in range or not
		// 1.) Predict positions of landmarks
		// 2.) Data association
		// 3.) Calculate the weights


		// 0.) Determine whether the observation is in range or not:

		vector<Map::single_landmark_s> landmark_candidates;

		// Uncomment the following line to display the particle's coordinates:
		//cout << "Particle no. " << i << ":\t\t" << p_x << "\t" << p_y << endl;

		// Iterate over all landmarks in the map:
		for (auto landmark_iterator : map_landmarks.landmark_list)
		{
			// 1.) Predict positions of landmarks
			//
			// Save the car coordinates of the observations and associate them!
			//if (dist(landmark_pos[0], 0., landmark_pos[1], 0.) <= sensor_range)

			/*	Reduce the list of landmarks to those within sensor range
			 *	increase the sensor range by a small percentage in order to incorporate deviation
			 *	of particle's position from ground truth.
			 */
			if (dist(landmark_iterator.x_f, landmark_iterator.y_f, p_x, p_y) <= sensor_range*1.1)
			{
				landmark_candidates.push_back(landmark_iterator);

				// Uncomment the following lines to display a list of potential landmarks within sensor range:
				/*cout << "Candidate landmark in sensor range: \t\t" 
					 << landmark_iterator.x_f << "\t" 
					 << landmark_iterator.y_f << "\t" 
					 << landmark_iterator.id_i << endl; 
				*/
			}
		}

		//Initialize the observed position of landmark:
		//Eigen::VectorXd landmark_pos = Eigen::VectorXd(3);
		//landmark_pos << landmark_iterator.x_f, landmark_iterator.y_f, 1;	

		


		// 2.) Data association
		//
		// The Euclidean distance between two objects is Galileo invariant. 
		// Hence, we can either transform the landmarks of the map into car-coordinates and call them 'predictions' or 
		// transform the observations into world-coordinates.
		// We will do the latter.
		
		for (auto obs_it : observations)
		{
			// Calculate the world coordinates of the observations:
			//landmark_pos = rotation * translation * landmark_pos;

			// temporary variables to avoid miscalculation in line (2) below:
			double tmp_x, tmp_y;

			tmp_x = obs_it.x * cos(p_theta)  -  obs_it.y * sin(p_theta) + p_x; // (1)
			tmp_y = obs_it.x * sin(p_theta)  +  obs_it.y * cos(p_theta) + p_y; // (2)

			obs_it.x = tmp_x;
			obs_it.y = tmp_y;

			double new_distance;
			double min_distance = 50.0; 
			LandmarkObs closest_landmark;

			// Find the nearest match from the candidate list (the selection from the map).
			for (auto cand_it : landmark_candidates)
			{
				new_distance = dist(obs_it.x, obs_it.y, cand_it.x_f, cand_it.y_f);

				if (new_distance < min_distance) {
					min_distance = new_distance;	
					// Three lines, since Map::single_landmark_s and LandmarkObs are different data structures
					closest_landmark.x = cand_it.x_f;
					closest_landmark.y = cand_it.y_f;
					closest_landmark.id = cand_it.id_i;
				}
			}

			if (min_distance < 50.0)
				cout << "Distance between predicted and observed points: " << min_distance << endl;


			// Uncomment the following lines to display the closest landmark:
			/*cout << "Closest landmark no." << closest_landmark.id << ":\t\t" 
				 << closest_landmark.x << "\t" 
				 << closest_landmark.y << endl; 
			*/

			// 3.) Calculate the weights
			double exponent = pow((obs_it.x-closest_landmark.x)/sigma_x, 2) + pow((obs_it.y-closest_landmark.y)/sigma_y, 2);
			particles[i].weight *= gauss_weight * exp( -0.5f * exponent );
			cout << particles[i].weight << endl;
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
	std::vector<Particle> particles_tmp = particles;

	for (int i=0; i<num_particles; ++i)
	{
		particles[i] = particles_tmp[ddist(eng)];
		particles[i].weight = 1.0;
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







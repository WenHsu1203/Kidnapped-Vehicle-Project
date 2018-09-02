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
#include "helper_functions.h"

using namespace std;

static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	normal_distribution<double> dist_x(x,std[0]);
	normal_distribution<double> dist_y(y,std[1]);
	normal_distribution<double> dist_theta(theta,std[2]);
	// NUMBERS to be tweaked
	num_particles = 1000;
	for (int i = 0 ; i < num_particles; i ++)
	{
		Particle New_Partilce;
		New_Partilce.id = i+1;
		New_Partilce.x = dist_x(gen);
		New_Partilce.y = dist_y(gen);
		New_Partilce.theta = dist_theta(gen);
		New_Partilce.weight = 1.0;
		particles.push_back(New_Partilce);
		weights.push_back(New_Partilce.weight);
	}

	is_initialized = true;


}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	normal_distribution<double> dist_x(0.0,std_pos[0]);
	normal_distribution<double> dist_y(0.0,std_pos[1]);
	normal_distribution<double> dist_theta(0.0,std_pos[2]);

	for (int i =0; i <num_particles ; i++)
	{	if(yaw_rate >= 0.00001)
		{
			particles[i].x = particles[i].x + velocity/yaw_rate * (sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta))+dist_x(gen);
			particles[i].y = particles[i].y + velocity/yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta+yaw_rate*delta_t))+dist_y(gen);
			particles[i].theta = particles[i].theta + yaw_rate*delta_t + dist_theta(gen);
		}
		else
		{
			particles[i].x = particles[i].x + velocity * cos(particles[i].theta) * delta_t + dist_x(gen);
			particles[i].y = particles[i].y + velocity * sin(particles[i].theta) * delta_t + dist_y(gen);
			particles[i].theta = particles[i].theta + dist_theta(gen);
		}
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (int i = 0 ; i < observations.size(); i++)
	{
		double min_dist = dist(predicted[0].x, predicted[0].y, observations[i].x,observations[i].y);
		int min_id = predicted[0].id;
		for (int j = 1; j < predicted.size(); j++)
		{
			double dist_temp = dist(predicted[j].x, predicted[j].y, observations[i].x,observations[i].y);
			if(dist_temp < min_dist)
			{
				min_id = predicted[j].id;
				min_dist = dist_temp;
			}
		}
		observations[i].id = min_id;
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	double sig_x = std_landmark[0];
	double sig_y = std_landmark[1];

	for (int i = 0 ; i< particles.size(); i++)
	{
		vector<LandmarkObs> predicted;
		for (int j = 0 ; j < map_landmarks.landmark_list.size();j++)
		{
			double dist_temp = dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f);
			if(dist_temp <=sensor_range)
			{
				predicted.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i, 
												map_landmarks.landmark_list[j].x_f,
												map_landmarks.landmark_list[j].y_f});
			}
		}
		vector<LandmarkObs> transformed_Obs;
		for (int j = 0; j < observations.size(); j++)
		{
			double x_map = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y);
			double y_map = particles[i].y + (sin(particles[i].theta) * observations[j].x) + (cos(particles[i].theta) * observations[j].y);
			transformed_Obs.push_back(LandmarkObs{observations[j].id, x_map, y_map});
		}

		dataAssociation(predicted, transformed_Obs);
		particles[i].weight = 1.0;
		for (int j= 0; j <transformed_Obs.size();j++)
		{
			double x_obs = transformed_Obs[j].x;
			double y_obs = transformed_Obs[j].y;
			double mu_x;
			double mu_y;
			for (int k = 0; k < predicted.size(); k++)
			{
				if(predicted[k].id == transformed_Obs[j].id)
				{
					mu_x = predicted[k].x;
					mu_y = predicted[k].y;
				}
			}
			double gauss_norm= (1/(2 * M_PI * sig_x * sig_y));
			double exponent= ((x_obs - mu_x)*(x_obs - mu_x))/(2 * sig_x*sig_x) + ((y_obs - mu_y)*(y_obs - mu_y))/(2 * sig_y*sig_y);
			particles[i].weight *= gauss_norm * exp(-exponent);
		}
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	std::vector<Particle> particles_;
	int index = rand() % num_particles;
	double beta = 0.0;
	double m_w = weights[0];
	for (int i = 1; i < num_particles; i ++)
	{
		m_w = max(m_w, weights[i]);
	}
	for (int i = 0 ; i < num_particles ; i++)
	{
		beta += rand()/double(RAND_MAX) * 2.0 *m_w;
		while(beta > weights[index])
		{
			beta -= weights[index];
			index = (index+1)%num_particles;
		}
		particles_.push_back(particles[index]);
	}
	particles = particles_;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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

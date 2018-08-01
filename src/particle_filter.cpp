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

typedef unsigned int uint;

#define NUM_PARTICLE   (int)100

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	/* TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	     x, y, theta and their uncertainties from GPS) and all weights to 1. 
	   Add random Gaussian noise to each particle.
	   NOTE: Consult particle_filter.h for more information about this method (and others in this file). */
		 
	std::default_random_engine gen;
	
	num_particles = NUM_PARTICLE;

	/* define normal distributions for sensor noise*/
	normal_distribution<double> N_x(x, std[0]);
	normal_distribution<double> N_y(y, std[1]);
	normal_distribution<double> N_theta(0, std[2]);

    // init particles
    for (int i = 0; i < num_particles; i++) 
	{
		Particle particle;
		particle.id = i;
		particle.x = N_x(gen);
		particle.y = N_y(gen);
		particle.theta = N_theta(gen);
		particle.weight = 1.0;
		
		particles.push_back(particle);
		weights.push_back(1);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	/* TODO: Add measurements to each particle and add random Gaussian noise.
	   NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	   http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	   http://www.cplusplus.com/reference/random/default_random_engine/ */
	
	std::default_random_engine gen;
	
	for(int i = 0; i < num_particles; i++)
	{
		double x_0 = particles[i].x;
		double y_0 = particles[i].y;
		double theta_0 = particles[i].theta;
		
		double x_f = x_0 + velocity / yaw_rate * (sin(theta_0 + yaw_rate * delta_t) - sin(theta_0));
		double y_f = y_0 + velocity / yaw_rate * (cos(theta_0) - cos(theta_0 + yaw_rate * delta_t));
		double theta_f = theta_0 + yaw_rate * delta_t;
		
		normal_distribution<double> N_x(x_f, std_pos[0]);
	    normal_distribution<double> N_y(y_f, std_pos[1]);
		normal_distribution<double> N_theta(theta_f, std_pos[2]);
		
		particles[i].x = N_x(gen);
		particles[i].y = N_y(gen);
		particles[i].theta = N_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> Landmarks, std::vector<LandmarkObs>& observations) {
	/* TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	     observed measurement to this particular landmark.
	   NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	     implement this method and use it as a helper during the updateWeights phase. */
		 
	for(uint i = 0; i < observations.size(); i++)
	{
		double x_o = observations[i].x;
		double y_o = observations[i].y;
		double MinDist = dist(x_o, y_o, Landmarks[0].x, Landmarks[0].y);
		
		for (uint j = 0; j < Landmarks.size(); j++)
		{
			double CurrDist = dist(x_o, y_o, Landmarks[j].x, Landmarks[j].y);
			if(CurrDist < MinDist)
			{
				observations[i].id = Landmarks[j].id;
			}
		}
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
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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

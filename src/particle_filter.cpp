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
    } 

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	/* TODO: Add measurements to each particle and add random Gaussian noise.
	   NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	   http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	   http://www.cplusplus.com/reference/random/default_random_engine/ */
	
 	std::default_random_engine gen;
	
	normal_distribution<double> N_x(0, std_pos[0]);
	normal_distribution<double> N_y(0, std_pos[1]);
	normal_distribution<double> N_theta(0, std_pos[2]);
	
	for(int i = 0; i < num_particles; i++)
	{
		double x_0 = particles[i].x;
		double y_0 = particles[i].y;
		double theta_0 = particles[i].theta;
		
		double x_f;
		double y_f;
		double theta_f;
		
		if(yaw_rate != 0)
		{
			x_f = x_0 + velocity / yaw_rate * (sin(theta_0 + yaw_rate * delta_t) - sin(theta_0));
			y_f = y_0 + velocity / yaw_rate * (cos(theta_0) - cos(theta_0 + yaw_rate * delta_t));
			theta_f = theta_0 + yaw_rate * delta_t;
		}
		else
		{
			x_f = x_0 + velocity * delta_t * cos(theta_0);
			y_f = y_0 + velocity * delta_t * sin(theta_0);
			theta_f = particles[i].theta;
		}
		
		particles[i].x = x_f + N_x(gen);
		particles[i].y = y_f + N_y(gen);
		particles[i].theta = theta_f + N_theta(gen);
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
		double MinDist = numeric_limits<double>::max();
		
		for (uint j = 0; j < Landmarks.size(); j++)
		{
			double CurrDist = dist(x_o, y_o, Landmarks[j].x, Landmarks[j].y);
			if(CurrDist < MinDist)
			{
				MinDist = CurrDist;
				observations[i].id = Landmarks[j].id;
			}
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	/* TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	     more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	   NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	     according to the MAP'S coordinate system. You will need to transform between the two systems.
	     Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	     The following is a good resource for the theory:
	     https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	     and the following is a good resource for the actual equation to implement (look at equation 
	     3.33
	     http://planning.cs.uiuc.edu/node99.html */
		 
	/*  Weight update process
	 *  1. Transform the observations into map coodination regarding to the 
	 *  2. Extract the landmark, which are found in sensor range
	 *  3. Associate the observations to the extracted landmarks
	 *  4. Calculate weights
	 */
	
  	for( int i = 0; i < num_particles; i++)
	{
		double x_p = particles[i].x;
		double y_p = particles[i].y;
		double theta_p = particles[i].theta;
		
		/* Transform the observations into map coodination regarding to the*/
  		vector<LandmarkObs> TransformedObs;
		LandmarkObs TransformObs;
		
		for(uint j = 0; j < observations.size(); j++)
		{
			TransformObs.x = cos(theta_p) * observations[j].x - sin(theta_p) * observations[j].y + x_p;
			TransformObs.y = sin(theta_p) * observations[j].x + cos(theta_p) * observations[j].y + y_p;
			TransformObs.id = observations[j].id;
			TransformedObs.push_back(TransformObs);
		}
		
		/* Extract the landmark, which are found in sensor range*/
   		vector<LandmarkObs> ExtractedLandmarks;
		LandmarkObs Landmark;
		for(uint j = 0; j < map_landmarks.landmark_list.size(); j++)
		{
			double Dist = dist(x_p, y_p, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f);
			
			if( Dist < sensor_range)
			{
				Landmark.x = map_landmarks.landmark_list[j].x_f;
				Landmark.y = map_landmarks.landmark_list[j].y_f;
				Landmark.id = map_landmarks.landmark_list[j].id_i;
				
				ExtractedLandmarks.push_back(Landmark);
			}
		}
		
		/* Associate the observations to the extracted landmarks*/
		dataAssociation(ExtractedLandmarks, TransformedObs);
		particles[i].weight = 1.0;
		
		/* Calculate weights */
		for( uint j = 0; j < TransformedObs.size(); j++)
		{
			double x = TransformedObs[j].x;
			double y = TransformedObs[j].y;
			double M_x = 0;
			double M_y = 0;
			/* Looking for the nearest landmark from the extracted landmark*/
			for (uint k = 0; k < ExtractedLandmarks.size(); k++)
			{
				if(TransformedObs[j].id == ExtractedLandmarks[k].id)
				{
					M_x = ExtractedLandmarks[k].x;
					M_y = ExtractedLandmarks[k].y;
				}
			}
			
			particles[i].weight *= exp(-pow(x - M_x,2) / 2 / pow(std_landmark[0], 2) - pow(y - M_y, 2) / 2 / pow(std_landmark[1],2))
									/ 2 / M_PI / std_landmark[0] / std_landmark[1];
		}
		
	}	
}

void ParticleFilter::resample() {
	/* TODO: Resample particles with replacement with probability proportional to their weight. 
	   NOTE: You may find std::discrete_distribution helpful here.
	     http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution */
	
	default_random_engine gen;
	vector<Particle> resample_particles;
	vector<double> weights;

	/* get all weights from the particles*/
 	double maxWeight = numeric_limits<double>::min();
 	for (int i = 0; i < num_particles; i++) 
	{
		weights.push_back(particles[i].weight);
		if(particles[i].weight > maxWeight) 
		{
			maxWeight = particles[i].weight;
		}
    }
	
 	uniform_real_distribution<double> distribution(0.0, maxWeight);
	uniform_int_distribution<int> DistInt(0, num_particles - 1);
	int index = DistInt(gen);
	double beta = 0.0;
	
	/* Sampling*/
	for(int i = 0; i < num_particles; i++) 
	{
		beta += distribution(gen) * 2.0;
		while(weights[index] < beta) 
		{
			beta -= weights[index];
			index = (index + 1) % num_particles;
		}
		resample_particles.push_back(particles[index]);
	}
	particles = resample_particles;
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

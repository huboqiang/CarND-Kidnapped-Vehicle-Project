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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  
  num_particles = 100;
  
  normal_distribution<double> dist_x(  x,   std[0]);
  normal_distribution<double> dist_y(  y,   std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  default_random_engine gen;
  for(int i=0; i<num_particles; i++){
    Particle pl;
    pl.id = i;
    pl.x  = dist_x(gen);
    pl.y  = dist_y(gen);
    pl.theta  = dist_theta(gen);
    pl.weight = 1.0;
    particles.push_back(pl);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  
  normal_distribution<double> dist_x(0.0,   std_pos[0]);
  normal_distribution<double> dist_y(0.0,   std_pos[1]);
  normal_distribution<double> dist_theta(0.0, std_pos[2]);
  default_random_engine gen;
  
  for(int i=0; i<num_particles; i++){
    Particle pl = particles[i];
    if(yaw_rate < 0.001){
      particles[i].x += velocity *delta_t * cos(pl.theta) + dist_x(gen);
      particles[i].y += velocity *delta_t * sin(pl.theta) + dist_y(gen);
    }
    else{
      particles[i].x += velocity / yaw_rate * (sin(pl.theta+yaw_rate*delta_t) - sin(pl.theta)) + dist_x(gen);
      particles[i].y += velocity / yaw_rate * (cos(pl.theta) - cos(pl.theta+yaw_rate*delta_t)) + dist_y(gen);
    }
    particles[i].theta += yaw_rate*delta_t + dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
  for(int i_tobs=0; i_tobs<observations.size(); i_tobs++){
    double tobs_x = observations[i_tobs].x;
    double tobs_y = observations[i_tobs].y;
    double min_distVal = 100000.0;  // a very big value
    int    min_distIdx = 0;
    for(int i_ldmk=0; i_ldmk<predicted.size(); i_ldmk++){
      double distance = dist(tobs_x, tobs_y, predicted[i_ldmk].x, predicted[i_ldmk].y);
      if(distance < min_distVal){
        min_distVal = distance;
        min_distIdx = i_ldmk;
      }
    }
    observations[i_tobs].id = min_distIdx;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
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
  
  /*
      I just tried to convert the idea in Lesson14, Slide13 into the codes below.
      In this slides, this step were divided into sub-tasks:
        
        1. Transform each observation marker from the vehicle's coordinates to 
              the map's coordinates, in respect to our particle.
        
        2. Associate each transformed observation with a land mark identifier.
   
        3. The particles final weight will be calculated as the product of 
              each measurement's Multivariate-Gaussian probability.
   
   */
  
  for(int i=0; i<num_particles; i++){
    
    /* 
       Step0, convert map.landmark_list to the
              LandmarkObs structure
    */
    vector<LandmarkObs> ldmk_ObsMap;
    for(int j=0; j<map_landmarks.landmark_list.size(); j++){
      LandmarkObs lm;
      lm.id= map_landmarks.landmark_list[j].id_i;
      lm.x = map_landmarks.landmark_list[j].x_f;
      lm.y = map_landmarks.landmark_list[j].y_f;
      if( dist(lm.x, lm.y, particles[i].x, particles[i].y) < sensor_range){
        ldmk_ObsMap.push_back(lm);
      }
    }
    
    
    /*
       Step1, transformation from OBSx to TOBSx.
              The observations are given in the VEHICLE'S coordinate system.
              These codes would first convert the VEHICLE's coordinate 
              obs to the MAP's coord.
     */
    vector<LandmarkObs> vec_tObs;
    for(int j=0; j<observations.size(); j++){
      LandmarkObs tObs;
      
      tObs.x  = particles[i].x + observations[j].x*cos(particles[i].theta) -
                                 observations[j].y*sin(particles[j].theta);
      tObs.y  = particles[i].y + observations[j].x*sin(particles[i].theta) +
                                 observations[j].y*cos(particles[j].theta);
      vec_tObs.push_back(tObs);
    }
    
    /*
       Step2, finding the nearest landmarks for each TOBSx.
              each TOBs should have a ID for a landmark, which 
              is closest to this landmark.
     */
    dataAssociation(ldmk_ObsMap, vec_tObs);

    
    /*
       Step3, calculating the particle's final weight
     */
    particles[i].weight = 1;
    vector<double> vec_senseX;
    vector<double> vec_senseY;
    vector<int> vec_associations;
    
    for(int j=0; j<vec_tObs.size(); j++){
      double ox = vec_tObs[j].x;
      double oy = vec_tObs[j].y;
      int    associateId = vec_tObs[j].id;
      
      double ux = ldmk_ObsMap[associateId].x;
      double uy = ldmk_ObsMap[associateId].y;

      double up = (ox - ux)*(ox - ux)/(2*std_landmark[0]*std_landmark[0]) +
                  (oy - uy)*(oy - uy)/(2*std_landmark[1]*std_landmark[1]);
      
      double Pxy = 1.0 / (2*M_PI*std_landmark[0]*std_landmark[1]) * pow(M_E, -1*up);
      vec_senseX.push_back(ox);
      vec_senseY.push_back(oy);
      vec_associations.push_back(associateId);
      particles[i].weight *= Pxy;
    }
    SetAssociations(particles[i], vec_associations, vec_senseX, vec_senseY);
  }
  
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  
  /*
    Convert python code below to cpp code:
   
   ```python
   p3 = []
   index = int(random.random() * N)
   beta = 0.0
   mw = max(w)
   for i in range(N):
       beta += random.random() * 2.0 * mw
       while beta > w[index]:
           beta -= w[index]
           index = (index + 1) % N
       p3.append(p[index])
   
   p = p3
   ```
  */
  
  default_random_engine gen;
  
  vector<Particle> new_particles;
  
  vector<double> weights;
  for(int i=0; i<num_particles; i++){
    weights.push_back(particles[i].weight);
  }
  uniform_real_distribution<double> unirealdist(0.0, 1.0);
  int index   = int(unirealdist(gen) * num_particles);
  double beta = 0.0;
  double mw   = *max_element(weights.begin(), weights.end());
  for(int i=0; i<num_particles; i++){
    beta += unirealdist(gen) * 2.0 * mw;
    while(beta > weights[index]){
      beta -= weights[index];
      index = (index+1) % num_particles;
    }
    new_particles.push_back(particles[index]);
  }
  particles = new_particles;
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

// my first program in C++
#include <iostream>
#include <vector>
#include <Eigen/Dense>

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
typedef Eigen::Matrix<Scalar, 1, 1> Vector1;


double computeProbabilityOfCollisionNPositionsKDTree(Vector3 const& robot_position, Vector3 const& sigma_robot_position, std::vector<Vector3> const& closest_pts, double interpolation_radius, bool print) {
  int num_nearest_neighbors = 1;
  Vector3 sigma_depth_point = Vector3(0.01, 0.01, 0.01);

  double probability_no_collision = 1.0;

  if (closest_pts.size() > 0) {
    for (size_t i = 0; i < std::min((int)closest_pts.size(), num_nearest_neighbors); i++) {

      Vector3 depth_position = closest_pts[i];

      //interpolate towards robot
      double norm = (robot_position - depth_position).norm();
      if (norm >= 2*interpolation_radius) {
        depth_position = depth_position + (robot_position - depth_position)/norm*interpolation_radius;
      } else {
        depth_position = depth_position + (robot_position - depth_position)*0.5;
      }

      Vector3 sigma_robot_position_nn = sigma_robot_position*1.0;
      for (int i = 0; i <3; i++) {
        if (sigma_robot_position_nn(i) < 0) {
          sigma_robot_position_nn(i)=-sigma_robot_position_nn(i);
        }
      }
      Vector3 total_sigma = sigma_robot_position_nn + sigma_depth_point;
      Vector3 inverse_total_sigma = Vector3(1/total_sigma(0), 1/total_sigma(1), 1/total_sigma(2));  
    
      double volume = 0.267; // 4/3*pi*r^3, with r=0.4 as first guess
      double denominator = std::sqrt( 248.05021344239853*(total_sigma(0))*(total_sigma(1))*(total_sigma(2)) ); // coefficient is 2pi*2pi*2pi
      double exponent = -0.5*(robot_position - depth_position).transpose() * inverse_total_sigma.cwiseProduct(robot_position - depth_position);

      double probability_of_collision = volume / denominator * pow(2.71828,exponent);

      probability_no_collision = probability_no_collision * (1 - probability_of_collision);
      
      if (0) {
        std::cout << "depth position      " << depth_position.transpose() << std::endl;
        std::cout << "robot position      " << robot_position.transpose() << std::endl;
        std::cout << "total_sigma         " << total_sigma.transpose() << std::endl;
        std::cout << "prob collision      " << 1 - probability_no_collision << std::endl;
      }

    }

    return 1 - probability_no_collision;
  }
  return 0.0; // if no points in closest_pts
}

void generateSamples() {
	std::cout << "Gen samples" << std::endl;

	// --------- Parameters ---------

	// robot_position
	Vector3 robot_position = Vector3(0.0, 0.0, 3.0);
	
	// sigma_robot_position
	Vector3 sigma_robot_position = Vector3(0.1, 0.1, 0.1);
	
	// closest_pts
	std::vector<Vector3> closest_pts;
	closest_pts.push_back(Vector3(0.0, 0.0, 3.0));

	// interpolation_radius
	double interpolation_radius = 0.3;


	std::cout << std::endl;
	std::cout << "sigma_robot_position " << sigma_robot_position.transpose() << std::endl;

	for (int i = 0; i < 60; i++) {
		robot_position = Vector3(-3.0 + i*0.1, 0.0, 3.0);
		double probability = computeProbabilityOfCollisionNPositionsKDTree(robot_position, sigma_robot_position, closest_pts, interpolation_radius, 0);
		std::cout << "robot_position: " << robot_position.transpose() << std::endl;
		std::cout << "Probability: " << probability << std::endl;
	}

	sigma_robot_position = Vector3(1.0, 1.0, 1.0);
	std::cout << std::endl;
	std::cout << "sigma_robot_position " << sigma_robot_position.transpose() << std::endl;
	

	for (int i = 0; i < 60; i++) {
		robot_position = Vector3(-3.0 + i*0.1, 0.0, 3.0);
		double probability = computeProbabilityOfCollisionNPositionsKDTree(robot_position, sigma_robot_position, closest_pts, interpolation_radius, 0);
		std::cout << "robot_position: " << robot_position.transpose() << std::endl;
		std::cout << "Probability: " << probability << std::endl;
	}

	sigma_robot_position = Vector3(3.0, 3.0, 3.0);
	std::cout << std::endl;
	std::cout << "sigma_robot_position " << sigma_robot_position.transpose() << std::endl;
	
	for (int i = 0; i < 60; i++) {
		robot_position = Vector3(-3.0 + i*0.1, 0.0, 3.0);
		double probability = computeProbabilityOfCollisionNPositionsKDTree(robot_position, sigma_robot_position, closest_pts, interpolation_radius, 0);
		std::cout << "robot_position: " << robot_position.transpose() << std::endl;
		std::cout << "Probability: " << probability << std::endl;
	}


}

int main()
{
	generateSamples();
}
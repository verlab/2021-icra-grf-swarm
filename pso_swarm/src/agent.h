#ifndef ORCA_AGENT_H_
#define ORCA_AGENT_H_


#include "ros/ros.h"
//#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

#include "line.h"
#include "definitions.h"
#include "kd_tree.h"
#include "orca_simulator.h"

#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif

//namespace ORCA{

	class ORCA_Simulator;
	class Obstacle;

	class Base : public Vector2{
		public:
	    Base(){}

	    void baseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	        double roll, pitch, heading;
	        set_x(msg->pose.pose.position.x);
	        set_y(msg->pose.pose.position.y);

	        tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, \
	                                          msg->pose.pose.orientation.y, \
	                                          msg->pose.pose.orientation.z, \
	                                          msg->pose.pose.orientation.w);
	        tf::Matrix3x3(q).getRPY(roll, pitch, heading);
	    }
	};

	class Pose : public Vector2{
		public:

	    Pose() {}
	    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	        double roll, pitch, heading;
			set_x(msg->pose.pose.position.x);
	        set_y(msg->pose.pose.position.y);

	        tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, \
	                                          msg->pose.pose.orientation.y, \
	                                          msg->pose.pose.orientation.z, \
	                                          msg->pose.pose.orientation.w);
	        tf::Matrix3x3(q).getRPY(roll, pitch, heading);
	    }

	    /*void poseCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg) {
	        //double roll, pitch, heading;
			set_x(msg->pose.pose.position.x);
	        set_y(msg->pose.pose.position.y);

	        //tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, \
	                                          //msg->pose.pose.orientation.y, \
	                                          //msg->pose.pose.orientation.z, \
	                                          //msg->.pose.pose.orientation.w);
	        //tf::Matrix3x3(q).getRPY(roll, pitch, heading);
	    }*/
	};


	class Agent {
		private:
		std::vector<std::pair<float, const Agent *> > agentNeighbors_;
		int maxNeighbors_;
		float maxSpeed_;
		Vector2 newVelocity_;
		std::vector<std::pair<float, const Obstacle *> > obstacleNeighbors_;
		std::vector<Line> orcaLines_;
		ORCA_Simulator *sim_;
		float timeHorizon_;
		float timeHorizonObst_;
		Pose pose_;
		Base base_;

		friend class KdTree;
		friend class ORCA_Simulator;

		public:
		int id_;
		float radius_;
		float neighborDist_;
		float heading_;
		Vector2 prefVelocity_;
		Vector2 velocity_;
		Vector2 flockVel_;
		Vector2 goal_;
		Vector2 v1_;
		Vector2 v2_;
		Vector2 v3_;
		Vector2 neighbor_;
		Vector2 pVel_;
		Vector2 gBestPosition_;
		Vector2 goalVector_;
		int emptyMemory_;

		int flock_id_;

		float fitness_;
		
		int numNeighbors_;
		std::vector<std::pair<int, Vector2>> memory_;
		std::pair<int, Vector2> otherMemory_;



		Vector2 position_;
		int other_flock_;
		int follower_;
		int free_vision_;
		int vision_;
		int turn_right_;
		Vector2 leader_position_;


		/**
		 * \brief      Constructs an agent instance.
		 * \param      sim             The simulator instance.
		 */
		explicit Agent(ORCA_Simulator *sim);

		void setFlockID(int f);

		void move(double linVelx, double angVelz);

		/**
		 * \brief      Computes the neighbors of this agent.
		 */
		void computeNeighbors();

		/**
		 * \brief      Computes the new velocity of this boid.
		 */
		void computeNewBoidVelocity(int *vec);

		/**
		 * \brief      Computes the new goal of this boid.
		 */
		void computeNewPSOVelocity(int numAgents, int numFlocks);

		/**
		 * \brief      Computes the new velocity of this agent.
		 */
		void computeNewVelocity();

		/**
		 * \brief      Inserts an agent neighbor into the set of neighbors of
		 *             this agent.
		 * \param      agent           A pointer to the agent to be inserted.
		 * \param      rangeSq         The squared range around this agent.
		 */
		void insertAgentNeighbor(const Agent *agent, float &rangeSq);

		/**
		 * \brief      Inserts a static obstacle neighbor into the set of neighbors
		 *             of this agent.
		 * \param      obstacle        The number of the static obstacle to be
		 *                             inserted.
		 * \param      rangeSq         The squared range around this agent.
		 */
		void insertObstacleNeighbor(const Obstacle *obstacle, float rangeSq);

		/**
		 * \brief      Updates the two-dimensional position and two-dimensional
		 *             velocity of this agent.
		 */
		void update();
	};

	/**
	 * \relates    Agent
	 * \brief      Solves a one-dimensional linear program on a specified line
	 *             subject to linear constraints defined by lines and a circular
	 *             constraint.
	 * \param      lines         Lines defining the linear constraints.
	 * \param      lineNo        The specified line constraint.
	 * \param      radius        The radius of the circular constraint.
	 * \param      optVelocity   The optimization velocity.
	 * \param      directionOpt  True if the direction should be optimized.
	 * \param      result        A reference to the result of the linear program.
	 * \return     True if successful.
	 */
	bool linearProgram1(const std::vector<Line> &lines, int lineNo,
						float radius, const Vector2 &optVelocity,
						bool directionOpt, Vector2 &result);

	/**
	 * \relates    Agent
	 * \brief      Solves a two-dimensional linear program subject to linear
	 *             constraints defined by lines and a circular constraint.
	 * \param      lines         Lines defining the linear constraints.
	 * \param      radius        The radius of the circular constraint.
	 * \param      optVelocity   The optimization velocity.
	 * \param      directionOpt  True if the direction should be optimized.
	 * \param      result        A reference to the result of the linear program.
	 * \return     The number of the line it fails on, and the number of lines if successful.
	 */
	int linearProgram2(const std::vector<Line> &lines, float radius,
						  const Vector2 &optVelocity, bool directionOpt,
						  Vector2 &result);

	/**
	 * \relates    Agent
	 * \brief      Solves a two-dimensional linear program subject to linear
	 *             constraints defined by lines and a circular constraint.
	 * \param      lines         Lines defining the linear constraints.
	 * \param      numObstLines  Count of obstacle lines.
	 * \param      beginLine     The line on which the 2-d linear program failed.
	 * \param      radius        The radius of the circular constraint.
	 * \param      result        A reference to the result of the linear program.
	 */
	void linearProgram3(const std::vector<Line> &lines, int numObstLines, int beginLine,
						float radius, Vector2 &result);
//}

#endif /* ORCA_AGENT_H_ */

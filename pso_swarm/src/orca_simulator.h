#ifndef ORCA_ORCA_SIMULATOR_H_
#define ORCA_ORCA_SIMULATOR_H_

#include <cstddef>
#include <limits>
#include <vector>

#include "ros/ros.h"
#include "vector2.h"

#include "line.h"
#include "kd_tree.h"

//namespace ORCA {

	class Agent;
	class KdTree;
	class Obstacle;
	class Pose;


	/**
	 * \brief      Defines the simulation.
	 *
	 * The main class of the library that contains all simulation functionality.
	 */
	class ORCA_Simulator {
		public:
		std::vector<Agent *> agents_;
		Agent *defaultAgent_;
		float globalTime_;
		KdTree *kdTree_;
		std::vector<Obstacle *> obstacles_;
		float timeStep_;

		friend class Agent;
		friend class KdTree;
		friend class Obstacle;

		public:

		/**
		 * \brief      Constructs a simulator instance.
		 */
		ORCA_Simulator();

		/**
		 * \brief      Constructs a simulator instance and sets the default
		 *             properties for any new agent that is added.
		 * \param      timeStep        The time step of the simulation.
		 *                             Must be positive.
		 * \param      neighborDist    The default maximum distance (center point
		 *                             to center point) to other agents a new agent
		 *                             takes into account in the navigation. The
		 *                             larger this number, the longer he running
		 *                             time of the simulation. If the number is too
		 *                             low, the simulation will not be safe. Must be
		 *                             non-negative.
		 * \param      maxNeighbors    The default maximum number of other agents a
		 *                             new agent takes into account in the
		 *                             navigation. The larger this number, the
		 *                             longer the running time of the simulation.
		 *                             If the number is too low, the simulation
		 *                             will not be safe.
		 * \param      timeHorizon     The default minimal amount of time for which
		 *                             a new agent's velocities that are computed
		 *                             by the simulation are safe with respect to
		 *                             other agents. The larger this number, the
		 *                             sooner an agent will respond to the presence
		 *                             of other agents, but the less freedom the
		 *                             agent has in choosing its velocities.
		 *                             Must be positive.
		 * \param      timeHorizonObst The default minimal amount of time for which
		 *                             a new agent's velocities that are computed
		 *                             by the simulation are safe with respect to
		 *                             obstacles. The larger this number, the
		 *                             sooner an agent will respond to the presence
		 *                             of obstacles, but the less freedom the agent
		 *                             has in choosing its velocities.
		 *                             Must be positive.
		 * \param      radius          The default radius of a new agent.
		 *                             Must be non-negative.
		 * \param      maxSpeed        The default maximum speed of a new agent.
		 *                             Must be non-negative.
		 * \param      velocity        The default initial two-dimensional linear
		 *                             velocity of a new agent (optional).
		 */
		ORCA_Simulator(float timeStep, float neighborDist, int maxNeighbors,
					 float timeHorizon, float timeHorizonObst, float radius,
					 float maxSpeed, const Vector2 &velocity = Vector2());

		/**
		 * \brief      Destroys this simulator instance.
		 */
		~ORCA_Simulator();

		/**
		 * \brief      Adds a new agent with default properties to the
		 *             simulation.
		 * \param      position        The two-dimensional starting position of
		 *                             this agent.
		 * \return     The number of the agent, or ORCA::ORCA_ERROR when the agent
		 *             defaults have not been set.
		 */
		int addAgent( float x, float y, int numFlocks);//, const Vector2 &position);

		/**
		 * \brief      Adds a new agent to the simulation.
		 * \param      position        The two-dimensional starting position of
		 *                             this agent.
		 * \param      neighborDist    The maximum distance (center point to
		 *                             center point) to other agents this agent
		 *                             takes into account in the navigation. The
		 *                             larger this number, the longer the running
		 *                             time of the simulation. If the number is too
		 *                             low, the simulation will not be safe.
		 *                             Must be non-negative.
		 * \param      maxNeighbors    The maximum number of other agents this
		 *                             agent takes into account in the navigation.
		 *                             The larger this number, the longer the
		 *                             running time of the simulation. If the
		 *                             number is too low, the simulation will not
		 *                             be safe.
		 * \param      timeHorizon     The minimal amount of time for which this
		 *                             agent's velocities that are computed by the
		 *                             simulation are safe with respect to other
		 *                             agents. The larger this number, the sooner
		 *                             this agent will respond to the presence of
		 *                             other agents, but the less freedom this
		 *                             agent has in choosing its velocities.
		 *                             Must be positive.
		 * \param      timeHorizonObst The minimal amount of time for which this
		 *                             agent's velocities that are computed by the
		 *                             simulation are safe with respect to
		 *                             obstacles. The larger this number, the
		 *                             sooner this agent will respond to the
		 *                             presence of obstacles, but the less freedom
		 *                             this agent has in choosing its velocities.
		 *                             Must be positive.
		 * \param      radius          The radius of this agent.
		 *                             Must be non-negative.
		 * \param      maxSpeed        The maximum speed of this agent.
		 *                             Must be non-negative.
		 * \param      velocity        The initial two-dimensional linear velocity
		 *                             of this agent (optional).
		 * \return     The number of the agent.
		 */
		int addAgent(const Vector2 &position, float neighborDist,
						int maxNeighbors, float timeHorizon,
						float timeHorizonObst, float radius, float maxSpeed,
						const Vector2 &velocity = Vector2());

		void setAgentPosition(int i);

		/**
		 * \brief      Adds a new obstacle to the simulation.
		 * \param      vertices        List of the vertices of the polygonal
		 *             obstacle in counterclockwise order.
		 * \return     The number of the first vertex of the obstacle,
		 *             or ORCA::ORCA_ERROR when the number of vertices is less than two.
		 * \note       To add a "negative" obstacle, e.g. a bounding polygon around
		 *             the environment, the vertices should be listed in clockwise
		 *             order.
		 */
		int addObstacle(const std::vector<Vector2> &vertices);

		/**
		 * \brief      Lets the simulator perform a simulation step and updates the
		 *             two-dimensional position and two-dimensional velocity of
		 *             each boid.
		 */
		 void boidDoStep(int *vec);

		/**
		 * \brief      Lets the simulator perform a simulation step and updates the
		 *             two-dimensional position and two-dimensional velocity of
		 *             each PSOagent.
		 */
		void PSODoStep(int numAgents, int flockSize);


		/**
		 * \brief      Lets the simulator perform a simulation step and updates the
		 *             two-dimensional position and two-dimensional velocity of
		 *             each agent.
		 */
		void orcaDoStep();

		int mst();

		/**
		 * \brief      Returns the specified agent neighbor of the specified
		 *             agent.
		 * \param      agentNo         The number of the agent whose agent
		 *                             neighbor is to be retrieved.
		 * \param      neighborNo      The number of the agent neighbor to be
		 *                             retrieved.
		 * \return     The number of the neighboring agent.
		 */
		int getAgentAgentNeighbor(int agentNo, int neighborNo) const;

		/**
		 * \brief      Returns the maximum neighbor count of a specified agent.
		 * \param      agentNo         The number of the agent whose maximum
		 *                             neighbor count is to be retrieved.
		 * \return     The present maximum neighbor count of the agent.
		 */
		int getAgentMaxNeighbors(int agentNo) const;

		/**
		 * \brief      Returns the maximum speed of a specified agent.
		 * \param      agentNo         The number of the agent whose maximum speed
		 *                             is to be retrieved.
		 * \return     The present maximum speed of the agent.
		 */
		float getAgentMaxSpeed(int agentNo) const;

		/**
		 * \brief      Returns the maximum neighbor distance of a specified
		 *             agent.
		 * \param      agentNo         The number of the agent whose maximum
		 *                             neighbor distance is to be retrieved.
		 * \return     The present maximum neighbor distance of the agent.
		 */
		float getAgentNeighborDist(int agentNo) const;

		/**
		 * \brief      Returns the count of agent neighbors taken into account to
		 *             compute the current velocity for the specified agent.
		 * \param      agentNo         The number of the agent whose count of agent
		 *                             neighbors is to be retrieved.
		 * \return     The count of agent neighbors taken into account to compute
		 *             the current velocity for the specified agent.
		 */
		int getAgentNumAgentNeighbors(int agentNo) const;

		/**
		 * \brief      Returns the count of obstacle neighbors taken into account
		 *             to compute the current velocity for the specified agent.
		 * \param      agentNo         The number of the agent whose count of
		 *                             obstacle neighbors is to be retrieved.
		 * \return     The count of obstacle neighbors taken into account to
		 *             compute the current velocity for the specified agent.
		 */
		int getAgentNumObstacleNeighbors(int agentNo) const;


		/**
		 * \brief      Returns the count of ORCA constraints used to compute
		 *             the current velocity for the specified agent.
		 * \param      agentNo         The number of the agent whose count of ORCA
		 *                             constraints is to be retrieved.
		 * \return     The count of ORCA constraints used to compute the current
		 *             velocity for the specified agent.
		 */
		int getAgentNumORCALines(int agentNo) const;

		/**
		 * \brief      Returns the specified obstacle neighbor of the specified
		 *             agent.
		 * \param      agentNo         The number of the agent whose obstacle
		 *                             neighbor is to be retrieved.
		 * \param      neighborNo      The number of the obstacle neighbor to be
		 *                             retrieved.
		 * \return     The number of the first vertex of the neighboring obstacle
		 *             edge.
		 */
		int getAgentObstacleNeighbor(int agentNo, int neighborNo) const;

		/**
		 * \brief      Returns the specified ORCA constraint of the specified
		 *             agent.
		 * \param      agentNo         The number of the agent whose ORCA
		 *                             constraint is to be retrieved.
		 * \param      lineNo          The number of the ORCA constraint to be
		 *                             retrieved.
		 * \return     A line representing the specified ORCA constraint.
		 * \note       The halfplane to the left of the line is the region of
		 *             permissible velocities with respect to the specified
		 *             ORCA constraint.
		 */
		const Line &getAgentORCALine(int agentNo, int lineNo) const;

		/**
		 * \brief      Returns the two-dimensional position of a specified
		 *             agent.
		 * \param      agentNo         The number of the agent whose
		 *                             two-dimensional position is to be retrieved.
		 * \return     The present two-dimensional position of the (center of the)
		 *             agent.
		 */
		const Vector2 &getAgentPosition(int agentNo) const;


		const Vector2 &getAgentGoal(int agentNo) const;


		const Pose &getAgentPose(int agentNo) const;



		/**
		 * \brief      Returns the two-dimensional preferred velocity of a
		 *             specified agent.
		 * \param      agentNo         The number of the agent whose
		 *                             two-dimensional preferred velocity is to be
		 *                             retrieved.
		 * \return     The present two-dimensional preferred velocity of the agent.
		 */
		const Vector2 &getAgentPrefVelocity(int agentNo) const;

		/**
		 * \brief      Returns the radius of a specified agent.
		 * \param      agentNo         The number of the agent whose radius is to
		 *                             be retrieved.
		 * \return     The present radius of the agent.
		 */
		float getAgentRadius(int agentNo) const;

		/**
		 * \brief      Returns the time horizon of a specified agent.
		 * \param      agentNo         The number of the agent whose time horizon
		 *                             is to be retrieved.
		 * \return     The present time horizon of the agent.
		 */
		float getAgentTimeHorizon(int agentNo) const;

		/**
		 * \brief      Returns the time horizon with respect to obstacles of a
		 *             specified agent.
		 * \param      agentNo         The number of the agent whose time horizon
		 *                             with respect to obstacles is to be
		 *                             retrieved.
		 * \return     The present time horizon with respect to obstacles of the
		 *             agent.
		 */
		float getAgentTimeHorizonObst(int agentNo) const;

		/**
		 * \brief      Returns the two-dimensional linear velocity of a
		 *             specified agent.
		 * \param      agentNo         The number of the agent whose
		 *                             two-dimensional linear velocity is to be
		 *                             retrieved.
		 * \return     The present two-dimensional linear velocity of the agent.
		 */
		const Vector2 &getAgentVelocity(int agentNo) const;

		/**
		 * \brief      Returns the global time of the simulation.
		 * \return     The present global time of the simulation (zero initially).
		 */
		float getGlobalTime() const;

		/**
		 * \brief      Returns the count of agents in the simulation.
		 * \return     The count of agents in the simulation.
		 */
		int getNumAgents() const;

		/**
		 * \brief      Returns the count of obstacle vertices in the simulation.
		 * \return     The count of obstacle vertices in the simulation.
		 */
		int getNumObstacleVertices() const;

		/**
		 * \brief      Returns the two-dimensional position of a specified obstacle
		 *             vertex.
		 * \param      vertexNo        The number of the obstacle vertex to be
		 *                             retrieved.
		 * \return     The two-dimensional position of the specified obstacle
		 *             vertex.
		 */
		const Vector2 &getObstacleVertex(int vertexNo) const;

		/**
		 * \brief      Returns the number of the obstacle vertex succeeding the
		 *             specified obstacle vertex in its polygon.
		 * \param      vertexNo        The number of the obstacle vertex whose
		 *                             successor is to be retrieved.
		 * \return     The number of the obstacle vertex succeeding the specified
		 *             obstacle vertex in its polygon.
		 */
		int getNextObstacleVertexNo(int vertexNo) const;

		/**
		 * \brief      Returns the number of the obstacle vertex preceding the
		 *             specified obstacle vertex in its polygon.
		 * \param      vertexNo        The number of the obstacle vertex whose
		 *                             predecessor is to be retrieved.
		 * \return     The number of the obstacle vertex preceding the specified
		 *             obstacle vertex in its polygon.
		 */
		int getPrevObstacleVertexNo(int vertexNo) const;

		/**
		 * \brief      Returns the time step of the simulation.
		 * \return     The present time step of the simulation.
		 */
		float getTimeStep() const;

		/**
		 * \brief      Processes the obstacles that have been added so that they
		 *             are accounted for in the simulation.
		 * \note       Obstacles added to the simulation after this function has
		 *             been called are not accounted for in the simulation.
		 */
		void processObstacles();

		/**
		 * \brief      Performs a visibility query between the two specified
		 *             points with respect to the obstacles
		 * \param      point1          The first point of the query.
		 * \param      point2          The second point of the query.
		 * \param      radius          The minimal distance between the line
		 *                             connecting the two points and the obstacles
		 *                             in order for the points to be mutually
		 *                             visible (optional). Must be non-negative.
		 * \return     A boolean specifying whether the two points are mutually
		 *             visible. Returns true when the obstacles have not been
		 *             processed.
		 */
		bool queryVisibility(const Vector2 &point1, const Vector2 &point2,
							 float radius = 0.0f) const;

		/**
		 * \brief      Sets the default properties for any new agent that is
		 *             added.
		 * \param      neighborDist    The default maximum distance (center point
		 *                             to center point) to other agents a new agent
		 *                             takes into account in the navigation. The
		 *                             larger this number, the longer he running
		 *                             time of the simulation. If the number is too
		 *                             low, the simulation will not be safe.
		 *                             Must be non-negative.
		 * \param      maxNeighbors    The default maximum number of other agents a
		 *                             new agent takes into account in the
		 *                             navigation. The larger this number, the
		 *                             longer the running time of the simulation.
		 *                             If the number is too low, the simulation
		 *                             will not be safe.
		 * \param      timeHorizon     The default minimal amount of time for which
		 *                             a new agent's velocities that are computed
		 *                             by the simulation are safe with respect to
		 *                             other agents. The larger this number, the
		 *                             sooner an agent will respond to the presence
		 *                             of other agents, but the less freedom the
		 *                             agent has in choosing its velocities.
		 *                             Must be positive.
		 * \param      timeHorizonObst The default minimal amount of time for which
		 *                             a new agent's velocities that are computed
		 *                             by the simulation are safe with respect to
		 *                             obstacles. The larger this number, the
		 *                             sooner an agent will respond to the presence
		 *                             of obstacles, but the less freedom the agent
		 *                             has in choosing its velocities.
		 *                             Must be positive.
		 * \param      radius          The default radius of a new agent.
		 *                             Must be non-negative.
		 * \param      maxSpeed        The default maximum speed of a new agent.
		 *                             Must be non-negative.
		 * \param      velocity        The default initial two-dimensional linear
		 *                             velocity of a new agent (optional).
		 */
		void setAgentDefaults(float neighborDist, int maxNeighbors,
							  float timeHorizon, float timeHorizonObst,
							  float radius, float maxSpeed,
							  const Vector2 &velocity = Vector2());

		/**
		 * \brief      Sets the maximum neighbor count of a specified agent.
		 * \param      agentNo         The number of the agent whose maximum
		 *                             neighbor count is to be modified.
		 * \param      maxNeighbors    The replacement maximum neighbor count.
		 */
		void setAgentMaxNeighbors(int agentNo, int maxNeighbors);

		/**
		 * \brief      Sets the maximum speed of a specified agent.
		 * \param      agentNo         The number of the agent whose maximum speed
		 *                             is to be modified.
		 * \param      maxSpeed        The replacement maximum speed. Must be
		 *                             non-negative.
		 */
		void setAgentMaxSpeed(int agentNo, float maxSpeed);

		/**
		 * \brief      Sets the maximum neighbor distance of a specified agent.
		 * \param      agentNo         The number of the agent whose maximum
		 *                             neighbor distance is to be modified.
		 * \param      neighborDist    The replacement maximum neighbor distance.
		 *                             Must be non-negative.
		 */
		void setAgentNeighborDist(int agentNo, float neighborDist);

		/**
		 * \brief      Sets the two-dimensional position of a specified agent.
		 * \param      agentNo         The number of the agent whose
		 *                             two-dimensional position is to be modified.
		 * \param      position        The replacement of the two-dimensional
		 *                             position.
		 */
		void setAgentPosition(int agentNo, const Vector2 &position);

		/**
		 * \brief      Sets the two-dimensional preferred velocity of a
		 *             specified agent.
		 * \param      agentNo         The number of the agent whose
		 *                             two-dimensional preferred velocity is to be
		 *                             modified.
		 * \param      prefVelocity    The replacement of the two-dimensional
		 *                             preferred velocity.
		 */
		void setAgentPrefVelocity(int agentNo, const Vector2 &prefVelocity);

		/**
		 * \brief      Sets the radius of a specified agent.
		 * \param      agentNo         The number of the agent whose radius is to
		 *                             be modified.
		 * \param      radius          The replacement radius.
		 *                             Must be non-negative.
		 */
		void setAgentRadius(int agentNo, float radius);

		/**
		 * \brief      Sets the time horizon of a specified agent with respect
		 *             to other agents.
		 * \param      agentNo         The number of the agent whose time horizon
		 *                             is to be modified.
		 * \param      timeHorizon     The replacement time horizon with respect
		 *                             to other agents. Must be positive.
		 */
		void setAgentTimeHorizon(int agentNo, float timeHorizon);

		/**
		 * \brief      Sets the time horizon of a specified agent with respect
		 *             to obstacles.
		 * \param      agentNo         The number of the agent whose time horizon
		 *                             with respect to obstacles is to be modified.
		 * \param      timeHorizonObst The replacement time horizon with respect to
		 *                             obstacles. Must be positive.
		 */
		void setAgentTimeHorizonObst(int agentNo, float timeHorizonObst);

		/**
		 * \brief      Sets the two-dimensional linear velocity of a specified
		 *             agent.
		 * \param      agentNo         The number of the agent whose
		 *                             two-dimensional linear velocity is to be
		 *                             modified.
		 * \param      velocity        The replacement two-dimensional linear
		 *                             velocity.
		 */
		void setAgentVelocity(int agentNo, const Vector2 &velocity);

		/**
		 * \brief      Sets the time step of the simulation.
		 * \param      timeStep        The time step of the simulation.
		 *                             Must be positive.
		 */
		void setTimeStep(float timeStep);

		int main(int argc, char **argv);

	};
//}
#endif /* ORCA_ORCA_SIMULATOR_H_ */

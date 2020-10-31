
#ifndef ORCA_KD_TREE_H_
#define ORCA_KD_TREE_H_

/**
 * \file       KdTree.h
 * \brief      Contains the KdTree class.
 */

#include "definitions.h"
#include "orca_simulator.h"
#include "agent.h"
#include "obstacle.h"
//namespace ORCA {

	//class ORCA_Simulator;
	//class Agent;
	//class Obstacle;

	/**
	 * \brief      Defines <i>k</i>d-trees for agents and static obstacles in the
	 *             simulation.
	 */
	class KdTree {
	private:
		/**
		 * \brief      Defines an agent <i>k</i>d-tree node.
		 */
		class AgentTreeNode {
		public:
			/**
			 * \brief      The beginning node number.
			 */
			int begin;

			/**
			 * \brief      The ending node number.
			 */
			int end;

			/**
			 * \brief      The left node number.
			 */
			int left;

			/**
			 * \brief      The maximum x-coordinate.
			 */
			float maxX;

			/**
			 * \brief      The maximum y-coordinate.
			 */
			float maxY;

			/**
			 * \brief      The minimum x-coordinate.
			 */
			float minX;

			/**
			 * \brief      The minimum y-coordinate.
			 */
			float minY;

			/**
			 * \brief      The right node number.
			 */
			int right;
		};

		/**
		 * \brief      Defines an obstacle <i>k</i>d-tree node.
		 */
		class ObstacleTreeNode {
		public:
			/**
			 * \brief      The left obstacle tree node.
			 */
			ObstacleTreeNode *left;

			/**
			 * \brief      The obstacle number.
			 */
			const Obstacle *obstacle;

			/**
			 * \brief      The right obstacle tree node.
			 */
			ObstacleTreeNode *right;
		};

		/**
		 * \brief      Constructs a <i>k</i>d-tree instance.
		 * \param      sim             The simulator instance.
		 */
		explicit KdTree(ORCA_Simulator *sim);

		/**
		 * \brief      Destroys this kd-tree instance.
		 */
		~KdTree();

		/**
		 * \brief      Builds an agent <i>k</i>d-tree.
		 */
		void buildAgentTree();

		void buildAgentTreeRecursive(int begin, int end, int node);

		/**
		 * \brief      Builds an obstacle <i>k</i>d-tree.
		 */
		void buildObstacleTree();

		ObstacleTreeNode *buildObstacleTreeRecursive(const std::vector<Obstacle *> &
													 obstacles);

		/**
		 * \brief      Computes the agent neighbors of the specified agent.
		 * \param      agent           A pointer to the agent for which agent
		 *                             neighbors are to be computed.
		 * \param      rangeSq         The squared range around the agent.
		 */
		void computeAgentNeighbors(Agent *agent, float &rangeSq) const;

		/**
		 * \brief      Computes the obstacle neighbors of the specified agent.
		 * \param      agent           A pointer to the agent for which obstacle
		 *                             neighbors are to be computed.
		 * \param      rangeSq         The squared range around the agent.
		 */
		void computeObstacleNeighbors(Agent *agent, float rangeSq) const;

		/**
		 * \brief      Deletes the specified obstacle tree node.
		 * \param      node            A pointer to the obstacle tree node to be
		 *                             deleted.
		 */
		void deleteObstacleTree(ObstacleTreeNode *node);

		void queryAgentTreeRecursive(Agent *agent, float &rangeSq,
									 int node) const;

		void queryObstacleTreeRecursive(Agent *agent, float rangeSq,
										const ObstacleTreeNode *node) const;

		/**
		 * \brief      Queries the visibility between two points within a
		 *             specified radius.
		 * \param      q1              The first point between which visibility is
		 *                             to be tested.
		 * \param      q2              The second point between which visibility is
		 *                             to be tested.
		 * \param      radius          The radius within which visibility is to be
		 *                             tested.
		 * \return     True if q1 and q2 are mutually visible within the radius;
		 *             false otherwise.
		 */
		bool queryVisibility(const Vector2 &q1, const Vector2 &q2,
							 float radius) const;

		bool queryVisibilityRecursive(const Vector2 &q1, const Vector2 &q2,
									  float radius,
									  const ObstacleTreeNode *node) const;

		std::vector<Agent *> agents_;
		std::vector<AgentTreeNode> agentTree_;
		ObstacleTreeNode *obstacleTree_;
		ORCA_Simulator *sim_;

		static const int MAX_LEAF_SIZE = 10;

		friend class Agent;
		friend class ORCA_Simulator;
	};
//}

#endif /* ORCA_KD_TREE_H_ */

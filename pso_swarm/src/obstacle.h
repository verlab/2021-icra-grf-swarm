
#ifndef ORCA_OBSTACLE_H_
#define ORCA_OBSTACLE_H_

/**
 * \file       Obstacle.h
 * \brief      Contains the Obstacle class.
 */

//#include "definitions.h"
#include "vector2.h"

//namespace ORCA {
	/**
	 * \brief      Defines static obstacles in the simulation.
	 */
	class Obstacle {
	private:
		/**
		 * \brief      Constructs a static obstacle instance.
		 */
		explicit Obstacle();

		bool isConvex_;
		Obstacle *nextObstacle_;
		Vector2 point_;
		Obstacle *prevObstacle_;
		Vector2 unitDir_;

		int id_;

		friend class Agent;
		friend class KdTree;
		friend class ORCA_Simulator;
	};
//}

#endif /* ORCA_OBSTACLE_H_ */

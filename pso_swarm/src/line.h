#ifndef ORCA_LINE_H_
#define ORCA_LINE_H_

#include "vector2.h"

//namespace ORCA{

 	//class Agent;
 	//class KdTree;
	//class Obstacle;


	class Line {
		public:
			/**
			 * \brief     A point on the directed line.
			 */
			Vector2 point;

			/**
			 * \brief     The direction of the directed line.
			 */
			Vector2 direction;

			friend class Agent;
			friend class KdTree;
			friend class Obstacle;
	};
//}
#endif /* ORCA_LINE_H_ */
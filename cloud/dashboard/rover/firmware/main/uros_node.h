#ifndef UROS_NODE_H
#define UROS_NODE_H

/* Starts the micro-ROS task. Never returns control of that task; the
 * control loop lives in its own task so that motor safety does not depend
 * on the middleware being alive. */
void uros_node_start(void);

#endif /* UROS_NODE_H */

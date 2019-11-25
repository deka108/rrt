# Lab 4 - Rapidly-exploring Random Tree (RRT)
Lab 4 for [COMSW4733 Computational Aspects of Robotics](https://www.cs.columbia.edu/~allen/F19/) at Columbia University (Instructor: [Prof. Peter Allen](http://www.cs.columbia.edu/~allen/)).

## Authors

| Name | UNI|
| - | - |
| Deka Auliya Akbar | da2897 |
| Madhavan Seshadri | ms5945 |
| Shravan Karthik | sk4653 |

----
### Prerequisites:

1. Installation of pybullet on the machine
2. demo.py needs to be working on the host machine

----
### Usage
1. To run part 1 which implements the RRT Algorithm, please use the following command:
`python demo.py`
1. To run part 2 which implements the Bidirectional RRT Algorithm, please use the following command:
`python demo.py --rrt`
1. To run part 3 which smoothenes the Bidirectional RRT Algorithm, please use the following command:
`python demo.py --birrt --smoothing`

----
### Methods in demo.py
1. Function `set_joint_positions` takes two parameters `joins` and `values` resets the join positions 
of the bot with the values parameter
1. Function `draw_sphere_marker` creates a sphere shaped marker taking position coordinates and radius
and colors it with the parameter as part of the request
1. Function `remove_marker` takes `marker_id` as a parameter and is used to remove the marker whose 
marker_id matches the parameter.
1. Function `find_nn` takes two parameters `V` and `q` and finds the nearest node q (configuration space) from the
 list of nodes passed in the parameter `V`. It achieves this by taking an L1 norm of the C space values
1. Function `progress_by_step_size` takes two parameters `q_src` and `q_dest` and takes a small step in the direction 
along the line connecting q_src and q_dest by a distance of delta.
1. Function `build_path` takes `stard_idx`, `goal_idx`, `V`, `E` as the input parameters where `V` and `E` give the 
graph representation through vertices and edges and returns a valid set of nodes as a path between q_start and q_goal if
the path can be found and returns `None` otherwise.
1. Function `find_path` takes `start_idx`, `goal_idx`, `adj_list` and tries to find a path based on the graph definition
 as provided by the adjacency list definition of the graph. A typical representation of this list is that, the value to 
 a dictionary entry will be all the set of nodes which are adjacent to the given node.
1. Function `extend_rrt` takes the graph definition as G(V,E), a random C Space configuration `q_rand` and color. It 
then extends the RRT algorithm by finding the nearest node on the tree for a given 
random point q_rand.
1. Function `rrt` is the core implementation of the RRT algorithm which works using the following three step approach:
    1. Select a random configuration q_rand (q_rand = q_goal with certain bias prob)
    1. Extend the RRT towards q_rand by q_new, a node step-size away from q_near in RRT.
    1. If q_new is within step_size away from q_goal, goal is found and build a path 
        from q_start to q_goal.
1. Function `check_connected` takes set of vertices `V1` and `V2` as input which represent the nodes in the different 
bidirectional RRTs and checks if the any node in the first is connected to any node in the second list by a small 
step_size interval. The RRTs are connected if are two points v1 in V1 and v2 in V2 which are at 
distance <= step_size away from each-other
1. Function `birrt` us the core implementation of the bidirectional RRT algorithm. It works using the following 
4 main steps:
    1. Grow two RRTs simultaneously one starting from q_start and another starting from q_goal.
    1. Grow RRT1 using q_new1 towards q_rand1 (similar to RRT algorithm).
    1. Grow RRT2 towards q_new1 of RRT1, check if the two Trees are connected:
        - If connected: stop the search because the solution is found, combine the two trees and 
            build a path.
        - If not connected, grow RRT2 towards q_rand2
    1. Swap the two RRTs after each iteration, and repeat step 1-4
1. Function `birrt_smoothing` implements the smoothing of the path found in the `birrt` method. 
It is implemented using the three main steps:
    1. Calls BiRRT to get a path from q_start to q_goal.
    1. Select two random nodes from the path.
    1. Pick two random nodes and try to interpolate between them with step-size. 
        Check if the interpolation is collision-free and have less number of step-size steps
        compared to the original path.
        - If True, remove the previous jagged path and add the smoothed new 
            between the two random nodes

#### Video Link

The run for these algorithms is available at https://www.youtube.com/playlist?list=PLMbCjZrjdlPNAQHShtGJahKy7xniH-x88



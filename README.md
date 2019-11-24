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
### Methods
1. Function `set_joint_positions` takes two parameters `joins` and `values` resets the join positions 
of the bot with the values parameter
1. 

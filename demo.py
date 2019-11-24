from __future__ import division
import pybullet as p
import pybullet_data
import numpy as np
import random
import time
import argparse


UR5_JOINT_INDICES = [0, 1, 2]


def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        p.resetJointState(body, joint, value)


def draw_sphere_marker(position, radius, color):
   vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
   marker_id = p.createMultiBody(basePosition=position, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vs_id)
   return marker_id


def remove_marker(marker_id):
   p.removeBody(marker_id)


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--birrt', action='store_true', default=False)
    parser.add_argument('--smoothing', action='store_true', default=False)
    args = parser.parse_args()
    return args

def find_nn(V, q):
    """Find the nearest node from q in V.
    
    Arguments:
        V {list of numpy arrays} -- The list of nodes in the RRT
        q {numpy array} -- a configuration
    
    Returns:
        numpy array -- q_near: the nearest node in V from q
        int -- The index of q_near in V
    """
    distances = np.linalg.norm(V - q, axis=1)
    nearest_idx = np.argmin(distances)

    return V[nearest_idx], nearest_idx

def progress_by_step_size(q_src, q_dest):
    """Progress from q_src to q_dest by step_size
    
    Arguments:
        q_src {numpy array} -- The start configuration
        q_dest {numpy array} -- The end configuration
    
    Returns:
        numpy array -- q_new: a node step_size away from q_src
            heading in the direction of q_dest
    """
    diff = q_dest - q_src
    norm = np.linalg.norm(diff)
    delta = (step_size / norm) * diff

    return q_src + delta

def build_path(start_idx, goal_idx, V, E):
    """Build the graph and find a path from start to goal index in T(V, E).
    
    Arguments:
        start_idx {int} -- index of q_start in V
        goal_idx {int} -- index of q_goal in V
        V {list of numpy arrays} -- The list of nodes in the RRT
        E {list of index pairs} -- The list of edges in the RRT
    
    Returns:
        {list of numpy arrays} -- The list of nodes that form a path 
            from q_start to q_goal in RRT. Return None if path between start 
            and goal index is not found. 
    """

    # build a graph
    adj_list = {
        k:[]        
        for k in range(len(V))
    }
    for edge in E:
        adj_list[edge[0]].append(edge[1])
        adj_list[edge[1]].append(edge[0])

    path_idx = find_path(start_idx, goal_idx, adj_list, 
        visited=set(), final_path=[])
    
    if path_idx is not None:
        return path_idx

def find_path(start_idx, goal_idx, adj_list, visited=set(), final_path=[]):
    """Find a path between start and goal based on adjacency list with DFS.
    
    Arguments:
        start_idx {int} -- index of q_start in V
        goal_idx {int} -- index of q_goal in V
        adj_list {dict[int]: list[int]} -- the adjacency list that map node index to
            its neighbouring node indexes
    
    Keyword Arguments:
        visited {set} -- Set of visited node indexes (default: {set()})
        final_path {list} -- The path from start node index to current node index 
            (default: {[]})
    
    Returns:
        list of int -- The list of node indexes that forms a path from q_start and 
            q_goal. Return None if path between start and goal index is not found.
    """
    visited.add(start_idx)
    final_path.append(start_idx)

    if start_idx == goal_idx:
        return final_path

    for n in adj_list[start_idx]:
        if n not in visited:
            res = find_path(n, goal_idx, adj_list, 
                visited, final_path)

            if res is not None:
                return res
    
    # backtrack try different node
    final_path.pop()
    visited.remove(start_idx)
    
def extend_rrt(V, E, q_rand, world_pos, color):
    """Extends the RRT by finding the nearest node on the tree for a given 
    random point q_rand.
    
    Arguments:
        V {list of numpy arrays} -- The list of nodes in the RRT
        E {list of index pairs} -- The list of edges in the RRT
        q_rand {numpy array} -- a random configuration
        world_pos {list of list of floats} -- The list of world positions
        color {list of floats} -- The RGB color for drawing the path [R, G, B]
    
    Returns:
        numpy array -- a new configuration collision-free between q_near and q_rand.
            Return None if the configuration is not collision-free.
    """
    q_near, q_near_idx = find_nn(V, q_rand)
    
    # avoid division by 0
    if np.linalg.norm(q_rand - q_near) == 0.0:
        return

    # move from q_near to q
    q_new = progress_by_step_size(q_near, q_rand)

    if not collision_fn(q_new):
        cur_world_pos = p.getLinkState(ur5, 3)[0]
        V.append(q_new)

        # insert index of node to E
        E.append((q_near_idx, len(V)-1))
        world_pos.append(cur_world_pos)
        
        p.addUserDebugLine(
            lineFromXYZ=world_pos[q_near_idx],
            lineToXYZ=cur_world_pos,
            lineColorRGB=color, 
            lineWidth = 0.5
        )

        return q_new

def rrt():
    """Implements a RRT algorithm. 
    
    1. Select a random configuration q_rand (q_rand = q_goal with certain bias prob)
    2. Extend the RRT towards q_rand by q_new, a node step-size away from q_near in RRT.
    3. If q_new is within step_size away from q_goal, goal is found and build a path 
        from q_start to q_goal.
    
    Expect:
        step_size [float]: step size for growing the tree
        bias [float]: the bias probability towards q_goal
        lower_bound [float]: the lower boundary value of a possible random configuration
        upper_bound [float]: the upper boundary value of a possible random configuration
        q_start [numpy array]: the start configuration
        q_goal [numpy array]: the goal configuration
        start_position [list of floats]: the start position of the world coordinate

    Returns:
        list of numpy array -- the path from q_start to q_goal if exist. 
            Return None if path is not found
    """
    V = [q_start] # insert q_start
    E = []
    world_pos = [start_position]

    ###############################################
    # TODO your code to implement the rrt algorithm
    ###############################################
    
    is_goal_found = False
    for idx in range(N):
        follow_goal = np.random.choice([False, True], p=[1-bias, bias])
        if follow_goal:
            q_rand = q_goal
        else:
            q_rand = np.random.uniform(lower_bound, upper_bound, 3)
        
        q_new = extend_rrt(V, E, q_rand, world_pos, [0, 1, 0])

        if q_new is not None and np.linalg.norm(q_goal - q_new) < step_size:
            print("goal is found!")
            is_goal_found = True
            break
    
    if is_goal_found:
        print("path building....")
        path_idx = build_path(0, len(V) - 1, V, E)
        path = [V[idx] for idx in path_idx]
        print("path building done!")
        
        return path

def check_connected(V1, V2):
    """Checks if nodes in RRT 1 and nodes in RRT are connected by a step size.
    The RRTs are connected if are two points v1 in V1 and v2 in V2 which are at 
        distance <= step_size away from each-other
    
    Arguments:
        V1 {list of numpy arrays} -- The list of nodes in the RRT1.
        V2 {list of numpy arrays} -- The list of nodes in the RRT2.
    
    Returns:
        int, int -- The node indexes of v1 in RRT1 and v2 in RRT2, which connects the trees together.
            Return None, None if the two trees aren't connected.
    """
    for idx1, v1 in enumerate(V1):
        for idx2, v2 in enumerate(V2):
            if np.linalg.norm(v1-v2) <= step_size:
                return idx1, idx2
    return None, None

def birrt():
    """Implements the bi-RRT algorithm.
    
    1. Grow two RRTs simultaneously one starting from q_start and another starting from q_goal.
    2. Grow RRT1 using q_new1 towards q_rand1 (similar to RRT algorithm).
    3. Grow RRT2 towards q_new1 of RRT1, check if the two Trees are connected:
        - If connected: stop the search because the solution is found, combine the two trees and 
            build a path.
        - If not connected, grow RRT2 towards q_rand2
    4. Swap the two RRTs after each iteration, and repeat step 1-4

    Expect:
        step_size [float]: step size for growing the tree
        bias [float]: the bias probability towards q_goal
        lower_bound [float]: the lower boundary value of a possible random configuration
        upper_bound [float]: the upper boundary value of a possible random configuration
        q_start [numpy array]: the start configuration
        q_goal [numpy array]: the goal configuration
        start_position [list of floats]: the start position of the world coordinate

    Returns:
        list of numpy array -- the path from q_start to q_goal if exist. 
            Return None if path is not found
    """
    #################################################
    # TODO your code to implement the birrt algorithm
    #################################################
    V1 = [q_start] # insert q_start
    E1 = []
    world_pos1 = [start_position]

    V2 = [q_goal] # insert q_goal
    E2 = []
    world_pos2 = [goal_position]

    is_t1 = True # t1 == tree starts from q_start
    is_goal_found = False

    for idx in range(N):
        follow_bias1 = np.random.choice([False, True], p=[1-bias, bias])
        if follow_bias1:
            if is_t1:
                q_rand1 = q_goal
            else:
                q_rand1 = q_start
        else:
            q_rand1 = np.random.uniform(lower_bound, upper_bound, 3)

        if is_t1:
            color1 = [0, 1, 0]
            color2 = [0, 0, 1]
        else:
            color1 = [0, 0, 1]
            color2 = [0, 1, 0]

        q_new1 = extend_rrt(V1, E1, q_rand1, world_pos1, color1)
        
        if q_new1 is not None:
            extend_rrt(V2, E2, q_new1, world_pos2, color2)
            idx1, idx2 = check_connected(V1, V2)
            if idx1 and idx2:
                print("two graphs are connected!")
                # combining two graphs

                # change index for second one
                len_first_one = len(V1)

                # add all nodes in V2 to V1
                for v in V2:
                    V1.append(v)

                # add all edges in V2 to V1
                for e in E2:
                    E1.append(
                        (e[0] + len_first_one, 
                        e[1] + len_first_one)
                    )

                # connect the two trees
                E1.append((idx1, len_first_one + idx2))

                is_goal_found = True
                break
            else:
                # generate q_rand2 for growing the second tree
                follow_bias2 = np.random.choice([False, True], p=[1-bias, bias])
                
                if follow_bias2:
                    if is_t1: # if first tree starts from q_start, q_rand2 of T2 should be going towards q_start
                        q_rand2 = q_start
                    else:
                        q_rand2 = q_goal
                else:
                    q_rand2 = np.random.uniform(lower_bound, upper_bound, 3)

                extend_rrt(V2, E2, q_rand2, world_pos2, color2)
                    
        # swap trees
        is_t1 = not is_t1
        V1, V2 = V2, V1
        E1, E2 = E2, E1
        world_pos1, world_pos2 = world_pos2, world_pos1
    
    if is_goal_found:
        print('path building...')
        start_idx = np.argwhere((V1 == q_start).all(axis=1))[0][0]
        goal_idx = np.argwhere((V1 == q_goal).all(axis=1))[0][0]
        
        path_idx = build_path(start_idx, goal_idx, V1, E1)
        path = [V1[idx] for idx in path_idx]
        print("path building done!")
        return path


def birrt_smoothing():
    """Implements BiRRT with smoothing.

    Expect:
        step_size [float]: step size for growing the tree
        bias [float]: the bias probability towards q_goal
        lower_bound [float]: the lower boundary value of a possible random configuration
        upper_bound [float]: the upper boundary value of a possible random configuration
        q_start [numpy array]: the start configuration
        q_goal [numpy array]: the goal configuration
        start_position [list of floats]: the start position of the world coordinate
        smooth_count: the smooth count

    For the smoothing:
    1. Calls BiRRT to get a path from q_start to q_goal.
    2. Select two random nodes from the path.
    3. Pick two random nodes and try to interpolate between them with step-size. 
        Check if the interpolation is collision-free and have less number of step-size steps
        compared to the original path.
        - If True, remove the previous jagged path and add the smoothed new 
            between the two random nodes
    
    Returns:
        list of numpy array -- the smoothed route between q_start and q_goal.
    """
    ################################################################
    # TODO your code to implement the birrt algorithm with smoothing
    ################################################################
    path = birrt()

    if path is not None:
        for i in range(smooth_count):
            # select two random points
            q1_idx, q2_idx = random.sample(range(len(path)), 2)
            num_of_skips = abs(q2_idx - q1_idx)
            
            if q1_idx > q2_idx:
                q1_idx, q2_idx = q2_idx, q1_idx
            
            q1 = path[q1_idx]
            q2 = path[q2_idx]

            # if no collision when moving from q1 to q2
            is_collision = False
            num_of_steps = 0

            q_start = q1
            new_path = []
            while True:
                # base case: collide or within step size
                q_new = progress_by_step_size(q_start, q2)
                num_of_steps += 1

                if collision_fn(q_new):
                    is_collision = True
                    break
                
                new_path.append(q_new)
                if np.linalg.norm(q_new - q2) <= step_size:
                    break
            
                q_start = q_new
            
            # snip the path between q1 and q2
            if not is_collision and num_of_skips > num_of_steps:
                j = 0
                N = len(path)

                # remove elements between q1_idx and q2_idx
                for i in range(N):
                    if i <= q1_idx or i >= q2_idx:
                        path[j] = path[i]
                        j += 1
                
                path = path[:j]
                
                # insert the new path between q1_idx and q2_idx
                for i in range(len(new_path)):
                    path.insert(q1_idx + 1 +i, new_path[i])
        
        return path                

if __name__ == "__main__":
    args = get_args()

    # set up simulator
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, True)
    p.resetDebugVisualizerCamera(cameraDistance=1.400, cameraYaw=58.000, cameraPitch=-42.200, cameraTargetPosition=(0.0, 0.0, 0.0))

    # load objects
    plane = p.loadURDF("plane.urdf")
    ur5 = p.loadURDF('assets/ur5/ur5.urdf', basePosition=[0, 0, 0.02], useFixedBase=True)
    obstacle1 = p.loadURDF('assets/block.urdf',
                           basePosition=[1/4, 0, 1/2],
                           useFixedBase=True)
    obstacle2 = p.loadURDF('assets/block.urdf',
                           basePosition=[2/4, 0, 2/3],
                           useFixedBase=True)
    obstacles = [plane, obstacle1, obstacle2]

    # start and goal
    start_conf = (-0.813358794499552, -0.37120422397572495, -0.754454729356351)
    start_position = (0.3998897969722748, -0.3993956744670868, 0.6173484325408936)
    goal_conf = (0.7527214782907734, -0.6521867735052328, -0.4949270744967443)
    goal_position = (0.35317009687423706, 0.35294029116630554, 0.7246701717376709)
    goal_marker = draw_sphere_marker(position=goal_position, radius=0.02, color=[1, 0, 0, 1])
    set_joint_positions(ur5, UR5_JOINT_INDICES, start_conf)    

    # define constants
    N = 1000
    smooth_count = 100
    step_size = 0.04
    bias = 0.05
    
    lower_bound = -1
    upper_bound = 1

    q_start = np.array(start_conf)
    q_goal = np.array(goal_conf)

    # place holder to save the solution path
    path_conf = None

    # get the collision checking function
    from collision_utils import get_collision_fn, get_joint_positions
    collision_fn = get_collision_fn(ur5, UR5_JOINT_INDICES, obstacles=obstacles,
                                       attachments=[], self_collisions=True,
                                       disabled_collisions=set())

    if args.birrt:
        if args.smoothing:
            # using birrt with smoothing
            path_conf = birrt_smoothing()
        else:
            # using birrt without smoothing
            path_conf = birrt()
    else:
        # using rrt
        path_conf = rrt()

    if path_conf is None:
        # pause here
        input("no collision-free path is found within the time budget, finish?")
    else:
        ###############################################
        # TODO your code to highlight the solution path
        ###############################################

        # execute the path
        draw_path = False
        while True:
            if not draw_path:
                for q in path_conf:
                    set_joint_positions(ur5, UR5_JOINT_INDICES, q)
                    cur_world_pos = p.getLinkState(ur5, 3)[0]
                    draw_sphere_marker(cur_world_pos, 0.02, 
                        [1, 0, 0, 1])

                draw_path = True
            
            for q in path_conf:
                set_joint_positions(ur5, UR5_JOINT_INDICES, q)
                time.sleep(0.3)

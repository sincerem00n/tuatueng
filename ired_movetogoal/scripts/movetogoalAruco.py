#!/usr/bin/python3
"""
http://wiki.ros.org/move_base
http://wiki.ros.org/actionlib
"""
import rospy
import tf.transformations
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import Pose, Point, PoseStamped, PoseWithCovarianceStamped, Quaternion
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import actionlib
import numpy as np
import math
from collections import deque
from std_srvs.srv import Empty
from std_srvs.srv import SetBool, SetBoolResponse
from ired_aruco.msg import MPSPosition

def clear_costmaps():
    rospy.wait_for_service('/move_base/clear_costmaps')
    try:
        clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        clear_costmaps_service()
        rospy.loginfo("Cleared costmaps successfully.")
    except rospy.ServiceException as e:
        rospy.logwarn(f"Failed to clear costmaps: {e}")

# Call this function before attempting to move to a new goal
clear_costmaps()

# Grid configuration (0: free, 1: obstacle, 2: target, 3: end move)
grid = [
    [0, 0],
    [0, 0],
    [0, 0],
    [0, 3],
]

# Define a dictionary to map cell names to grid coordinates
map_a = {
    'B4': (0, 0),
    'A4': (0, 1),
    'B3': (1, 0),
    'A3': (1, 1),
    'B2': (2, -0.2),
    'A2': (2, 1),
    'B1': (3, 0),
    'A1': (3, 1)
}

# Define rnage dictionary to map cell names 
range_dict = {
    'B4': ((1.6,3.5), (0.91,1.5)),
    # 'B4': ((1.7,3.5), (0.51,1.5)),
    'A4': ((1.6,3.5), (-0.5,0.90)),
    'B3': ((1,1.59), (0.91,1.5)),
    'A3': ((0.7,1.59), (-0.5,0.90)),
    'A2': ((-1,0.69), (-0.5,0.90)),
}

# Function to get target cells from user input, allowing for "all"
def get_target_cells_from_input():
    target_cells = []
    while True:
        target_cell = input("Enter the target cell (A-B),(1-4) or 'all' to visit all cells: ").upper()
        if target_cell == 'ALL':
            target_cells = list(map_a.values())
            #except B1 and A1
            target_cells.remove((3,0))
            target_cells.remove((3,1))
            break
        if target_cell not in map_a:
            print(f"Invalid cell name '{target_cell}'. Valid options are: {', '.join(map_a.keys())}")
        else:
            target_cells.append(map_a[target_cell])
    return target_cells


# Starting position for the robot in the grid
start_pos = (3, 1)

# Cell size in meters (considering the variation)
cell_size = 1.0 # approximately 1x1 meter grid cells

current_pose = None  # Global variable to store the current pose

def convert_grid_to_real_world(grid_pos):
    """Convert grid coordinates to real-world coordinates."""
    return -1 * grid_pos[0] * cell_size + 3, -1 * grid_pos[1] * cell_size + 1  # start from (0, 0) in the grid map converted to real-world coordinates of cartesian plane

def pose_callback(pose_with_covariance):
    """Callback to update current robot pose."""
    global current_pose
    pose = pose_with_covariance.pose.pose
    current_pose = (pose.position.x, pose.position.y, pose.orientation.z)  # You may want to use a full orientation quaternion for more accuracy

def move_base_status_callback(_):
    """Callback for move base status updates."""
    pass

def move_base_result_callback(_):
    """Callback for move base result updates."""
    pass

class moveBaseAction():
    def __init__(self):
        self.move_base_action = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_action.wait_for_server(rospy.Duration(5))

    def createGoal(self, x, y):
        quat = tf.transformations.quaternion_from_euler(0, 0, 0)  # Fixed orientation
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(x, y, 0.000), Quaternion(quat[0], quat[1], quat[2], quat[3]))

        return goal

    # Main program
    def moveToPoint(self, x, y):
        target_point = self.createGoal(x, y)
        self.moveToGoal(target_point)
        success = self.move_base_action.wait_for_result()
        state = self.move_base_action.get_state()
        
        print("Move to %f, %f ->" % (
            target_point.target_pose.pose.position.x,
            target_point.target_pose.pose.position.y
        ))
        
        if success and state == GoalStatus.SUCCEEDED:
            print("Complete")
            return True
        else:
            print("Fail")
            self.move_base_action.cancel_goal()
            return False


    def moveToGoal(self, goal):
        self.move_base_action.send_goal(goal)
        success = self.move_base_action.wait_for_result()
        state = self.move_base_action.get_state()
        print("Move to %f, %f ->" % (
            goal.target_pose.pose.position.x,
            goal.target_pose.pose.position.y
        ))
        if success and state == GoalStatus.SUCCEEDED:
            print(" Complete")
            return True
        else:
            print(" Fail")
            self.move_base_action.cancel_goal()
            return False
            
    
    # Function to change the orientation of the robot
    def changeOrientation(self, angle):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Convert the angle to a quaternion
        quat = tf.transformations.quaternion_from_euler(0, 0, angle)
        goal.target_pose.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        
        self.move_base_action.send_goal(goal)
        success = self.move_base_action.wait_for_result()
        state = self.move_base_action.get_state()
        
        if success and state == GoalStatus.SUCCEEDED:
            return True
        else:
            self.move_base_action.cancel_goal()
            return False
        
def orientation(p, q, r):
    """Return orientation of the triplet (p, q, r).
    - 0 if p, q and r are collinear
    - 1 if clockwise
    - -1 if counterclockwise
    """
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val == 0:
        # Handle collinear case by returning 0
        # Optionally sort or check distance to select the next point
        return 0
    return 1 if val > 0 else -1


def gift_wrapping_hull(points):
    """Gift Wrapping Algorithm (Jarvis March) to find the convex hull."""
    n = len(points)
    if n < 3:
        return points  # Convex hull is undefined for fewer than 3 points

    hull = []
    
    # Start with the leftmost point
    leftmost = np.argmin(points[:, 0])
    point_on_hull = leftmost
    
    while True:
        hull.append(points[point_on_hull])
        endpoint = (point_on_hull + 1) % n
        
        for j in range(n):
            # Check if 'j' is more counterclockwise than 'endpoint'
            if orientation(points[point_on_hull], points[j], points[endpoint]) == -1:
                endpoint = j
            elif orientation(points[point_on_hull], points[j], points[endpoint]) == 0:
                # If collinear, choose the farther one
                dist_endpoint = np.linalg.norm(points[point_on_hull] - points[endpoint])
                dist_j = np.linalg.norm(points[point_on_hull] - points[j])
                if dist_j > dist_endpoint:
                    endpoint = j
        
        point_on_hull = endpoint
        if point_on_hull == leftmost:
            break
    
    return np.array(hull)

def fixed_five_point_hull(points, fixed=True):
    """Get the convex hull points, optionally fixed to 5 points."""
    hull_points = gift_wrapping_hull(points)

    if fixed:
        # If hull has more than 5 points, reduce to 5 points
        if len(hull_points) > 5:
            selected_points = hull_points[::len(hull_points) // 5][:5]  # Select every nth point
        elif len(hull_points) < 5:
            # If fewer than 5, add points from the original set that aren’t in the hull
            extra_points = []
            for point in points:
                if len(hull_points) + len(extra_points) >= 5:
                    break
                if not any(np.array_equal(point, hp) for hp in hull_points):
                    extra_points.append(point)
            selected_points = np.vstack([hull_points, extra_points[:5 - len(hull_points)]])
        else:
            selected_points = hull_points  # Exactly 5 points
    else:
        selected_points = hull_points  # Do not fix the number of points

    # Sort selected points in counterclockwise order for consistent plotting
    center = selected_points.mean(axis=0)
    angles = np.arctan2(selected_points[:, 1] - center[1], selected_points[:, 0] - center[0])
    selected_points = selected_points[np.argsort(angles)]

    return selected_points

# # aruco_tf Service
# save_position_ = rospy.Service('/save_position_service', SetBool, savePosition)
# rospy.loginfo('Aruco save position service on /save_position_service [std_srvs/SetBool]')

# Service asking position
def ask_position(State):
    ask_position = rospy.ServiceProxy('/save_position_service', SetBool)
    ask_position(State)
    return 
# Define a global list to store data
mps_data = []

def MPS_pos_callback(data):
    global mps_data
    number_found = data.number_found
    ids = data.id
    x_values = list(data.x)  # Convert to list for mutability
    y_values = list(data.y)  # Convert to list for mutability
    theta_values = data.theta

    for i in range(number_found):
        mps_id = ids[i]
        
        # Check if the ID already exists in the list
        existing_entry = next((entry for entry in mps_data if entry['id'] == mps_id), None)
        
        if existing_entry is None:
            # Only store if the ID is unique
            mps_data.append({
                "id": mps_id,
                "x": x_values[i],
                "y": y_values[i],
                "theta": theta_values[i]
            })
            rospy.loginfo(f"New target found: ID {mps_id} at ({x_values[i]}, {y_values[i]})")
        else:
            # Update existing entry if the position is significantly different
            if (abs(existing_entry["x"] - x_values[i]) > 0.1 or
                    abs(existing_entry["y"] - y_values[i]) > 0.1):
                existing_entry["x"] = x_values[i]
                existing_entry["y"] = y_values[i]
                existing_entry["theta"] = theta_values[i]
                rospy.loginfo(f"Updated target ID {mps_id} to new position ({x_values[i]}, {y_values[i]})")

def timer(duration): 
    start_time = time.time()
    while time.time() - start_time < duration:
        ask_position(True)
        rospy.sleep(1)
    ask_position(False)
    print("Timer ended")
        

# Modified main function to attempt alternate paths
def main():
    rospy.init_node('move_to_goal', anonymous=True)
    rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_base/status', GoalStatusArray, move_base_status_callback)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_result_callback)
    rospy.Subscriber('/aruco_mps_explored', MPSPosition, MPS_pos_callback)

    clear_costmaps()
    mba = moveBaseAction()

    initial_pos = [(2.5, -0.2), (2, 1), (2, 1)]
    # Ask for the initial position
    duration = 10
    timer(duration)
    # Check if two targets have been found
    if len(mps_data) == 2:
        rospy.loginfo("All targets visited.")
        
        #check each mps_data ids of x and y values are in the range of the target cells
        for entry in mps_data:
            for key, value in range_dict.items():
                #compare the x and y values of the mps_data to the range_dict
                if (entry['x'] >= value[0][0] and entry['x'] <= value[0][1]) and (entry['y'] >= value[1][0] and entry['y'] <= value[1][1]):
                    rospy.loginfo(f"Current position of ID {entry['id']} is in cell {key}")
                    break

    #check dict if have 2 ids end the progra
    else:  
    # Initialize the move_base action interface
        for pos in initial_pos:
            if pos == (2, 1):
                mba.changeOrientation(-45)
                print("tilted")
            if mba.moveToPoint(*convert_grid_to_real_world(pos)):
                rospy.loginfo(f"Reached the initial position at {pos}")
                time.sleep(1)
                timer(duration)

                if len(mps_data) == 2:
                    rospy.loginfo("All targets visited.")
                    for entry in mps_data:
                        for key, value in range_dict.items():
                            #compare the x and y values of the mps_data to the range_dict
                            if (entry['x'] >= value[0][0] and entry['x'] <= value[0][1]) and (entry['y'] >= value[1][0] and entry['y'] <= value[1][1]):
                                rospy.loginfo(f"Current position of ID {entry['id']} is in cell {key}")
                                break
                    break                            
            else:
                rospy.logwarn("Failed to reach the initial position.")
            rospy.sleep(1)  # Allow some time between movements

    # End move to starting position
    end_move = [(3, 1)]
    for pos in end_move:
        if mba.moveToPoint(*convert_grid_to_real_world(pos)):
            rospy.loginfo(f"Reached the position at {pos}")
        else:
            rospy.logwarn("Failed to reach the end position.")
        rospy.sleep(1)  # Allow some time between movements

    rospy.loginfo("Reach to end position. Mission complete.")
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
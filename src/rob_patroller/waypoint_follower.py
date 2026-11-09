import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Point
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import yaml
import os
import math
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped
# Stanja
PATROLLING = 0          # Vozi po točkama
APPROACHING_OBJECT = 1  # Prekida vožnju i ide prema objektu
WAITING_AT_OBJECT = 2   # Čeka ispred objekta
CANCELING_PATROL = 3  # Otkaži patrolu (privremeno)

class WaypointFollower(Node):
    def __init__(self):
        super().__init__("waypoint_follower")

        
        self.safe_distance = 0.6 
        self.wait_time_at_object = 3.0

        self.nav = BasicNavigator()
        self.callback_group = ReentrantCallbackGroup()

        package_path = get_package_share_directory('rob_patroller')
        yaml_path = os.path.join(package_path, "config", "clicked_points.yaml")
        
        self.patrol_waypoints = self.load_waypoints(yaml_path)
        self.get_logger().info(f"Učitano {len(self.patrol_waypoints)} točaka za patrolu.")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.subscription = self.create_subscription(
            Point,
            '/yellow_object_point',
            self.object_detected_callback,
            qos,
            callback_group=self.callback_group
        )
        
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose',
            self.pose_callback,
            amcl_qos
        )

        # --- STATE MACHINE VARIJABLE ---
        self.state = PATROLLING
        # self.object_processed = False # Osigurač da ne reagiramo na isti objekt više puta
        self.resume_patrol_needed = False
        self.wait_start_time = None
        self.current_pose = None
        self.saved_approach_goal = None
        self.visited_objects = set()


        self.nav.waitUntilNav2Active()
        
        self.start_patrol()

        self.timer = self.create_timer(1, self.control_loop, callback_group=self.callback_group)

    def load_waypoints(self, yaml_path):
        try:
            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f)
                raw_points = data["points"]
        except Exception as e:
            self.get_logger().error(f"Greška pri učitavanju YAML-a: {e}")
            return []

        forward = raw_points
        backward = raw_points[-2:0:-1] 
        full_path = forward + backward

        waypoints = []
        for p in full_path:
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose.position.x = float(p[0])
            goal.pose.position.y = float(p[1])
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0 
            waypoints.append(goal)
        
        return waypoints

    def start_patrol(self):
        self.get_logger().info("Započinjem patrolu...")
        self.nav.followWaypoints(self.patrol_waypoints)
        self.state = PATROLLING

    def pose_callback(self, msg):
        self.current_pose = PoseStamped()
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose

    def object_detected_callback(self, msg: Point):
    # prvo provjeri uvjete
        if self.state != PATROLLING or self.current_pose is None:
            return

        # Provjeri je li objekt već posjećen
        obj_key = (round(msg.x, 0), round(msg.y, 0))
        if obj_key in self.visited_objects:
            self.get_logger().info(f"Objekt {obj_key} je već obrađen, ignoriram.")
            return
        self.visited_objects.add(obj_key)

        self.get_logger().info(
            f"Detektiran objekt na: ({msg.x:.2f}, {msg.y:.2f}), "
            f"robot na: ({self.current_pose.pose.position.x:.2f}, "
            f"{self.current_pose.pose.position.y:.2f})"
        )
        
        self.state = CANCELING_PATROL
        self.nav.cancelTask()
        # self.object_processed = True 
        
        # Safe Distance
        robot_x = self.current_pose.pose.position.x
        robot_y = self.current_pose.pose.position.y
        
        obj_x = msg.x
        obj_y = msg.y
        
        # Vektor od robota do objekta
        dx = obj_x - robot_x
        dy = obj_y - robot_y
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist == 0: 
            return 

        dir_x = dx / dist
        dir_y = dy / dist
        target_x = obj_x - (dir_x * self.safe_distance)
        target_y = obj_y - (dir_y * self.safe_distance)
        
        yaw = math.atan2(dy, dx)
        q = quaternion_from_euler(0, 0, yaw)
        
        # ciljna poza
        approach_goal = PoseStamped()
        approach_goal.header.frame_id = "map"
        approach_goal.header.stamp = self.get_clock().now().to_msg()
        approach_goal.pose.position.x = target_x
        approach_goal.pose.position.y = target_y
        approach_goal.pose.orientation.x = q[0]
        approach_goal.pose.orientation.y = q[1]
        approach_goal.pose.orientation.z = q[2]
        approach_goal.pose.orientation.w = q[3]
        self.saved_approach_goal = approach_goal
        
        # self.nav.goToPose(approach_goal)
        # self.state = APPROACHING_OBJECT

    def control_loop(self):
        """Glavna logika koja se vrti 10 puta u sekundi."""
        
        is_complete = self.nav.isTaskComplete()
        
        if self.state == PATROLLING:
            if is_complete:
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("Patrola završena. Ponavljam krug...")
                    self.start_patrol()
                elif result == TaskResult.CANCELED:
                    pass
                elif result == TaskResult.FAILED:
                    self.get_logger().error("Patrola nije uspjela! Pokušavam ponovno...")
                    self.start_patrol()

        elif self.state == APPROACHING_OBJECT:
            if is_complete:
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f"Stigao do objekta. Čekam {self.wait_time_at_object} sekundi...")
                    self.state = WAITING_AT_OBJECT
                    self.wait_start_time = self.get_clock().now()
                elif result == TaskResult.FAILED:
                    self.get_logger().warn("Nisam uspio doći do objekta. Nastavljam patrolu. Dinajs")
                    self.state = PATROLLING
                    self.start_patrol()

        elif self.state == WAITING_AT_OBJECT:
            elapsed = self.get_clock().now() - self.wait_start_time
            if elapsed.nanoseconds / 1e9 > self.wait_time_at_object:
                self.get_logger().info("Čekanje završeno. Nastavljam patrolu.")
                
                # self.object_processed = False 
                
                self.start_patrol()
        
        elif self.state == CANCELING_PATROL:
            if self.nav.isTaskComplete():
                self.get_logger().info(f"Idem na sigurnu točku: ({self.saved_approach_goal.pose.position.x:.2f}, {self.saved_approach_goal.pose.position.y:.2f})")
                self.nav.goToPose(self.saved_approach_goal)
                self.state = APPROACHING_OBJECT

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

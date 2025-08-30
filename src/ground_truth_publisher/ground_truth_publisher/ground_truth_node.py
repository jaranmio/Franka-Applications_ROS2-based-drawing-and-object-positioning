import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import TransformBroadcaster
from pxr import Usd, UsdGeom, Gf
import math

class USDPosePublisher(Node):
    def __init__(self, usd_file_path, prim_paths, reference_prim = '/World'):
        super().__init__('usd_pose_publisher')
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Set up prim paths
        self.usd_file_path = usd_file_path
        self.prim_paths = prim_paths
        self.reference_prim = reference_prim     
        
        # Timer to periodically extract and publish poses
        self.timer = self.create_timer(0.1, self.publish_poses)

    def publish_poses(self):
        # Open the USD file
        self.stage = Usd.Stage.Open(self.usd_file_path)
        if not self.stage:
            self.get_logger().error(f"Failed to open USD file: {self.usd_file_path}")
            return
        
        for prim_path in self.prim_paths:
            prim = self.stage.GetPrimAtPath(prim_path)

            if prim:
                # Check if the prim is of type Xform (transformable object)
                if prim.IsA(UsdGeom.Xform):

                    reference_position, reference_orientation = self.get_reference_pose()
                    position_raw, orientation_raw = self.get_prim_pose(prim_path)
                    relative_translation = position_raw - reference_position
                    relative_rotation = orientation_raw * reference_orientation.GetInverse()
                    self.get_logger().info(f"Prim {prim_path} is at position {relative_translation} and orientation {relative_rotation} relative to {self.reference_prim}")

                    # Create TransformStamped message
                    transform_msg = TransformStamped()
                    transform_msg.header.stamp = self.get_clock().now().to_msg()
                    transform_msg.header.frame_id = 'fr3_link0'  # Set base frame
                    name = prim_path.split('/')[2]
                    transform_msg.child_frame_id = f'target_{name}_frame'  # Child frame is the prim

                    transform_msg.transform.translation.x = relative_translation[0]
                    transform_msg.transform.translation.y = relative_translation[1]
                    transform_msg.transform.translation.z = relative_translation[2]

                    transform_msg.transform.rotation.x = relative_rotation.GetImaginary()[0]
                    transform_msg.transform.rotation.y = relative_rotation.GetImaginary()[1]
                    transform_msg.transform.rotation.z = relative_rotation.GetImaginary()[2]
                    transform_msg.transform.rotation.w = relative_rotation.GetReal()

                    # Publish the transform
                    self.tf_broadcaster.sendTransform(transform_msg)
                else:
                    self.get_logger().error(f"Prim at {prim_path} is not transformable (Xform).")
            else:
                self.get_logger().error(f"Prim at {prim_path} not found.")

    def get_reference_pose(self):
        # Retrieve the base link transform for fr3_link0
        ref_position, ref_orientation = self.get_prim_pose(self.reference_prim)
        self.get_logger().info(f"Reference {self.reference_prim} is at position {ref_position}, orientation {ref_orientation}")
        return ref_position, ref_orientation
    
    # returns 2-array with position and orientation
    def get_prim_pose(self, prim_path):
        prim = self.stage.GetPrimAtPath(prim_path)

        xformable = UsdGeom.Xformable(prim)
        world_transform_matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        position = world_transform_matrix.ExtractTranslation()

        rotation = world_transform_matrix.ExtractRotation()
        orientation = Gf.Quatd(rotation.GetQuaternion().GetReal(), rotation.GetQuaternion().GetImaginary()) 
        
        return position, orientation.GetNormalized()


def main(args=None):
    rclpy.init(args=args)

    try:
        # Define the USD file path and prim paths to track
        usd_file_path = '/home/qpaig/AI-Robotic project/fr3/fr3_experiments_4f_sim.usd'
        prim_paths = ['/World/LASER/Group/PH3E_Step/tn__PH3EStep_bC', '/World/CAMERA/Group/PH3E_Step/tn__PH3EStep_bC']
        reference_prim = '/World/fr3'

        node = USDPosePublisher(usd_file_path, prim_paths, reference_prim)
        
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        # Catch other exceptions, log them, and gracefully shut down
        print(f"Error occurred: {e}")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

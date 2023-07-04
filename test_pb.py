import os
import math
import numpy as np
import random
import pybullet_planning as pp
from compas.robots import RobotModel
from compas_fab.robots import Robot

def get_bar_grasp_gen_fn(bar_length, tool_pose=pp.unit_pose(), reverse_grasp=False, safety_margin_length=0.0):
    """[summary]

    # converted from https://pybullet-planning.readthedocs.io/en/latest/reference/generated/pybullet_planning.primitives.grasp_gen.get_side_cylinder_grasps.html
    # to get rid of the rotation around the local z axis

    """

    # rotate the cylinder's frame to make x axis align with the longitude axis
    longitude_x = pp.Pose(euler=pp.Euler(pitch=np.pi/2))
    def gen_fn():
        while True:
            # translation along the longitude axis
            slide_dist = random.uniform(-bar_length/2+safety_margin_length, bar_length/2-safety_margin_length)
            translate_along_x_axis = pp.Pose(point=pp.Point(slide_dist,0,0))

            for j in range(1 + reverse_grasp):
                # the base pi/2 is to make y align with the longitude axis, conforming to the convention (see image in the doc)
                # flip the gripper, gripper symmetry
                rotate_around_z = pp.Pose(euler=[0, 0, math.pi/2 + j * math.pi])

                object_from_gripper = pp.multiply(longitude_x, translate_along_x_axis, \
                    rotate_around_z, tool_pose)
                yield pp.invert(object_from_gripper)
    return gen_fn


def main():
    pp.connect(use_gui=True)
    
    HERE = os.path.dirname(__file__)
    # robot_urdf = os.path.join(HERE,'ur_description/urdf/ur5.urdf')
    robot_urdf = os.path.join(HERE,'mt_husky_moveit_config/urdf/husky_ur5_e.urdf')
    gripper_obj = os.path.join(HERE,'robotiq_85/meshes/static/robotiq_85_close_20mm.obj')
    # robot_urdf = os.path.join(HERE,'robotiq_85/urdf/robotiq_85_gripper_simple.urdf')
    # robot_urdf = os.path.join(HERE,'mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e.urdf')
    # print(robot_urdf)
    assert os.path.exists(robot_urdf)
    assert os.path.exists(gripper_obj)
    
    robot = pp.load_pybullet(robot_urdf, fixed_base=False, cylinder=True)
    pp.clone_body(robot, collision=True, visual=False)
    pp.camera_focus_on_body(robot)

    # pp.dump_body(robot)
    base_bb = pp.create_box(0.9864, 0.6851, 0.3767)
    pp.set_pose(base_bb, pp.Pose(point=[0,0,0.18835]))

    joints = pp.get_movable_joints(robot)
    for j in joints:
        child_link = pp.child_link_from_joint(j)
        # print('Joint {} - Child {}'.format(pp.get_joint_name(robot, j), pp.get_link_name(robot, child_link)))
        link_pose = pp.get_link_pose(robot, child_link)
        pp.draw_pose(link_pose)

    tool0_pose = pp.get_link_pose(robot, pp.link_from_name(robot, 'ur_arm_tool0'))
    pp.draw_pose(tool0_pose)

    ee = pp.create_obj(gripper_obj)
    pp.set_pose(ee, tool0_pose)

    tool0_from_ee = pp.Pose(euler=pp.Euler(yaw=-np.pi/2), point=[0,0,0.138])
    tcp_pose = pp.multiply(tool0_pose, tool0_from_ee)
    # tcp_pose = pp.get_link_pose(robot, pp.link_from_name(robot, 'central_tcp'))
    pp.draw_pose(tcp_pose)

    bar_length = 0.5
    bar_body = pp.create_cylinder(0.01, bar_length, mass=pp.STATIC_MASS)
    grasp_gen = get_bar_grasp_gen_fn(bar_length)

    for _ in range(10):
        gripper_from_object = next(grasp_gen())
        world_from_object = pp.multiply(tcp_pose, gripper_from_object)
        pp.set_pose(bar_body, world_from_object)
        pp.wait_if_gui() 

    pp.wait_if_gui()

if __name__ == '__main__':
    main()
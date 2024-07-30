import numpy
from dm_control import mjcf
from dm_control.mjcf import attribute
import mujoco
import mujoco.viewer as viewer
import numpy as np
from scipy.spatial.transform import Rotation
import re

def general_fixes(hand_mjcf, **config):
    geoms = hand_mjcf.find_all("geom")
    num_geoms = len(geoms)
    print("Number of geoms:", num_geoms)
    for duplicate_geom in [geom for geom in geoms if geom.group is None]:
        duplicate_geom.remove(True)


    # bring contype and conaffinity to default values
    geoms = hand_mjcf.find_all("geom")
    num_geoms_1 = len(geoms)
    print("Number of geoms_new:", num_geoms_1)
    for geom in geoms:
        geom.contype = 1
        geom.conaffinity = 2

    return hand_mjcf


# def remove_inertials(hand_mjcf, default_density=1000, **config):

#     for inertial in [body.inertial for body in hand_mjcf.find_all('body')
#                      if body.inertial is not None and len(body.geom) > 0]:
#         inertial.remove()
#     hand_mjcf.default.geom.density = default_density
#     hand_mjcf.compiler.exactmeshinertia = True
#     return hand_mjcf

def constraint_sole_rolling(hand_mjcf, positive_limit=np.pi / 2 / 2, negative_limit=-np.pi / 10 / 2, **config):
    # region Get joints in converted MJCF file
    joints = hand_mjcf.find_all("joint")
    for joint in joints:
        del joint.range
    # endregion

    # region Define defaults to be applied to all unspecified fields
    # Add reasonable limits to joint rotations.
    # Could be fine-tuned, but the tendon length will enforce this partially anyway.
    
    hand_mjcf.default.joint.range = [negative_limit, positive_limit]
    hand_mjcf.default.joint.limited = True
    # endregion

    # constraints for non-sole joints
    # free_joints = ["posterior_arch_to_frontal_arch", "posterior_arch_to_heel"]
    # for joint in [joint for joint in joints if (joint.name in free_joints)]:
    #     joint.range = [-np.pi/2, np.pi/2]

    # for joint in [joint for joint in joints if ("constraint" in joint.name.split("_"))]:
    #     joint.range = [-np.pi/2, np.pi/6]

    # region Helper functions for pairing up joints with their virtual counterparts
    def joint_name_split(name):
        return name.split("_virtual")

    # endregion

    # region negative angle range for the reverse-rollover joint
    # for joint in [joint for joint in joints
    #               if ("reverse" in joint_name_split(joint.name)[0] or "virtual_7" in joint_name_split(joint.name)[0])]:
    #     joint.range = [-positive_limit, -negative_limit]
    # endregion

    # region Define equality constraints to generate rolling motion without needing contacts
    for joint in [joint for joint in joints if "virtual" in joint.name]:
        virtual_joint = joint
        real_joint = virtual_joint.name.split("_virtual")[0] + virtual_joint.name.split("_virtual")[1] 
        # print("Name:", virtual_joint.name)
        if virtual_joint is not None:
            hand_mjcf.equality.add("joint",
                                   name="_eq_".join([real_joint, virtual_joint.name]),
                                   joint1=real_joint,
                                   joint2=virtual_joint)
    # endregion
    return hand_mjcf


def add_joint_stiffness_sole(hand_mjcf,
                             joint_stiffness=0.2, joint_damping=0.07, joint_armature=0.001,
                             **config):
    # region default joint values
    hand_mjcf.default.joint.stiffness = joint_stiffness
    hand_mjcf.default.joint.damping = joint_damping
    hand_mjcf.default.joint.armature = joint_armature
#     # endregion

# ----------------------------------------------------------------

#     # region special joint parameters settings
#     foot_mjcf.find("joint", "cube_to_frontal_arch").stiffness = frontal_stiffness
#     foot_mjcf.find("joint", "cube_to_frontal_arch").damping = frontal_damping
#     foot_mjcf.find("joint", "cube_to_frontal_arch").armature = frontal_armature
#     # foot_mjcf.find("joint", "cube_to_posterior_arch").stiffness = posterior_stiffness
#     # foot_mjcf.find("joint", "cube_to_posterior_arch").damping = posterior_damping
#     foot_mjcf.find("joint", "posterior_arch_to_heel").stiffness = heel_stiffness
#     foot_mjcf.find("joint", "posterior_arch_to_heel").damping = heel_damping
#     foot_mjcf.find("joint", "posterior_arch_to_heel").armature = heel_armature
#     # endregion

    return hand_mjcf


# def add_sole_colliders(foot_mjcf, **config):
#     geoms = foot_mjcf.find_all("geom")
#     contype = 4
#     conaffinity = 1

#     for geom in [geom for geom in geoms if geom.mesh.name == "phalanx_softfoot_size_m"]:
#         body = geom.parent
#         body.add("geom", type="box", size=[0.012, 0.0095, 0.0055],
#                  contype=contype, conaffinity=conaffinity, pos=[0.005, 0, 0], group=4, rgba=[0.8, 0.1, 0.1, 0.3])

#     for geom in [geom for geom in geoms if geom.mesh.name == "phalanx_softfoot_size_reverse"]:
#         body = geom.parent
#         body.add("geom", type="box", size=[0.012, 0.009, 0.0055], euler=[0, 0, np.deg2rad(7)],
#                  contype=contype, conaffinity=conaffinity, pos=[0.005, 0, 0], group=4, rgba=[0.8, 0.1, 0.1, 0.3])

#     for geom in [geom for geom in geoms if geom.mesh.name == "phalanx_softfoot_roll"]:
#         body = geom.parent
#         body.add("geom", type="box", size=[0.012, 0.0082, 0.0055], euler=[0, 0, -np.deg2rad(8)],
#                  contype=contype, conaffinity=conaffinity, pos=[0.005, 0, 0], group=4, rgba=[0.8, 0.1, 0.1, 0.3])

#     for geom in [geom for geom in geoms if geom.mesh.name == "phalanx_softfoot_distal"]:
#         body = geom.parent
#         body.add("geom", type="box", size=[0.012, 0.0095, 0.0055],
#                  contype=contype, conaffinity=conaffinity, pos=[0.004, 0, 0], group=4, rgba=[0.8, 0.1, 0.1, 0.3])

#     foot_mjcf.find("body", "heel").add("geom", type="box", size=[0.032, 0.015, 0.037], euler=[0, 0, np.deg2rad(9)],
#                                        contype=contype, conaffinity=conaffinity, pos=[-0.005, 0, 0], group=4,
#                                        rgba=[0.8, 0.1, 0.1, 0.3])
#     return foot_mjcf


# def add_sole_tendon(foot_mjcf, n_fingers=1,
#                     tendon_range=(0, 0.289), tendon_length=[0.1, 0.3], tendon_stiffness=10000000,
#                     **config):

#     bodies = foot_mjcf.find_all("body")
#     excluded_bodies = ["frontal_arch", "cube", "posterior_arch",
#                        "constraint_phalanx_1", "constraint_phalanx_2", "constraint_phalanx_3", "constraint_phalanx_4",
#                        "constraint_phalanx_5"]

#     different_pins_bodies = ["reverse", "rollover", "virtual_7", "virtual_8"]
#     n_body = 18

#     contype = 4
#     conaffinity = 4

#     # materials for the additional geometries and sites
#     foot_mjcf.asset.add("material", name="yellow tendon site", rgba=[0.9, 0.7, 0.1, 0.3], emission=0.1)
#     foot_mjcf.asset.add("material", name="green pulleys", rgba=[0.1, 0.9, 0.3, 0.4], emission=0.8)

#     # region Add cylinder geoms to as pulleys for the tendon and sites for the wrapping direction
#     for body in [body for body in bodies if body.name not in excluded_bodies]:
#         if ("distal_phalanx" in body.name):
#             body.add("geom", type="cylinder", size=[0.0035, 0.006], name="roll_" + body.name,
#                      contype=contype, conaffinity=conaffinity, pos=[0, 0, 0], group=4, material="green pulleys")
#             body.add("site", name="site_roll_" + body.name, pos=[0, 0.0038, 0], size=[.001, .001, .001], material="yellow tendon site")
#             body.add("site", name="site_roll_" + body.name + "_extreme", pos=[0.019, 0.0038, 0],
#                      size=[.001, .001, .001], material="yellow tendon site")

#         elif ("heel" in body.name):
#             for i in range(1, n_fingers + 1):
#                 posz = 0.037 - 0.0185*(i-1)
#                 body.add("geom", type="cylinder", size=[0.0025, 0.006], name="roll_" + body.name + "_" + str(i),
#                          contype=contype, conaffinity=conaffinity, pos=[0.0008, 0.0335, posz], group=4, rgba=[0.8, 0.5, 0.15, 1])
#                 body.add("site", name="site_roll_" + body.name + "_" + str(i), pos=[0.0008, 0.038, posz], size=[.001, .001, .001], material="yellow tendon site")
#                 body.add("site", name="site_roll_" + body.name + "_" + str(i) + "_extreme", pos=[-0.007, 0.012, posz],
#                          size=[.001, .001, .001], material="yellow tendon site")

#         # elif (different_pins_bodies[1] not in body.name and different_pins_bodies[2] not in body.name):
#         #     body.add("geom", type="cylinder", size=[0.0035, 0.006], name="roll_" + body.name,
#         #              contype=contype, conaffinity=conaffinity, pos=[0, 0, 0], group=4, rgba=[0.1, 0.8, 0.1, 1])
#         #     body.add("site", name="site_roll_" + body.name, pos=[0, 0.0038, 0], size=[.001, .001, .001])
#         else:
#             body.add("geom", type="cylinder", size=[0.0035, 0.006], name="roll_" + body.name,
#                      contype=contype, conaffinity=conaffinity, pos=[0, 0, 0], group=4, material="green pulleys")
#             body.add("site", name="site_roll_" + body.name, pos=[0, 0.0038, 0], size=[.001, .001, .001], material="yellow tendon site")

#     # endregion

#     # region Insert the tendon element going through sites and rolling the cylinders. A tendon for each finger
#     # is defined by describing the routing through the sites on the pulleys and the pulleys to wrap on.

#     geoms = foot_mjcf.find_all("geom")

#     for i in range(1, n_fingers + 1):
#         tendon = foot_mjcf.tendon.add("spatial", name="tendon_" + str(i),
#                                       width=0.0004, rgba=[0.9, 0.7, 0.1, 1],
#                                       limited="true", range=tendon_range, stiffness=tendon_stiffness)


#         # modify the tendon element so that it can accept a list (bug fix) so that it can define a deadband
#         tendon._attributes["springlength"] = attribute.Array(name="springlength",
#                                                              required=False,
#                                                              parent=tendon,
#                                                              value=np.array(tendon_length),
#                                                              conflict_allowed=False,
#                                                              conflict_behavior='replace',
#                                                              length=np.array(tendon_length).size,
#                                                              dtype=float)


#         count_v = 0
#         count_r = 0

#         # Routing definition
#         #beginning at the heel
#         tendon.add("site", site="site_roll_heel_" + str(i) + "_extreme")
#         tendon.add("geom", geom="roll_heel_" + str(i), sidesite="site_roll_heel_" + str(i))

#         for j in range(1, n_body):
#             # even number = real link
#             if (j % 2) == 0:
#                 count_r += 1
#                 if j == 12:
#                     for geom in [geom for geom in geoms if
#                                  geom.name is not None and geom.name == "roll_reverse_phalanx_" + str(i)]:
#                         tendon.add("site", site="site_" + geom.name)
#                         tendon.add("geom", geom=geom.name, sidesite="site_" + geom.name)

#                 elif j == 14:
#                     for geom in [geom for geom in geoms if
#                                  geom.name is not None and geom.name == "roll_rollover_phalanx_" + str(i)]:
#                         tendon.add("site", site="site_" + geom.name)
#                         tendon.add("geom", geom=geom.name, sidesite="site_" + geom.name)

#                 elif j == 16:
#                     for geom in [geom for geom in geoms if
#                                  geom.name is not None and geom.name == "roll_phalanx_6_" + str(i)]:
#                         tendon.add("site", site="site_" + geom.name)
#                         tendon.add("geom", geom=geom.name, sidesite="site_" + geom.name)

#                 else:
#                     for geom in [geom for geom in geoms if
#                                  geom.name is not None and (
#                                          geom.name == ("roll_phalanx_" + str(count_r) + "_" + str(i)))]:
#                         tendon.add("site", site="site_" + geom.name)
#                         tendon.add("geom", geom=geom.name, sidesite="site_" + geom.name)


#             # odd number = virtual link
#             else:
#                 count_v += 1
#                 for geom in [geom for geom in geoms if
#                              geom.name is not None and (geom.name == ("roll_virtual_" + str(count_v) + "_" + str(i)))]:
#                     tendon.add("site", site="site_" + geom.name)
#                     tendon.add("geom", geom=geom.name, sidesite="site_" + geom.name)

#         tendon.add("site", site="site_roll_distal_phalanx_" + str(i))
#         tendon.add("geom", geom="roll_distal_phalanx_" + str(i), sidesite="site_roll_distal_phalanx_" + str(i))
#         tendon.add("site", site="site_roll_distal_phalanx_" + str(i) + "_extreme")

#     # endregion

#     return foot_mjcf


# def add_back_springs(foot_mjcf, spring_stiffness=50, spring_length=0.027, spring_range=(0.026, 0.040), **config):
#     frontal_arch = foot_mjcf.find("body", "frontal_arch")
#     heel = foot_mjcf.find("body", "heel")

#     # region Definition of the sites for the springs extremities
#     site_r = foot_mjcf.worldbody.add("site", name="site_posterior_spring_r", pos=[-0.014, -0.0456, 0.004],
#                                      size=[.001, .001, .001])
#     site_l = foot_mjcf.worldbody.add("site", name="site_posterior_spring_l", pos=[-0.014, -0.0456, -0.011],
#                                      size=[.001, .001, .001])

#     # The sites are transformed in relative coordinates
#     frontal_arch_quat = frontal_arch.quat
#     frontal_arch_rotation = Rotation.from_quat([*frontal_arch_quat[1:4], frontal_arch_quat[0]])

#     site_r.pos = frontal_arch_rotation.apply(site_r.pos)
#     site_l.pos = frontal_arch_rotation.apply(site_l.pos)

#     site_r.pos += frontal_arch.pos
#     site_l.pos += frontal_arch.pos

#     heel.add("site", name="site_heel_spring_r", pos=[0.026, 0.0125, 0.0075], size=[.001, .001, .001],
#              rgba=[0.9, 0.1, 0.1, .5])
#     heel.add("site", name="site_heel_spring_l", pos=[0.026, 0.0125, -0.0075], size=[.001, .001, .001],
#              rgba=[0.9, 0.1, 0.1, .5])
#     # endregion

#     # region tendons definition
#     spring_r = foot_mjcf.tendon.add("spatial", name="back_spring_r", limited="true", range=spring_range,
#                                     springlength=spring_length,
#                                     width=0.0015, stiffness=spring_stiffness, rgba=[0.3, 0.4, 0.45, 0.5])
#     spring_l = foot_mjcf.tendon.add("spatial", name="back_spring_l", limited="true", range=spring_range,
#                                     springlength=spring_length,
#                                     width=0.0015, stiffness=spring_stiffness, rgba=[0.3, 0.35, 0.4, 0.6])

#     spring_r.add("site", site="site_posterior_spring_r")
#     spring_r.add("site", site="site_heel_spring_r")

#     spring_l.add("site", site="site_posterior_spring_l")
#     spring_l.add("site", site="site_heel_spring_l")
#     # endregion

#     return foot_mjcf


# def add_elastic(foot_mjcf, n_fingers=1,
#                 elastic_length=0.01, elastic_range=(0.013, 0.018), elastic_stiffness=589, elastic_damping=0.1,
#                 **config):
#     bodies = foot_mjcf.find_all("body")
#     excluded_bodies = ["frontal_arch", "cube", "posterior_arch",
#                        "constraint_phalanx_1", "constraint_phalanx_2", "constraint_phalanx_3", "constraint_phalanx_4",
#                        "constraint_phalanx_5"]

#     different_bodies = ["reverse", "rollover"]
#     n_body = 18
#     sides = ["l", "r"]

#     contype = 4
#     conaffinity = 4

#     foot_mjcf.asset.add("material", name="blue elastic site", rgba=[0.1, 0.5, 0.9, 0.3], emission=0.1)

#     v_count = 1

#     # region define sites for the elastics
#     # the sites are defined in the real link frame, so that they are related to the real bodies and
#     # don't slide with the movement of virtual joints
#     for body in [body for body in bodies
#                  if (body.name not in excluded_bodies and body.name.split('_')[0] != "virtual")]:
#         # define the first row of sites (virtual_1) depending on the heel
#         if body.name == "heel":
#             for line in range(n_fingers):
#                 rot_quat = [0.82, 0, 0, -0.57]
#                 pos_f1 = [0.013, 0, 0.037-0.0185*line]
#                 delta_l = Rotation.from_quat([*rot_quat[1:4], rot_quat[0]]).apply([0, -0.004, 0.00625])
#                 delta_r = Rotation.from_quat([*rot_quat[1:4], rot_quat[0]]).apply([0, -0.004, -0.00625])
#                 pos_el_l = pos_f1+delta_l
#                 pos_el_r = pos_f1+delta_r

#                 body.add("site", name=f"site_elastic_virtual_1_{line+1}_l", pos=pos_el_l, quat=rot_quat, size=[.001, .001, .001], material="blue elastic site")
#                 body.add("site", name=f"site_elastic_virtual_1_{line+1}_r", pos=pos_el_r, quat=rot_quat, size=[.001, .001, .001], material="blue elastic site")

#         # for all the other sites in the sole
#         elif (different_bodies[0] not in body.name and different_bodies[1] not in body.name):
#             v_count += 1

#             # sites attached to the body frame
#             body.add("site", name="site_elastic_" + body.name + "_l", pos=[0, -0.004, 0.00625], size=[.001, .001, .001], material="blue elastic site")
#             body.add("site", name="site_elastic_" + body.name + "_r", pos=[0, -0.004, -0.00625],
#                      size=[.001, .001, .001], material="blue elastic site")
#             # sites related to the following point of the real link
#             body.add("site", name=f"site_elastic_virtual_{v_count}_{body.name.split('_')[-1]}_l",
#                      pos=[0.009, -0.004, 0.00625], size=[.001, .001, .001], material="blue elastic site")
#             body.add("site", name=f"site_elastic_virtual_{v_count}_{body.name.split('_')[-1]}_r",
#                      pos=[0.009, -0.004, -0.00625], size=[.001, .001, .001], material="blue elastic site")

#             if v_count > 9:
#                 v_count = 1

#         else:
#             v_count += 1

#             # reverse
#             if different_bodies[0] in body.name:
#                 # sites attached to the body frame (up)
#                 body.add("site", name="site_elastic_" + body.name + "_l", pos=[0, -0.004, 0.00625], size=[.001, .001, .001], material="blue elastic site")
#                 body.add("site", name="site_elastic_" + body.name + "_r", pos=[0, -0.004, -0.00625],
#                          size=[.001, .001, .001], material="blue elastic site")
#                 # sites related to the following point of the real link (down)
#                 body.add("site", name=f"site_elastic_virtual_{v_count}_{body.name.split('_')[-1]}_l",
#                          pos=[0.0103, 0.004, 0.00625], size=[.001, .001, .001], material="blue elastic site")
#                 body.add("site", name=f"site_elastic_virtual_{v_count}_{body.name.split('_')[-1]}_r",
#                          pos=[0.0103, 0.004, -0.00625], size=[.001, .001, .001], material="blue elastic site")
#                 # rollover
#             if different_bodies[1] in body.name:
#                 # sites attached to the body frame (down)
#                 body.add("site", name="site_elastic_" + body.name + "_l", pos=[0, 0.004, 0.00625], size=[.001, .001, .001], material="blue elastic site")
#                 body.add("site", name="site_elastic_" + body.name + "_r", pos=[0, 0.004, -0.00625],
#                          size=[.001, .001, .001], material="blue elastic site")
#                 # sites related to the following point of the real link (up)
#                 body.add("site", name=f"site_elastic_virtual_{v_count}_{body.name.split('_')[-1]}_l",
#                          pos=[0.019, -0.004, 0.00625], size=[.001, .001, .001], material="blue elastic site")
#                 body.add("site", name=f"site_elastic_virtual_{v_count}_{body.name.split('_')[-1]}_r",
#                          pos=[0.019, -0.004, -0.00625], size=[.001, .001, .001], material="blue elastic site")
#         # endregion

#     for i in range(1, n_fingers + 1):

#         count_v = 0
#         count_r = 0

#         for side in sides:
#             for j in range(1, int(n_body / 2 + 1)):
#                 count_r += 1

#                 tendon = foot_mjcf.tendon.add("spatial", name="elastic_" + str(i) + "_" + str(j) + "_" + side,
#                                               width=0.00025, limited="true", range=elastic_range,
#                                               rgba=[0.1, 0.5, 0.9, 1],
#                                               springlength=elastic_length, stiffness=elastic_stiffness,
#                                               damping=elastic_damping)

#                 if j == 6:
#                     tendon.add("site", site="site_elastic_virtual_" + str(j) + "_" + str(i) + "_" + side)
#                     tendon.add("site", site="site_elastic_reverse_phalanx_" + str(i) + "_" + side)

#                 elif j == 7:
#                     tendon.add("site", site="site_elastic_virtual_" + str(j) + "_" + str(i) + "_" + side)
#                     tendon.add("site", site="site_elastic_rollover_phalanx_" + str(i) + "_" + side)

#                 elif j == 8:
#                     tendon.add("site", site="site_elastic_virtual_" + str(j) + "_" + str(i) + "_" + side)
#                     tendon.add("site", site="site_elastic_phalanx_6_" + str(i) + "_" + side)

#                 elif j == 9:
#                     tendon.add("site", site="site_elastic_virtual_" + str(j) + "_" + str(i) + "_" + side)
#                     tendon.add("site", site="site_elastic_distal_phalanx_" + str(i) + "_" + side)

#                 else:
#                     tendon.add("site", site="site_elastic_virtual_" + str(j) + "_" + str(i) + "_" + side)
#                     tendon.add("site", site="site_elastic_phalanx_" + str(j) + "_" + str(i) + "_" + side)

#     return foot_mjcf


# def add_sensors(foot_mjcf, **config):
#     bodies = foot_mjcf.find_all("body")
#     joints = foot_mjcf.find_all("joint")
#     excluded_bodies = ["frontal_arch", "heel", "cube", "posterior_arch",
#                        "constraint_phalanx_1", "constraint_phalanx_2", "constraint_phalanx_3", "constraint_phalanx_4",
#                        "constraint_phalanx_5",
#                        "rollover_phalanx_1", "rollover_phalanx_2", "rollover_phalanx_3", "rollover_phalanx_4",
#                        "rollover_phalanx_5"]

#     # region define sites for the sensors and place imu
#     for body in [body for body in bodies if
#                  (body.name not in excluded_bodies and body.name.split("_")[0] != "virtual")]:
#         site = body.add("site", name="site_imu_" + body.name, pos=[0.0045, -0.012, 0], size=[.001, .001, .001],
#                         rgba=[0.85, 0.2, 0.45, 0.8], group=3)
#         # imu placement
#         foot_mjcf.sensor.add("accelerometer", site=site, name="IMU_acc_" + body.name)
#         foot_mjcf.sensor.add("gyro", site=site, name="IMU_gyro_" + body.name)
#     # endregion

#     # region place joint position sensor
#     for joint in joints:
#         foot_mjcf.sensor.add("jointpos", name="sensor_j_" + joint.name, joint=joint.name)
#     # # endregion

#     # sensor tendon
#     tendon = foot_mjcf.find("tendon", "tendon_1")
#     foot_mjcf.sensor.add("tendonpos", tendon=tendon, name="tendon_sensor")

#     return foot_mjcf


# def restyling(foot_mjcf, **config):
#     # add new materials to improve the aspect of the model
#     foot_mjcf.asset.add("material", name="red", rgba=[0.9, 0.1, 0.2, 1])
#     foot_mjcf.asset.add("material", name="gray", rgba=[0.3, 0.3, 0.3, 1])
#     foot_mjcf.asset.add("material", name="white", rgba=[0.9, 0.9, 0.9, 1])
#     foot_mjcf.asset.add("material", name="black", rgba=[0.1, 0.1, 0.1, 1])

#     # add material to geoms and remove rgba attribute
#     geoms = foot_mjcf.find_all("geom")
#     for geom in [geom for geom in geoms]:
#         if np.all(geom.rgba == [0.1, 0.1, 0.1, 1]):
#             geom.material = "black"
#         elif np.all(geom.rgba == [0.9, 0.9, 0.9, 1]):
#             geom.material = "white"
#         elif np.all(geom.rgba == [0.3, 0.3, 0.3, 1]):
#             geom.material = "gray"
#         elif np.all(geom.rgba == [0.9, 0.1, 0.2, 1]):
#             geom.material = "red"

#         del geom.rgba
#     return foot_mjcf


# # define the set and order of operation to perform on the basic foot model.
# DEFAULT_OPERATIONS = (general_fixes, constraint_sole_rolling, create_closed_loop, add_joint_stiffness_sole,
#                       add_sole_tendon, add_elastic, add_sensors)

# SENSORLESS_OPERATIONS = (general_fixes, constraint_sole_rolling, create_closed_loop, add_joint_stiffness_sole,
#                       add_sole_tendon, add_elastic, restyling)

# OPEN_CHAIN_OPERATIONS = (general_fixes, constraint_sole_rolling, add_joint_stiffness_sole, add_sole_tendon, add_elastic,
#                    add_sensors)

# NO_ELASTIC_OPERATIONS = (general_fixes, constraint_sole_rolling, create_closed_loop, add_joint_stiffness_sole,
#                       add_sole_tendon, add_sensors)

# ALL_OPERATIONS = (general_fixes, remove_inertials, constraint_sole_rolling, create_closed_loop,
#                   add_joint_stiffness_sole, add_sole_colliders, add_sole_tendon, add_back_springs, add_elastic,
#                   add_sensors)

ALL_OPERATIONS = (general_fixes,constraint_sole_rolling,add_joint_stiffness_sole)


def build_hand(xml_path="/home/esguerri/SoftHand/src/SoftHand-Plugin/softhands_description/urdf/xml_for_mujoco/right_softhand_v3.xml", operations=ALL_OPERATIONS, **config):
    hand_mjcf = mjcf.from_path(xml_path)
    print("Sono qua")
    
    for operation in operations:
        hand_mjcf = operation(hand_mjcf, **config)

    return hand_mjcf

destination = "export/"
model_name = "right_softhand_v3_edited"

def main():
    # hand_mjcf = build_hand(xml_path='/home/esguerri/SoftHand/src/SoftHand-Plugin/softhands_description/urdf/xml_for_mujoco/right_softhand_v1_2_research.xml', elastic_stiffness=0, joint_stiffness=0.5, elastic_range=[0.0, 0.04],
    #                              elastic_length=0.013, tendon_length=[0, 0.287], n_fingers=5, elastic_damping=70,
    #                              posterior_stiffness=0)
    
    hand_mjcf = build_hand(xml_path='/home/esguerri/SoftHand/src/SoftHand-Plugin/softhands_description/urdf/xml_for_mujoco/right_softhand_v3.xml')
    model = mujoco.MjModel.from_xml_string(hand_mjcf.to_xml_string(), hand_mjcf.get_assets())

    viewer.launch(model)
    print("Sono qui!")
    
    xml = re.sub("(-\w+)(?=\.stl)", "", hand_mjcf.to_xml_string())
    # print('xml: ', xml)
    xml = re.sub("<compiler", f"<compiler meshdir=\"{hand_mjcf.compiler.meshdir}\"", xml)

    f_name = destination + model_name + '.xml'
    with open(f_name, 'w') as file:
        file.write(xml)

    
if __name__ == '__main__':
    main()

import math
import time
import numpy as np
from isaacgym import gymapi, gymutil


current_positions = [0] * 21


def clamp(x, min_value, max_value):
    return max(min(x, max_value), min_value)




def transArray(array):
    tempArray =[]
    for i in range(len(array)):
        tempArray.append(transformPosition(array[i],lower_limits[i],upper_limits[i]))
    return tempArray


class AssetDesc:
    def __init__(self, file_name, flip_visual_attachments=False):
        self.file_name = file_name
        self.flip_visual_attachments = flip_visual_attachments


asset_descriptors = [
    AssetDesc("linker_hand_t24_1_right.urdf", False),#/home/moning/HHHHand/l20_8_urdf/mjcf/l20_8_urdf.xml

]



args = gymutil.parse_arguments(
    description="Linker Hand : Simulation for Gym",
    custom_parameters=[
        {"name": "--asset_id", "type": int, "default": 0, "help": "Asset id (0 - %d)" % (len(asset_descriptors) - 1)},
        {"name": "--speed_scale", "type": float, "default": 1.0, "help": "Animation speed scale"},
        {"name": "--show_axis", "action": "store_true", "help": "Visualize DOF axis"}])

if args.asset_id < 0 or args.asset_id >= len(asset_descriptors):
    print("*** Invalid asset_id specified.  Valid range is 0 to %d" % (len(asset_descriptors) - 1))
    quit()


# initialize gym
gym = gymapi.acquire_gym()

# configure sim
sim_params = gymapi.SimParams()
sim_params.dt = dt = 1.0 / 60.0
if args.physics_engine == gymapi.SIM_FLEX:
    pass
elif args.physics_engine == gymapi.SIM_PHYSX:
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 6
    sim_params.physx.num_velocity_iterations = 0
    sim_params.physx.num_threads = args.num_threads
    sim_params.physx.use_gpu = args.use_gpu

sim_params.use_gpu_pipeline = False
if args.use_gpu_pipeline:
    print("WARNING: Forcing CPU pipeline.")

sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)
if sim is None:
    print("*** Failed to create sim")
    quit()

# add ground plane
plane_params = gymapi.PlaneParams()
gym.add_ground(sim, plane_params)

# create viewer
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    print("*** Failed to create viewer")
    quit()

# load asset
asset_root = "../linker_hand"
asset_file = asset_descriptors[args.asset_id].file_name

asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = True
asset_options.flip_visual_attachments = asset_descriptors[args.asset_id].flip_visual_attachments
asset_options.use_mesh_materials = True
# *** START MODIFICATION: Enable position control ***
# We need to enable position control for the DOFs, so we'll set the default drive mode
asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS
# *** END MODIFICATION ***


print("Loading asset '%s' from '%s'" % (asset_file, asset_root))
asset = gym.load_asset(sim, asset_root, asset_file, asset_options)

# get array of DOF names
dof_names = gym.get_asset_dof_names(asset)

# get array of DOF properties
dof_props = gym.get_asset_dof_properties(asset)

# create an array of DOF states that will be used to update the actors
num_dofs = gym.get_asset_dof_count(asset)
dof_states = np.zeros(num_dofs, dtype=gymapi.DofState.dtype)

# get list of DOF types
dof_types = [gym.get_asset_dof_type(asset, i) for i in range(num_dofs)]

# get the position slice of the DOF state array
dof_positions = dof_states['pos']

# get the limit-related slices of the DOF properties array
stiffnesses = dof_props['stiffness']
dampings = dof_props['damping']
armatures = dof_props['armature']
has_limits = dof_props['hasLimits']
lower_limits = dof_props['lower']
upper_limits = dof_props['upper']
print(f" dof props : {dof_props}  dof names:{dof_names}" )

# *** START MODIFICATION: Set stiffness and damping for position control ***
# Set stiffness and damping values for all DOFs
# These values determine how stiffly the joints move to their targets
stiffnesses.fill(1000.0)
dampings.fill(100.0)
# *** END MODIFICATION ***

# initialize default positions, limits, and speeds (make sure they are in reasonable ranges)
defaults = np.zeros(num_dofs)
speeds = np.zeros(num_dofs)
for i in range(num_dofs):
    if has_limits[i]:
        if dof_types[i] == gymapi.DOF_ROTATION:
            lower_limits[i] = clamp(lower_limits[i], -math.pi, math.pi)
            upper_limits[i] = clamp(upper_limits[i], -math.pi, math.pi)
        # make sure our default position is in range
        if lower_limits[i] > 0.0:
            defaults[i] = lower_limits[i]
        elif upper_limits[i] < 0.0:
            defaults[i] = upper_limits[i]
    else:
        # set reasonable animation limits for unlimited joints
        if dof_types[i] == gymapi.DOF_ROTATION:
            # unlimited revolute joint
            lower_limits[i] = -math.pi
            upper_limits[i] = math.pi
        elif dof_types[i] == gymapi.DOF_TRANSLATION:
            # unlimited prismatic joint
            lower_limits[i] = -1.0
            upper_limits[i] = 1.0
    # set DOF position to default
    dof_positions[i] = defaults[i]
    # set speed depending on DOF type and range of motion
    if dof_types[i] == gymapi.DOF_ROTATION:
        speeds[i] = args.speed_scale * clamp(2 * (upper_limits[i] - lower_limits[i]), 0.25 * math.pi, 3.0 * math.pi)
    else:
        speeds[i] = args.speed_scale * clamp(2 * (upper_limits[i] - lower_limits[i]), 0.1, 7.0)

for i in range(num_dofs):
    print("DOF %d" % i)
    print("  Name:       '%s'" % dof_names[i])
    print("  Type:       %s" % gym.get_dof_type_string(dof_types[i]))
    print("  Stiffness:  %r" % stiffnesses[i])
    print("  Damping:    %r" % dampings[i])
    print("  Armature:   %r" % armatures[i])
    print("  Limited?    %r" % has_limits[i])
    if has_limits[i]:
        print("    Lower   %f" % lower_limits[i])
        print("    Upper   %f" % upper_limits[i])

num_envs = 1
num_per_row = 1
spacing = 1
env_lower = gymapi.Vec3(-spacing, 0.0, -spacing)
env_upper = gymapi.Vec3(spacing, spacing, spacing)

# position the camera
cam_pos = gymapi.Vec3(0, 1.5, 1.2)
cam_target = gymapi.Vec3(0, 1, 0)
gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

# cache useful handles
envs = []
actor_handles = []

print("Creating %d environments" % num_envs)
for i in range(num_envs):
    # create env
    env = gym.create_env(sim, env_lower, env_upper, num_per_row)
    envs.append(env)

    # add actor
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0.0, 1.32, 0.0)
    pose.r = gymapi.Quat.from_euler_zyx(3.14, -1.57, 1.57)

    actor_handle = gym.create_actor(env, asset, pose, "actor", i, 1)
    actor_handles.append(actor_handle)
    
    # *** START MODIFICATION: Apply the new DOF properties to the actor ***
    gym.set_actor_dof_properties(env, actor_handle, dof_props)
    # *** END MODIFICATION ***
    
    gym.set_actor_dof_states(env, actor_handle, dof_states, gymapi.STATE_ALL)#这个函数的含义是直接给环境中的一个对象设置相对应的位置和速度，不用时间步模拟
    #gym.set_actor_dof_position_targets(envs[i], actor_handles[i], dof_position_targets)
    #与target相比，target是设定目标，需要时间步的模拟进行步进。。
# Create a numpy array to hold the target positions
dof_position_targets = np.zeros(num_dofs, dtype=np.float32)

# Simulation loop
while not gym.query_viewer_has_closed(viewer):
    # Step the physics simulation
    gym.simulate(sim)  # Run the physics simulation step
    gym.fetch_results(sim, True)  # Fetch the results (including new positions, etc.)
    
    # *** START MODIFICATION: Define and set the target joint positions ***
    # In this example, we'll create a simple animation of closing and opening the hand
    # We use a sine wave to smoothly move the joints between their lower and upper limits.
    t = gym.get_sim_time(sim)
    
    # The 'phase' will cycle between 0 and 1
    phase = (np.sin(t * 2.0) + 1.0) / 2.0 
    
    # Calculate the target positions based on the phase
    # phase = 0 -> all joints at lower limit
    # phase = 1 -> all joints at upper limit
    for i in range(num_dofs):
        dof_position_targets[i] = (1.0 - phase) * lower_limits[i] + phase * upper_limits[i]

    # Apply the target positions to all actors (environments)
    for i in range(num_envs):
        # Here we use the key API call to send our target positions
        #gym.set_dof_position_target(envs[i], actor_handles[i], dof_position_targets)
        gym.set_actor_dof_position_targets(envs[i], actor_handles[i], dof_position_targets)
    # *** END MODIFICATION ***
    
    # Step the graphics (this will update the viewer)
    gym.step_graphics(sim)
    
    # Draw the updated scene (this renders the updated simulation to the viewer)
    gym.draw_viewer(viewer, sim, True)
    
    # Sync the frame time (ensure simulation and rendering are in sync)
    gym.sync_frame_time(sim)
    
# Cleanup
gym.destroy_viewer(viewer)
gym.destroy_sim(sim)

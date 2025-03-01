"""Forward and inverse kinematic transformations for a quadriped robot with calibration.

Equations from:
Sen, Muhammed Arif & Bakircioglu, Veli & Kalyoncu, Mete. (2017). 
Inverse Kinematic Analysis Of A Quadruped Robot.
International Journal of Scientific & Technology Research. 6. 
"""

from . import transformations
from math import pi, cos, sin, atan2, sqrt
import numpy as np

# Joint offset calibration parameters - to be set during calibration
joint_offsets = {
    'right_back': {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0},
    'right_front': {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0},
    'left_front': {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0},
    'left_back': {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0}
}

# Dimensional calibration parameters
dimensional_offsets = {
    'body_length_offset': 0.0,
    'body_width_offset': 0.0,
    'leg_lengths': {
        'right_back': {'l1': 0.0, 'l2': 0.0, 'l3': 0.0},
        'right_front': {'l1': 0.0, 'l2': 0.0, 'l3': 0.0},
        'left_front': {'l1': 0.0, 'l2': 0.0, 'l3': 0.0},
        'left_back': {'l1': 0.0, 'l2': 0.0, 'l3': 0.0}
    }
}

# Mounting angle offsets
mounting_offsets = {
    'right_back': np.eye(4),
    'right_front': np.eye(4),
    'left_front': np.eye(4),
    'left_back': np.eye(4)
}

# Rotation sign conventions for each leg and joint
# 1 means positive rotation as defined in the math, -1 means inverted
rotation_signs = {
    'right_back': {'theta1': 1, 'theta2': 1, 'theta3': 1},
    'right_front': {'theta1': 1, 'theta2': 1, 'theta3': 1},
    'left_front': {'theta1': -1, 'theta2': -1, 'theta3': -1},
    'left_back': {'theta1': -1, 'theta2': -1, 'theta3': -1}
}

def set_rotation_sign_convention(leg_name, theta1_sign, theta2_sign, theta3_sign):
    """Set the rotation sign convention for a specific leg
    
    Args:
        leg_name: String identifier for the leg ('right_back', 'right_front', etc.)
        theta1_sign: 1 for standard positive rotation, -1 for inverted rotation
        theta2_sign: 1 for standard positive rotation, -1 for inverted rotation
        theta3_sign: 1 for standard positive rotation, -1 for inverted rotation
    """
    if leg_name in rotation_signs:
        rotation_signs[leg_name]['theta1'] = theta1_sign
        rotation_signs[leg_name]['theta2'] = theta2_sign
        rotation_signs[leg_name]['theta3'] = theta3_sign
        print(f"Set rotation sign convention for {leg_name}: theta1={theta1_sign}, theta2={theta2_sign}, theta3={theta3_sign}")
    else:
        print(f"Error: Unknown leg name '{leg_name}'")

def apply_rotation_sign(leg_name, theta1, theta2, theta3):
    """Apply the rotation sign convention to convert from actual encoder values to internal model angles
    
    Args:
        leg_name: String identifier for the leg
        theta1, theta2, theta3: Raw angle values from encoders
        
    Returns:
        Tuple of angles adjusted for sign convention
    """
    if leg_name not in rotation_signs:
        return (theta1, theta2, theta3)
        
    signs = rotation_signs[leg_name]
    return (
        theta1 * signs['theta1'],
        theta2 * signs['theta2'],
        theta3 * signs['theta3']
    )

def revert_rotation_sign(leg_name, theta1, theta2, theta3):
    """Revert the rotation sign convention to convert from internal model angles to actual encoder values
    
    Args:
        leg_name: String identifier for the leg
        theta1, theta2, theta3: Internal model angle values
        
    Returns:
        Tuple of angles adjusted back to encoder convention
    """
    if leg_name not in rotation_signs:
        return (theta1, theta2, theta3)
        
    signs = rotation_signs[leg_name]
    return (
        theta1 * signs['theta1'],  # Multiplying by the same sign value inverts the operation
        theta2 * signs['theta2'],  # since signs are either 1 or -1
        theta3 * signs['theta3']
    )

def calibrate_leg_position(leg_name, actual_pose, measured_angles, nominal_l1, nominal_l2, nominal_l3):
    """Calibrate the leg kinematics to match actual pose with measured encoder angles.
    
    Args:
        leg_name: String identifier for the leg ('right_back', 'right_front', etc.)
        actual_pose: 4x4 homogeneous transform matrix of the actual leg end effector position
        measured_angles: Tuple of (theta1, theta2, theta3) measured by the encoders
        nominal_l1, nominal_l2, nominal_l3: Nominal link lengths from design specs
        
    Returns:
        None - updates the global calibration parameters
    """
    # Apply rotation sign convention to the measured angles
    theta1, theta2, theta3 = apply_rotation_sign(leg_name, *measured_angles)
    
    # Calculate forward kinematics using measured angles and nominal parameters
    nominal_fk = t_0_to_4(theta1, theta2, theta3, nominal_l1, nominal_l2, nominal_l3)
    
    # Calculate the error transformation
    error_transform = np.matmul(actual_pose, np.linalg.inv(nominal_fk))
    
    # Extract angle offsets (simplistic approach - could be enhanced with optimization)
    # This is a basic approach that computes joint offsets to make the forward 
    # kinematics match the actual pose
    
    # For demo purposes, we're using a simple approach:
    # 1. Try to find theta1 offset by looking at xy-plane rotation
    rotation_error = transformations.euler_from_matrix(error_transform[:3,:3], 'xyz')
    
    # Store the offset in the calibration dictionary
    joint_offsets[leg_name]['theta1'] = rotation_error[2]  # Z-axis rotation affects theta1
    
    # For a more comprehensive calibration, we would use optimization to find all parameters
    # that minimize the error between FK(measured_angles + offsets) and actual_pose
    
    # Update mounting offset for this leg
    mounting_offsets[leg_name] = error_transform
    
    print(f"Calibrated {leg_name} with offsets: {joint_offsets[leg_name]}")
    print(f"Mounting correction matrix added for {leg_name}")

def t_rightback(t_m, l, w):
    '''Creates a 4x4 numpy homogeneous transformation matrix representing coordinate system and 
    position of the rightback leg of a quadriped. Applies mounting calibration if available.

    Args:
        t_m: 4x4 numpy matrix. Homogeneous transform representing the coordinate system of the center
        of the robot body
        l: length of the robot body
        w: width of the robot body

    Returns: 
        4x4 numpy matrix. A homogeneous transformation representing the position of the right back leg
    '''
    # Apply dimensional offsets
    l_calibrated = l + dimensional_offsets['body_length_offset']
    w_calibrated = w + dimensional_offsets['body_width_offset']
    
    temp_homog_transf = np.block([[transformations.roty(pi/2), np.array([[-l_calibrated/2], [0], [w_calibrated/2]])],
                                 [np.array([0, 0, 0, 1])]])
    
    # Apply the base transformation
    base_transform = np.matmul(t_m, temp_homog_transf)
    
    # Apply mounting correction if calibrated
    return np.matmul(base_transform, mounting_offsets['right_back'])

def t_rightfront(t_m, l, w):
    '''Creates a 4x4 numpy homogeneous transformation matrix representing coordinate system and 
    position of the rightfront leg of a quadriped. Applies mounting calibration if available.

    Args:
        t_m: 4x4 numpy matrix. Homogeneous transform representing the coordinate system of the center
        of the robot body
        l: length of the robot body
        w: width of the robot body

    Returns: 
        4x4 numpy matrix. A homogeneous transformation representing the position of the right front leg
    '''
    # Apply dimensional offsets
    l_calibrated = l + dimensional_offsets['body_length_offset']
    w_calibrated = w + dimensional_offsets['body_width_offset']
    
    temp_homog_transf = np.block([[transformations.roty(pi/2), np.array([[l_calibrated/2], [0], [w_calibrated/2]])],
                                 [np.array([0, 0, 0, 1])]])
    
    # Apply the base transformation
    base_transform = np.matmul(t_m, temp_homog_transf)
    
    # Apply mounting correction if calibrated
    return np.matmul(base_transform, mounting_offsets['right_front'])

def t_leftfront(t_m, l, w):
    '''Creates a 4x4 numpy homogeneous transformation matrix representing coordinate system and 
    position of the left front leg of a quadriped. Applies mounting calibration if available.

    Args:
        t_m: 4x4 numpy matrix. Homogeneous transform representing the coordinate system of the center
        of the robot body
        l: length of the robot body
        w: width of the robot body

    Returns: 
        4x4 numpy matrix. A homogeneous transformation representing the position of the left front leg
    '''
    # Apply dimensional offsets
    l_calibrated = l + dimensional_offsets['body_length_offset']
    w_calibrated = w + dimensional_offsets['body_width_offset']
    
    temp_homog_transf = np.block([[transformations.roty(-pi/2), np.array([[l_calibrated/2], [0], [-w_calibrated/2]])],
                                 [np.array([0, 0, 0, 1])]])
    
    # Apply the base transformation
    base_transform = np.matmul(t_m, temp_homog_transf)
    
    # Apply mounting correction if calibrated
    return np.matmul(base_transform, mounting_offsets['left_front'])

def t_leftback(t_m, l, w):
    '''Creates a 4x4 numpy homogeneous transformation matrix representing coordinate system and 
    position of the left back leg of a quadriped. Applies mounting calibration if available.

    Args:
        t_m: 4x4 numpy matrix. Homogeneous transform representing the coordinate system of the center
        of the robot body
        l: length of the robot body
        w: width of the robot body

    Returns: 
        4x4 numpy matrix. A homogeneous transformation representing the position of the left back leg
    '''
    # Apply dimensional offsets
    l_calibrated = l + dimensional_offsets['body_length_offset']
    w_calibrated = w + dimensional_offsets['body_width_offset']
    
    temp_homog_transf = np.block([[transformations.roty(-pi/2), np.array([[-l_calibrated/2], [0], [-w_calibrated/2]])],
                                 [np.array([0, 0, 0, 1])]])
    
    # Apply the base transformation
    base_transform = np.matmul(t_m, temp_homog_transf)
    
    # Apply mounting correction if calibrated
    return np.matmul(base_transform, mounting_offsets['left_back'])

def t_0_to_1(theta1, l1, leg_name=None):
    '''Create the homogeneous transformation matrix for joint 0 to 1 for a quadriped leg.

    Args:
        theta1: Rotation angle in radians of the hip joint
        l1: Length of the hip joint link
        leg_name: Optional string to identify which leg's calibration to apply

    Returns:
        A 4x4 numpy matrix. Homogeneous transform from joint 0 to 1
    '''
    # Apply joint angle offset if leg_name is provided
    if leg_name and leg_name in joint_offsets:
        # Note: We don't apply rotation sign here because this function is called with already
        # sign-adjusted angles from t_0_to_4()
        theta1_calibrated = theta1 + joint_offsets[leg_name]['theta1']
        # Apply leg length offset if available
        l1_calibrated = l1
        if leg_name in dimensional_offsets['leg_lengths']:
            l1_calibrated = l1 + dimensional_offsets['leg_lengths'][leg_name]['l1']
    else:
        theta1_calibrated = theta1
        l1_calibrated = l1
    
    t_01 = np.block([[transformations.rotz(theta1_calibrated), 
                     np.array([[-l1_calibrated*cos(theta1_calibrated)],
                              [-l1_calibrated*sin(theta1_calibrated)],
                              [0]])],
                     [np.array([0, 0, 0, 1])]])
    return t_01

def t_1_to_2():
    '''Create the homogeneous transformation matrix for joint 1 to 2 for a quadriped leg.

    Args:
        None

    Returns:
        A 4x4 numpy matrix. Homogeneous transform from joint 1 to 2
    '''
    t_12 = np.array([[0, 0, -1, 0],
                     [-1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 0, 1]])
    return t_12

def t_2_to_3(theta2, l2, leg_name=None):
    '''Create the homogeneous transformation matrix for joint 2 to 3 for a quadriped leg.

    Args:
        theta2: Rotation angle in radians of the leg joint
        l2: Length of the upper leg link
        leg_name: Optional string to identify which leg's calibration to apply

    Returns:
        A 4x4 numpy matrix. Homogeneous transform from joint 2 to 3
    '''
    # Apply joint angle offset if leg_name is provided
    if leg_name and leg_name in joint_offsets:
        # Note: We don't apply rotation sign here because this function is called with already
        # sign-adjusted angles from t_0_to_4()
        theta2_calibrated = theta2 + joint_offsets[leg_name]['theta2']
        # Apply leg length offset if available
        l2_calibrated = l2
        if leg_name in dimensional_offsets['leg_lengths']:
            l2_calibrated = l2 + dimensional_offsets['leg_lengths'][leg_name]['l2']
    else:
        theta2_calibrated = theta2
        l2_calibrated = l2

    t_23 = np.block([[transformations.rotz(theta2_calibrated), 
                     np.array([[l2_calibrated*cos(theta2_calibrated)],
                              [l2_calibrated*sin(theta2_calibrated)],
                              [0]])],
                     [np.array([0, 0, 0, 1])]])
    return t_23

def t_3_to_4(theta3, l3, leg_name=None):
    '''Create the homogeneous transformation matrix for joint 3 to 4 for a quadriped leg.

    Args:
        theta3: Rotation angle in radians of the knee joint
        l3: Length of the lower leg link
        leg_name: Optional string to identify which leg's calibration to apply

    Returns:
        A 4x4 numpy matrix. Homogeneous transform from joint 3 to 4
    '''
    # Apply joint angle offset if leg_name is provided
    if leg_name and leg_name in joint_offsets:
        # Note: We don't apply rotation sign here because this function is called with already
        # sign-adjusted angles from t_0_to_4()
        theta3_calibrated = theta3 + joint_offsets[leg_name]['theta3']
        # Apply leg length offset if available
        l3_calibrated = l3
        if leg_name in dimensional_offsets['leg_lengths']:
            l3_calibrated = l3 + dimensional_offsets['leg_lengths'][leg_name]['l3']
    else:
        theta3_calibrated = theta3
        l3_calibrated = l3

    t_34 = np.block([[transformations.rotz(theta3_calibrated), 
                     np.array([[l3_calibrated*cos(theta3_calibrated)],
                              [l3_calibrated*sin(theta3_calibrated)],
                              [0]])],
                     [np.array([0, 0, 0, 1])]])
    return t_34

def t_0_to_4(theta1, theta2, theta3, l1, l2, l3, leg_name=None):
    '''Create the homogeneous transformation matrix from joint 0 to 4 of a quadriped leg

    Args:
        theta1: Rotation angle in radians of joint 1
        theta2: Rotation angle in radians of joint 2
        theta3: Rotation angle in radians of joint 3
        l1: Length of leg link 1, the hip length
        l2: Length of leg link 2, the uppper leg length
        l3: Length of leg link 3, the lower leg
        leg_name: Optional string to identify which leg's calibration to apply
    
    Returns:
        A 4x4 numpy matrix. Homogeneous transform from joint 0 to 4
    '''
    # If leg_name provided, apply the rotation sign convention
    if leg_name and leg_name in rotation_signs:
        theta1, theta2, theta3 = apply_rotation_sign(leg_name, theta1, theta2, theta3)
    
    return np.matmul(
        np.matmul(
            np.matmul(
                t_0_to_1(theta1, l1, leg_name), 
                t_1_to_2()
            ), 
            t_2_to_3(theta2, l2, leg_name)
        ), 
        t_3_to_4(theta3, l3, leg_name)
    )

def ikine(x4, y4, z4, l1, l2, l3, legs12=True, leg_name=None):
    '''Use inverse kinematics to calculate the leg angles for a leg to achieve a desired
    leg end point position (x4,y4,z4)

    Args:
        x4: x position of leg end point relative to leg start point coordinate system.
        y4: y position of leg end point relative to leg start point coordinate system.
        z4: z position of leg end point relative to leg start point coordinate system.
        l1: leg link 1 length
        l2: leg link 2 length
        l3: leg link 3 length
        legs12: Optional input, boolean indicating whether equations are for legs 1 or 2. 
                If false, then equation for legs 3 and 4 is used
        leg_name: Optional string to identify which leg's calibration to apply for reverse calculation

    Returns:
        A length 3 tuple of leg angles in the order (q1,q2,q3) in the encoder's convention
    '''
    # Apply leg length calibration if available
    if leg_name and leg_name in dimensional_offsets['leg_lengths']:
        l1_calibrated = l1 + dimensional_offsets['leg_lengths'][leg_name]['l1']
        l2_calibrated = l2 + dimensional_offsets['leg_lengths'][leg_name]['l2']
        l3_calibrated = l3 + dimensional_offsets['leg_lengths'][leg_name]['l3']
    else:
        l1_calibrated = l1
        l2_calibrated = l2
        l3_calibrated = l3

    # Supporting variable D
    D = (x4**2 + y4**2 + z4**2 - l1_calibrated**2 - l2_calibrated**2 - l3_calibrated**2)/(2*l2_calibrated*l3_calibrated)

    if legs12:
        q3 = atan2(sqrt(1-D**2), D)
    else:
        q3 = atan2(-sqrt(1-D**2), D)
    
    q2 = atan2(z4, sqrt(x4**2 + y4**2 - l1_calibrated**2)) - atan2(l3_calibrated*sin(q3), l2_calibrated + l3_calibrated*cos(q3))  

    q1 = atan2(y4, x4) + atan2(sqrt(x4**2 + y4**2 - l1_calibrated**2), -l1_calibrated)

    # Apply inverse of joint angle offsets if leg_name is provided
    if leg_name and leg_name in joint_offsets:
        q1 -= joint_offsets[leg_name]['theta1']
        q2 -= joint_offsets[leg_name]['theta2']
        q3 -= joint_offsets[leg_name]['theta3']

    # Convert from internal model angles to encoder convention
    if leg_name and leg_name in rotation_signs:
        q1, q2, q3 = revert_rotation_sign(leg_name, q1, q2, q3)

    return (q1, q2, q3)

def initialize_leg_configuration():
    """Initialize the leg configuration with default rotation sign conventions based on typical robot designs
    
    This sets up standard conventions:
    - Right legs have positive rotations as in the model
    - Left legs have inverted rotations relative to the model
    """
    # Default configuration for a typical quadruped
    set_rotation_sign_convention('right_back', 1, 1, 1)   # Right legs standard rotation direction
    set_rotation_sign_convention('right_front', 1, 1, 1)  # Right legs standard rotation direction
    set_rotation_sign_convention('left_front', -1, -1, -1)  # Left legs inverted rotation direction
    set_rotation_sign_convention('left_back', -1, -1, -1)   # Left legs inverted rotation direction
    
    print("Initialized default leg configuration with standard rotation sign conventions")

def save_calibration(filename):
    """Save calibration parameters to a file"""
    import json
    
    # Convert numpy arrays to lists for JSON serialization
    serializable_mounting_offsets = {}
    for leg, matrix in mounting_offsets.items():
        serializable_mounting_offsets[leg] = matrix.tolist()
    
    # Create calibration dictionary
    calibration = {
        'joint_offsets': joint_offsets,
        'dimensional_offsets': dimensional_offsets,
        'mounting_offsets': serializable_mounting_offsets,
        'rotation_signs': rotation_signs
    }
    
    with open(filename, 'w') as f:
        json.dump(calibration, f, indent=2)
    
    print(f"Calibration saved to {filename}")

def load_calibration(filename):
    """Load calibration parameters from a file"""
    import json
    global joint_offsets, dimensional_offsets, mounting_offsets, rotation_signs
    
    with open(filename, 'r') as f:
        calibration = json.load(f)
    
    joint_offsets = calibration['joint_offsets']
    dimensional_offsets = calibration['dimensional_offsets']
    
    # Load rotation signs if available
    if 'rotation_signs' in calibration:
        rotation_signs = calibration['rotation_signs']
    
    # Convert lists back to numpy arrays
    for leg, matrix_list in calibration['mounting_offsets'].items():
        mounting_offsets[leg] = np.array(matrix_list)
    
    print(f"Calibration loaded from {filename}")

# Initialize standard rotation sign conventions by default
initialize_leg_configuration()
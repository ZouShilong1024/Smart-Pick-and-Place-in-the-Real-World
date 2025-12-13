import json
import os
import base64

from PIL import Image
from io import BytesIO
import numpy as np
#from termcolor import colored
import datetime
import pybullet as p
from scipy.spatial.transform import Rotation as Rot

def get_time():
    current_time = datetime.datetime.now()
    return current_time.strftime("%y-%m-%d-%H-%M-%S")


def create_directory(path):
    try:
        # Create the directory
        os.makedirs(path)
        print(f"Directory created at {path}")
    except FileExistsError:
        pass
        # print(f"Directory already exists at {path}")
    except Exception as e:
        print(f"An error occurred while creating directory at {path}: {e}")


def load_image_as_pil(path, resize = True):
    try:
        # Open the image file
        img = Image.open(path)
        # Convert to RGB mode if it's not already in that mode
        img = img.convert("RGB")
        if resize:
            img = img.resize((512, 512))
        return img
    except FileNotFoundError:
        print("File not found. Please make sure you entered the correct path.")
        return None
    except Exception as e:
        print("An error occurred:", e)
        return None

# check if image is type Image, if not, change it
def convert_to_Image(input_variable):
    if isinstance(input_variable, Image.Image):
        # It's already a PIL Image, no need to convert
        return input_variable
    elif isinstance(input_variable, np.ndarray):
        # It's a NumPy array, convert it to PIL Image
        return Image.fromarray(input_variable)
    else:
        raise ValueError("Unsupported input type. It must be either 'Image' or 'numpy.ndarray'.")


# Function to encode the Image format img
def encode_image_from_Image(image, resize_image = None):
    if resize_image is not None:
        image = image.resize((resize_image[0], resize_image[1]))
    with BytesIO() as buffer:
        image.save(buffer, format="PNG")
        image_bytes = buffer.getvalue()
    base64_encoded = base64.b64encode(image_bytes).decode("utf-8")
    return base64_encoded

# Function to encode the image from path
def encode_image_from_path(image_path):
    input_image = Image.open("screenshots/floor_apple.png")
    encode_image_from_Image(input_image)


def load_json_file(path):
    try:
        with open(path, 'r') as file:
            data = json.load(file)
        return data
    except FileNotFoundError:
        print(f"File not found: {path}")
        return None
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON in file {path}: {str(e)}")
        return None
    

def load_txt_file(path):
    try:
        with open(path, 'r') as file:
            content = file.read()
        return content
    except FileNotFoundError:
        print(f"File not found: {path}")
        return None


def dict_to_string(input_dict):
    try:
        # Use json.dumps() to convert the dictionary to a JSON-formatted string
        json_string = json.dumps(input_dict, indent=4)  # You can specify the indentation level for pretty printing
        return json_string
    except Exception as e:
        return str(e)

def json_string_to_dict(json_string):
    try:
        data_dict = json.loads(json_string)
        return data_dict
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON: {str(e)}")
        return None


def save_string_to_txt(string, filename):
    try:
        with open(filename, 'w') as file:
            file.write(string)
        # print(f'Successfully saved the string to {filename}')
    except Exception as e:
        print(f'An error occurred while saving the string to {filename}: {e}')

def save_dict_to_json(data_dict, file_path):
    try:
        with open(file_path, 'w') as file:
            json.dump(data_dict, file, indent=4)
        # print(f"Dictionary saved as JSON to {file_path}")
    except Exception as e:
        print(f"Error saving dictionary as JSON: {str(e)}")



def list_subdirectories(directory_path):
    try:
        # Get the list of all entries in the directory
        entries = os.listdir(directory_path)
        
        # Filter the list to include only subdirectories
        subdirectories = [entry for entry in entries if os.path.isdir(os.path.join(directory_path, entry))]
        
        return subdirectories
    except Exception as e:
        return f"An error occurred: {e}"

def check_file_exists(file_path):
    try:
        # Check if the file exists
        if os.path.isfile(file_path):
            return True
        else:
            return False
    except Exception as e:
        return f"An error occurred: {e}"

def read_txt_files_in_directory(directory_path):
    txt_files = []
    txt_contents = []

    try:
        # List all files in the directory
        files = os.listdir(directory_path)
        # Filter only text files
        txt_files = [file for file in files if file.endswith('.txt')]
        
        for file_name in txt_files:
            file_path = os.path.join(directory_path, file_name)
            with open(file_path, 'r') as file:
                # Read content of each text file
                content = file.read()
                txt_contents.append(content)
        
    except FileNotFoundError:
        print("Directory not found. Please make sure you entered the correct path.")
    except Exception as e:
        print("An error occurred:", e)
    
    txt_files2 = [file.split('.')[0] for file in txt_files]
    
    return txt_files2, txt_contents



############################## <<< related with transformation <<<<< 

def calcul_transformation_matrix(pos, orn):
    R_matrix = np.array(p.getMatrixFromQuaternion([orn[0], orn[1], orn[2], orn[3]])).reshape(3, 3)
    
    T = np.eye(4)
    # Place the rotation matrix in the top-left 3x3 part
    T[0:3, 0:3] = R_matrix[:,:]
    # Set the translation vector
    T[0:3, 3] = pos
    
    
    return T


def draw_axis(T, axis_len=0.1, line_width=2, duration=1):
    origin = T[:3, 3]
    R_mat = T[:3, :3]
    axis_x = origin + axis_len * R_mat[:, 0]
    axis_y = origin + axis_len * R_mat[:, 1]
    axis_z = origin + axis_len * R_mat[:, 2]

    p.addUserDebugLine(
        lineFromXYZ=list(origin.reshape(-1)),
        lineToXYZ=list(axis_x.reshape(-1)),
        lineColorRGB=[1, 0, 0],
        lineWidth=line_width,
        lifeTime=duration
    )
    p.addUserDebugLine(
        lineFromXYZ=list(origin.reshape(-1)),
        lineToXYZ=list(axis_y.reshape(-1)),
        lineColorRGB=[0, 1, 0],
        lineWidth=line_width,
        lifeTime=duration
    )
    p.addUserDebugLine(
        lineFromXYZ=list(origin.reshape(-1)),
        lineToXYZ=list(axis_z.reshape(-1)),
        lineColorRGB=[0, 0, 1],
        lineWidth=line_width,
        lifeTime=duration
    )


def rot2eul(R_matrix):
    rotation = Rot.from_matrix(R_matrix)
    roll, pitch, yaw = rotation.as_euler('xyz', degrees=False)
    return np.array([roll, pitch, yaw])

def calcul_pos_orn(T):
    R_matix= T[0:3, 0:3]
    pos = T[0:3, 3]
    rpy = rot2eul(R_matix)
    orn = p.getQuaternionFromEuler(rpy)
    return pos, orn


def calculate_fov(focal_length, sensor_size):
    
    fov_rad = 2 * np.arctan(sensor_size / (2 * focal_length))
    fov_deg = np.degrees(fov_rad)
    return fov_deg


def slerp(q1, q2, num_points):
    """
    Perform spherical linear interpolation (SLERP) between two quaternions.

    :param q1: First quaternion (as a list or array [x, y, z, w])
    :param q2: Second quaternion (as a list or array [x, y, z, w])
    :param num_points: Number of interpolated points (including start and end)

    :return: List of interpolated quaternions (as a list of lists)
    """
    # Convert input quaternions to numpy arrays
    q1 = np.array(q1)
    q2 = np.array(q2)

    
    # Compute the dot product
    dot = np.dot(q1, q2)

    # If the dot product is negative, negate one quaternion to avoid interpolation through the long way
    if dot < 0.0:
        q2 = -q2
        dot = -dot

    # Clamp dot product to the range [ -1, 1 ] to avoid numerical issues
    dot = np.clip(dot, -1.0, 1.0)

    # Compute the angle between the quaternions
    theta_0 = np.arccos(dot)
    theta = np.linspace(0, theta_0, num_points)

    # Compute the interpolated quaternions
    interpolated_quaternions = []
    for t in theta:
        # Compute the interpolation factor for both quaternions
        sin_theta = np.sin(t)
        sin_theta_0 = np.sin(theta_0)
        if sin_theta_0 == 0:
            q_interpolated = q2
        else:
            s1 = np.cos(t) - dot * sin_theta / sin_theta_0
            s2 = sin_theta / sin_theta_0

            # Interpolate the quaternions
            q_interpolated = (s1 * q1) + (s2 * q2)
        interpolated_quaternions.append(q_interpolated)

    return interpolated_quaternions
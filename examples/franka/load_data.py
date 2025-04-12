import numpy as np

# Load the .npz file
file_path = '/home/yiyang/hsc/ogvla_realworld/data/train/vd_trajectory_1.npz'  # Replace with the actual file path
data = np.load(file_path)

# Display the content of the .npz file
print("Keys in the .npz file:", data.files)

# Iterate through the keys and print the content
for key in data.files:
    print(f"Content under key '{key}':")
    print(data[key])
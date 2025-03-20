import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np  # Add numpy for color normalization

def plot_3d_coordinates(data_file):
    with open(data_file, encoding='utf-8') as f:
        data = json.load(f)
    
    x, y, z = data['Matrice']
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Normalize y values for color mapping
    norm = plt.Normalize(min(z), max(z))
    colors = plt.colormaps.get_cmap('viridis')(norm(z))  # Use 'viridis' colormap for gradient
    
    ax.scatter(x, y, z, c=colors)
    
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
        
    # Set equal aspect ratio
    ax.set_box_aspect([1,1,1])
    
    plt.show()

# Utilisation de la fonction
plot_3d_coordinates('data.json')

import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_3d_coordinates(data_file):
    with open(data_file, encoding='utf-8') as f:
        data = json.load(f)
    
    x, y, z = data['Matrice']
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z)
    
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    
    plt.show()

# Utilisation de la fonction
plot_3d_coordinates('data.json')

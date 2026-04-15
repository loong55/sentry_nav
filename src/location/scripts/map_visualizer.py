import yaml
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import os
import sys

def load_map(yaml_path):
    with open(yaml_path, 'r') as file:
        map_data = yaml.safe_load(file)
    image_path = os.path.join(os.path.dirname(yaml_path), map_data['image'])
    image = mpimg.imread(image_path)
    resolution = map_data['resolution']
    origin = map_data['origin']
    return image, resolution, origin

def visualize_map(image, resolution, origin):
    height, width = image.shape
    extent = [
        origin[0], 
        origin[0] + width * resolution, 
        origin[1], 
        origin[1] + height * resolution
    ]

    fig, ax = plt.subplots()
    # 对图像进行垂直翻转，解决y轴镜像问题
    ax.imshow(np.flipud(image), cmap='gray', origin='lower', extent=extent)
    ax.quiver(origin[0], origin[1], 1, 0, scale=5, color='red', label='X-axis')
    ax.quiver(origin[0], origin[1], 0, 1, scale=5, color='blue', label='Y-axis')
    plt.scatter(0, 0, c='red', marker='o', label='Origin')
    plt.text(0, 0, '(0,0)', color='red', fontsize=10, ha='right', va='bottom')
    ax.legend()

    marked_points = []  # 存储标记的点

    def onclick(event):
        if event.inaxes == ax and event.button == 1:  # 鼠标左键点击
            x, y = event.xdata, event.ydata
            marked_points.append((x, y))
            ax.scatter(x, y, c='green', marker='x', label='Marked Point')
            ax.text(x, y, f'({x:.2f},{y:.2f})', color='green', fontsize=8, ha='left', va='bottom')
            fig.canvas.draw_idle()

    def onkeypress(event):
        if event.key == 'c':  # 按下 'c' 键清除标记点
            while marked_points:
                marked_points.pop()
            ax.clear()
            # 重新绘制地图和原点
            ax.imshow(np.flipud(image), cmap='gray', origin='lower', extent=extent)
            ax.quiver(origin[0], origin[1], 1, 0, scale=5, color='red', label='X-axis')
            ax.quiver(origin[0], origin[1], 0, 1, scale=5, color='blue', label='Y-axis')
            plt.scatter(0, 0, c='red', marker='o', label='Origin')
            plt.text(0, 0, '(0,0)', color='red', fontsize=10, ha='right', va='bottom')
            ax.legend()
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect('button_press_event', onclick)
    fig.canvas.mpl_connect('key_press_event', onkeypress)
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Map Visualization')
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 map_visualizer.py <map_yaml_path>")
        sys.exit(1)

    yaml_path = os.path.abspath(sys.argv[1])
    image, resolution, origin = load_map(yaml_path)
    visualize_map(image, resolution, origin)

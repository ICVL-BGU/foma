import matplotlib.pyplot as plt

def display_dot(x, y):
    if not (0 <= x <= 5 and 0 <= y <= 5):
        raise ValueError("Coordinates must be in the range [0,5].")
    
    fig, ax = plt.subplots(figsize=(5, 5))
    ax.set_xlim(0, 5)
    ax.set_ylim(5, 0)  # Invert y-axis to start from the top
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_frame_on(False)
    
    ax.scatter(x, y, color='black', s=100)
    plt.show()

# Example usage:
display_dot(1.188712284482759, 0.8620689655172415)

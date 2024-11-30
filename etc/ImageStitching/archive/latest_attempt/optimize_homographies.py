import pickle
import numpy as np
import cv2
from scipy.optimize import minimize

def loss(homographies_flat, point_sets, base_idx):
    num_matrices = len(homographies_flat) // 9
    homographies = [homographies_flat[i*9:(i+1)*9].reshape(3, 3) for i in range(num_matrices)]
    homographies.insert(base_idx, np.eye(3))
    # print(len(homographies))
    # print(homographies)
    errors = []
    for idxs, sets in point_sets.items():
        # print(idxs)
        if len(sets[0]) != len(sets[1]):
            raise ValueError("Number of points in point sets must be equal.")
        points_1 = cv2.perspectiveTransform(sets[0].reshape(-1, 1, 2), homographies[idxs[0]])
        # print(homographies[idxs[1]])
        points_2 = cv2.perspectiveTransform(sets[1].reshape(-1, 1, 2), homographies[idxs[1]])
        
        points_1 = points_1.reshape(-1, 2)
        points_2 = points_2.reshape(-1, 2)
        
        error = points_1 - points_2
        if base_idx not in idxs:
            error = error * 5
        errors.append(error)
    
    errors = np.vstack(errors)
    return np.linalg.norm(errors)

def optimize_homographies(homographies, point_sets, base_idx):
    initial_guess = np.array([homographies[h].flatten() for h in homographies if not np.array_equal(h, np.eye(3))]).flatten()
    result = minimize(loss, initial_guess, args=(point_sets, base_idx), method='L-BFGS-B')
    
    optimized_homographies_flat = result.x
    optimized_homographies = [optimized_homographies_flat[i*9:(i+1)*9].reshape(3, 3) for i in range(len(homographies))]
    
    return optimized_homographies

if __name__ == "__main__":
    with open('homographies.pkl', 'rb') as f:
        homographies = pickle.load(f)
    with open('points_pairs.pkl', 'rb') as f:
        point_sets = pickle.load(f)
    
    base_idx = 2
    optimized_homographies = optimize_homographies(homographies, point_sets, base_idx)
    
    for i, H in enumerate(optimized_homographies):
        # print(f"Homography for image {i}:\n{homographies[(base_idx,i if i<2 else i+1)]}\n")
        print(f"Optimized homography for image {i}:\n{H}\n")

    with open('optimized_homographies.pkl', 'wb') as f:
        pickle.dump(optimized_homographies, f)
        

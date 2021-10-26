import numpy as np

def getA1(th1, d1):
    c1 = np.cos(th1)
    s1 = np.sin(th1)
    return np.array([
        [c1, 0, -s1, 0],
        [s1, 0, c1, 0],
        [0, -1, 0, d1],
        [0, 0, 0, 1]
    ])

def getA2(th2, a2, d2):
    c1 = np.cos(th2)
    s1 = np.sin(th2)
    return np.array([
        [c1, -s1, 0, a2 * c1],
        [s1, c1, 0, a2 * s1],
        [0, 0, 1, d2],
        [0, 0, 0, 1]
    ])

def up(arr:np.array):
    return np.array([
        [0, -arr[2, 0], arr[1, 0]],
        [arr[2, 0], 0, -arr[0, 0]],
        [-arr[1, 0], arr[0, 0], 0]
    ])

def rodrigues(theta:float, arr:np.array)->np.array:
    if not arr.shape == (3, 1):
        arr = arr.reshape((3, 1))
    return np.cos(theta) * np.eye(3) + (1 - np.cos(theta)) * (arr @ arr.T) + np.sin(theta) * up(arr) 

if __name__ == "__main__":
    A1 = getA1(np.pi / 4, 1)
    A2 = getA2(np.pi / 4, 2, 0.5)
    A3 = getA1(np.pi / 4, 0.5)
    A = (A1 @ A2 @ A3)
    print("A1:")
    print(A1)
    print("A2:")
    print(A2)
    print("A3:")
    print(A3)
    print("A:")
    print(A)
    print(rodrigues(np.pi / 2, np.array([0, 1, 0])))
    R = np.array([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, -1]
    ])
    print(np.linalg.inv(R))
    print(A[:3, :3] @ np.array([[1,],[0,],[0,]]))
    print(A[:3, :3] @ np.array([[0,],[1,],[0,]]))
    print(A[:3, :3] @ np.array([[0,],[0,],[1,]]))
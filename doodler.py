import numpy as np
import matplotlib.pyplot as plt

LA1 = 1
LA2 = 0.5
LA3 = 0.75

LB1 = 1
LB2 = 0.5
LB3 = 0.75

thetaA1 = np.deg2rad(90)
thetaA2 = np.deg2rad(0)

thetaB1 = np.deg2rad(0)
thetaB2 = np.deg2rad(0)

r_A1_O_O  = np.array([LA1*np.cos(thetaA1), LA1*np.sin(thetaA1)])
r_A2_A1_O = np.array([LA2*np.cos(thetaA2), LA2*np.sin(thetaA2)])

r_A2_O_O = r_A1_O_O + r_A2_A1_O

r_B1_O_O  = np.array([LB1*np.cos(thetaB1), LB1*np.sin(thetaB1)])
r_B2_B1_O = np.array([LB2*np.cos(thetaB2), LB2*np.sin(thetaB2)])

r_B2_O_O = r_B1_O_O + r_B2_B1_O

def find_intersection(C0,C1,r0,r1):
    x0 = C0[0]
    y0 = C0[1]
    x1 = C1[0]
    y1 = C1[1]

    d = np.sqrt((x1-x0)**2 + (y1-y0)**2)
    tol = 1e-9

    if d > r0 + r1 + tol:
        print("No solution")
        return None
    elif d < abs(r0 - r1) - tol:
        print("No solution")
        return None
    elif np.isclose(d, 0.0, atol=tol) and np.isclose(r0, r1, atol=tol):
        print("Infinite solutions")
        return None
    else:
        a = (r0**2 - r1**2 + d**2) / (2*d)
        h = np.sqrt(max(0.0, r0**2 - a**2))
        x2 = x0 + a*(x1-x0)/d
        y2 = y0 + a*(y1-y0)/d

        xi1 = x2 + h*(y1-y0)/d
        yi1 = y2 - h*(x1-x0)/d
        xi2 = x2 - h*(y1-y0)/d
        yi2 = y2 + h*(x1-x0)/d

        if np.isclose(h, 0.0, atol=tol):
            print("One solution")
            return np.array([x2, y2])

        # Return the point with the lower y value for the two-intersection case
        if yi2 > yi1:
            return np.array([xi1, yi1])
        else:
            return np.array([xi2, yi2])

def plot_circles(C0, r0, C1, r1 , ri):
    fig, ax = plt.subplots()
    plt.plot([0,r_A1_O_O[0]],[0,r_A1_O_O[1]])
    plt.plot([0,r_B1_O_O[0]],[0,r_B1_O_O[1]])
    plt.plot([r_A1_O_O[0],r_A2_O_O[0]],[r_A1_O_O[1],r_A2_O_O[1]])
    plt.plot([r_B1_O_O[0],r_B2_O_O[0]],[r_B1_O_O[1],r_B2_O_O[1]])
    if ri is not None:
        plt.plot(ri[0], ri[1], 'go', label='Intersection Point')
    circle1 = plt.Circle(C0, r0, color='blue', fill=False)
    circle2 = plt.Circle(C1, r1, color='red', fill=False)
    ax.add_artist(circle1)
    ax.add_artist(circle2)
    ax.set_aspect('equal', adjustable='datalim')
    plt.xlim(-2, 2)
    plt.ylim(-2, 2)
    plt.grid()
    plt.show()
         
if __name__ == "__main__":
    r_intersect = find_intersection(r_A2_O_O, r_B2_O_O, LA3, LB3)
    print("Intersection point:", r_intersect)
    plot_circles(r_A2_O_O, LA3, r_B2_O_O, LB3,r_intersect)


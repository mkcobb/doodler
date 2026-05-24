import numpy as np
import matplotlib.pyplot as plt

# Notes on rough dimensions:
# Assuming the paper is 11 inches mounted on a circular rotating platform, the minimum radius of the platform needs to be about 7.7 inches to ensure the entire paper is on there.
# In order to give some wiggle room so we can shift the paper around to be off-center a little, make the platform radius 10 inches.  
# This means the minimum values of LA1 and LB1 are 10 inches.
# LA3 + LB3 must be greater than LA2 + LB2 + distance between fixed arm end points for feasibility.
# However, if LA3 + LB3 = LA2 + LB3 + dist btwn fixed arm end points, then the pen will only be able to reach one point on the paper.
# So we need LA3 + LB3 to be sufficiently larger than LA2 + LB2 + distance between fixed arm end points to give us a good range of motion for the pen. 

# Motor Parameters:
# The expression for motor speed is omega = c1 + c2*sin(c3*t + c4) where c1, c2, c3, and c4 are the parameters for each motor.

dt      = 0.01 # Seconds
t_final = 600   # Seconds

LA1 = 14 # Length of arm 1 for motor A in inches
LA2 = 4  # Length of arm 2 for motor A in inches
LA3 = 16 # Length of arm 3 for motor A in inches
thetaA1        = np.deg2rad(90) # Angle of arm 1 for motor A in radians
thetaA2_init   = np.deg2rad(90) # Initial angle of arm 2 for motor A in radians.
Motor_A_params = np.array([np.deg2rad( 10 ),np.deg2rad(0),np.deg2rad(0.1) ,0])  

LB1 = 12 # Length of arm 1 for motor B in inches
LB2 = 1  # Length of arm 2 for motor B in inches   
LB3 = 15 # Length of arm 3 for motor B in inches
thetaB1        = np.deg2rad(0)     # Angle of arm 1 for motor B in radians
thetaB2_init   = np.deg2rad(-180)  # Initial angle of arm 2 for motor B in radians.
Motor_B_params = np.array([np.deg2rad( 60 ),0,np.deg2rad(0.1) ,0]) 

Motor_T_params = np.array([np.deg2rad( 1 ),0,0     ,0]) 

r_A1_O_O = np.array([LA1*np.cos(thetaA1), LA1*np.sin(thetaA1)]) # Position of end of arm 1 for motor A in O frame
r_B1_O_O = np.array([LB1*np.cos(thetaB1), LB1*np.sin(thetaB1)]) # Position of end of arm 1 for motor B in O frame

d_A1_B1 = np.linalg.norm(r_A1_O_O - r_B1_O_O) # Distance between the end of arm 1 for motor A and motor B in inches

print(f"Distance between arm 1 endpoints: {d_A1_B1:.2f} inches")
print(f"Minimum total arm 3 lengths     : {d_A1_B1 + LA2 + LB2:.2f} inches")
print(f"Actual total arm 3 lengths      : {LA3 + LB3:.2f} inches")


def find_arm_endpoint(L1, L2, theta1, theta2):
    r1 = np.array([L1*np.cos(theta1), L1*np.sin(theta1)])
    r2 = np.array([L2*np.cos(theta2), L2*np.sin(theta2)])
    return r1 + r2 

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

def plot_motor_angles(t, thetaA2, thetaB2, thetaT):
    plt.figure(figsize=(10,6))
    plt.plot(t, np.rad2deg(thetaA2), label='Theta A2 (degrees)', linewidth=0.1)
    plt.plot(t, np.rad2deg(thetaB2), label='Theta B2 (degrees)', linewidth=0.1)
    plt.plot(t, np.rad2deg(thetaT), label='Theta T (degrees)', linewidth=0.1)
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.title('Motor Angles Over Time')
    plt.legend()
    plt.grid()

def plot_spirograph(ri_O_O, ri_O_T):
    fig, ax = plt.subplots(1,2, figsize=(12,6))
    ax[0].plot(ri_O_O[:,0], ri_O_O[:,1], 'r', label='Intersection in O frame', linewidth=0.1)
    ax[1].plot(ri_O_T[:,0], ri_O_T[:,1], 'b', label='Intersection in T frame', linewidth=0.1)
    ax[0].plot(0, 0, 'ko', label='Origin O')
    ax[1].plot(0, 0, 'ko', label='Origin O')
    ax[0].plot([5.5, 5.5, -5.5, -5.5, 5.5], [-5.5, 5.5, 5.5, -5.5, -5.5], 'k', label='Paper Boundary', linewidth=0.5)
    ax[1].plot([5.5, 5.5, -5.5, -5.5, 5.5], [-5.5, 5.5, 5.5, -5.5, -5.5], 'k', label='Paper Boundary', linewidth=0.5)
    ax[0].legend()
    ax[1].legend()
    ax[0].set_xlabel('X')
    ax[1].set_xlabel('X')
    ax[0].set_ylabel('Y')
    ax[1].set_ylabel('Y')
    ax[0].set_title('O Frame Spirograph')
    ax[1].set_title('T Frame Spirograph')
    ax[0].axis('equal')
    ax[1].axis('equal')
    ax[0].grid()
    ax[1].grid()
    

def step_motor(theta,t,dt,motor_params):
    # Steps motor position forwards in time by 1 time step, dt
    
    # Calculate motor speed
    omega = \
        motor_params[0] + \
        motor_params[1]*np.sin(motor_params[2]*t + motor_params[3])  
    
    # Integrate speed to get new angle
    return theta + omega * dt

def simulate(r_A1_O_O, r_B1_O_O, LA2, LB2,LA3,LB3,
             t_final,thetaA2_init,thetaB2_init,
             A_motor_params,B_motor_params,T_motor_params):

    # Initialize time vector and number of time steps
    t  = np.arange(0, t_final, dt)
    nt = len(t)

    # Initialize motor angle arrays
    thetaA2 = np.zeros(nt)
    thetaB2 = np.zeros(nt)
    thetaT  = np.zeros(nt)

    # Initialize intersection point arrays
    ri_O_O = np.zeros((nt, 2))
    ri_O_T = np.zeros((nt, 2))

    # Set initial motor angles
    thetaA2[0] = thetaA2_init
    thetaB2[0] = thetaB2_init
    thetaT[0]  = 0.0

    # Flag to indicate if the simulation failed (e.g., no intersection found)
    failed = False

    # Find the initial intersection point at t=0
    r_A2_O_O    = find_arm_endpoint(LA1, LA2, thetaA1, thetaA2[0])
    r_B2_O_O    = find_arm_endpoint(LB1, LB2, thetaB1, thetaB2[0])
    ri_O_O_temp = find_intersection(r_A2_O_O, r_B2_O_O, LA3, LB3)

    if ri_O_O_temp is None:
        # No solution, return nothing
        failed = True
        return failed , t , thetaA2, thetaB2, thetaT, ri_O_O , ri_O_T
    
    ri_O_O[0,:] = ri_O_O_temp

    # Compute the initial position of the intersection point in the T frame
    R_T_O = np.array([[np.cos(thetaT[0]), -np.sin(thetaT[0])],
                      [np.sin(thetaT[0]),  np.cos(thetaT[0])]])
    ri_O_T[0,:] = R_T_O @ ri_O_O[0,:]

    # Loop over all time steps in the simulation
    for ii in range(int(1),int(nt)):
        # Update time and motor angles
        thetaA2[ii] = step_motor(thetaA2[ii-1],t[ii],dt,A_motor_params)
        thetaB2[ii] = step_motor(thetaB2[ii-1],t[ii],dt,B_motor_params)
        thetaT[ii]  = step_motor(thetaT[ii-1] ,t[ii],dt,T_motor_params)

        # Find the new intersection point in the O frame
        r_A2_O_O = find_arm_endpoint(LA1, LA2, thetaA1, thetaA2[ii])
        r_B2_O_O = find_arm_endpoint(LB1, LB2, thetaB1, thetaB2[ii])
        ri_O_O_temp   = find_intersection(r_A2_O_O, r_B2_O_O, LA3, LB3) 

        # If no intersection is found, the simulation has failed
        if ri_O_O_temp is None:
            # No solution, return nothing
            failed = True
            return failed , t , thetaA2, thetaB2, thetaT, ri_O_O , ri_O_T
        
        ri_O_O[ii,:] = ri_O_O_temp

        # Compute the position of the intersection point in the T frame
        R_T_O = np.array([[np.cos(thetaT[ii]), -np.sin(thetaT[ii])],
                          [np.sin(thetaT[ii]),  np.cos(thetaT[ii])]])
        ri_O_T[ii,:] = R_T_O @ ri_O_O[ii,:]
        
    return failed , t , thetaA2, thetaB2, thetaT, ri_O_O, ri_O_T

if __name__ == "__main__":
    r_A1_O_O = np.array([LA1*np.cos(thetaA1), LA1*np.sin(thetaA1)])
    r_B1_O_O = np.array([LB1*np.cos(thetaB1), LB1*np.sin(thetaB1)])

    failed , t , thetaA2, thetaB2, thetaT, ri_O_O , ri_O_T = simulate(
        r_A1_O_O, r_B1_O_O, 
        LA2, LB2 , LA3 , LB3 ,
        t_final , thetaA2_init , thetaB2_init ,
        Motor_A_params , Motor_B_params , Motor_T_params)

    if failed:
        print("Simulation failed: No intersection found.")
    else:
        plot_spirograph(ri_O_O, ri_O_T)
        plot_motor_angles(t, thetaA2, thetaB2, thetaT)
        plt.show()
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import acos, asin, atan2, pi, cos, sin
import numpy as np
import copy

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

# Add labels
plt.title('Robot Dog Leg')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(-0.15,0.1)
ax.set_ylim(-0.15,0.1)
ax.set_zlim(-0.15,0.1)
# ax.set_xlim(-0.15, 0.15)
# ax.set_ylim(-0.15, 0.15)
# ax.set_zlim(-0.15, 0.15)

start = [0,0,0]

center_left = [0,0,0]
center_right = [0,0,0]
center_bottom = [0,0,0]
end = [0,0,0]

theta1 = 0
theta2 = 0
theta3 = 0
theta4 = 0

goal = [0.00,0.0245,-0.12]

a = 0.1059
d = 0.0245
c = 0.11058
b = 0.047434
e = 0.063725
de = 0.11472
s = 0.038

def dist(pts1, pts2):
    sum = 0
    for i in range(min(len(pts1),len(pts2))):
        sum += (pts1[i]-pts2[i])**2

    return sum ** (1/2)

def IK_Leg(targetX, targetY, targetZ):
    beta_0_prime = atan2(targetZ, targetY)
    A_prime_2 = dist((targetY,targetZ), (0,0))
    targetZ = -(A_prime_2**2 - s**2)**(1/2)
    
    beta_0 = acos(s/A_prime_2)
    
    if beta_0_prime < 0:
        beta_0_prime += 2 * pi
    theta_4 = beta_0_prime + beta_0 - 2 * pi
      
    
    A = dist((targetX,targetZ), (0,0))

    beta_1 = atan2(targetZ,targetX)
    if beta_1 < 0:
        beta_1 += 2 * pi

    beta_2 = acos(((b+e)**2 - a**2 - A**2)/(-2*a*A))

    theta_1 = beta_1 - pi - beta_2

    beta_3 = acos((A**2 - a**2 - (b+e)**2)/(-2*(b+e)*(a)))

    theta_3 = beta_3 - theta_1

    A_prime = (a**2 + b**2 - 2*a*b*cos(beta_3))**(1/2)
    beta_4 = acos((c**2 - d**2 - A_prime**2)/(-2*d*A_prime))

    beta_lost = acos((e**2 - A**2 - A_prime**2)/(-2*A*A_prime))

    theta_2 = 2 * pi - beta_1 - beta_4 + beta_lost
    
    return theta_1 * 180/pi, theta_2 * -180/pi, theta_3 * 180/pi, theta_4 * 180/pi

# for x in range(-5,10):
#     for y in range(-10,10):
#         for z in range(-10,0):
#             try:
#                 #print(float(x)/600,float(z)/600)
#                 t1, t2, t3, t4 = IK_Leg(float(x)/50,float(y)/50,float(z)/50)
#                 if t1 < -90 or t1 > 90 or t2 > 90 or t2 < -30 or t4 > 70 or t4 < -70:
#                     continue
#                 ax.scatter(float(x)/50,float(y)/50,float(z)/50, color = 'r')
                
#             except:
#                 continue

time = 0
dt = 0.1

def animate(i):
    global time
    global goal
    global theta1, theta2, theta3, theta4
    
    oldgoal = copy.deepcopy(goal)
    speed = 4*sin(0.04*time)
    goal[0] = -0.035*cos(speed*time)+0.01
    goal[1] = 0.01*sin(speed*time) + s
    goal[2] = 0.01*sin(speed*time) - 0.12

    ax.scatter(goal[0], goal[1], goal[2], c = 'b')

    time += dt

    # clear figure
    # plt.cla()
    # plt.title('Robot Dog Leg Sim')
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')

    #ax.scatter(start[0],start[1],start[2], color = 'y')
    #ax.scatter(goal[0],goal[1],goal[2], color = 'g')
    #ax.scatter(start[0],start[1],start[2], color = 'y')
    #print(theta1,theta2,theta3)
    try:
        theta1, theta2, theta3, theta4 = IK_Leg(goal[0],goal[1],goal[2])
        if theta1 < -90 or theta1 > 90 or theta2 > 90 or theta2 < -30 or theta4 > 70 or theta4 < -70:
            raise ValueError
    except:
        goal = oldgoal
        theta1, theta2, theta3, theta4 = IK_Leg(goal[0],goal[1],goal[2])
    print(theta1,theta2,theta3,theta4)

    start[1] = s * cos(theta4*pi/180)
    start[2] = s * sin(theta4*pi/180)

    # calculate new key points
    center_left[0] = start[0] + a * -cos(theta1*pi/180)
    center_left[1] = start[1] - cos(theta4*pi/180 + 3*pi/2) * (a * -sin(theta1*pi/180))
    center_left[2] = start[2] - sin(theta4*pi/180 + 3*pi/2) * (a * -sin(theta1*pi/180))
    #print(center_left)
    #ax.scatter(center_left[0],center_left[1],center_left[2], color = 'r')

    center_right[0] = start[0] + d * cos(theta2*pi/180)
    center_right[1] = start[1] - cos(theta4*pi/180 + 3*pi/2) * (d * +sin(theta2*pi/180))
    center_right[2] = start[2] - sin(theta4*pi/180 + 3*pi/2) * (d * +sin(theta2*pi/180))
    #print(center_right)
    #ax.scatter(center_right[0],center_right[1],center_right[2], color = 'r')
    
    center_bottom[0] = center_left[0] + b * cos(theta3*pi/180)
    center_bottom[1] = start[1] -cos(theta4*pi/180 + 3*pi/2) * (center_left[2] - b * sin(theta3*pi/180))
    center_bottom[2] = start[2] -sin(theta4*pi/180 + 3*pi/2) * (center_left[2] - b * sin(theta3*pi/180))
    #print(center_bottom)
    #ax.scatter(center_bottom[0],center_bottom[1],center_bottom[2], color = 'y')

    end[0] = center_bottom[0] + e * cos(theta3*pi/180)
    end[1] = start[1] -cos(theta4*pi/180 + 3*pi/2) * (center_bottom[2] - e *sin(theta3*pi/180))
    end[2] = start[2] -sin(theta4*pi/180 + 3*pi/2) * (center_bottom[2] - e *sin(theta3*pi/180))
    #ax.scatter(end[0],end[1],end[2], color = 'b')

    line0, = ax.plot([0, start[0]],
                     [0, start[1]],
                     [0, start[2]],
                     color = 'g')

    line1, = ax.plot([start[0], center_left[0]],
                     [start[1], center_left[1]],
                     [start[2], center_left[2]],
                     color ='g')
    
    line2, = ax.plot([start[0], center_right[0]],
                     [start[1], center_right[1]],
                     [start[2], center_right[2]],
                     color ='g')
    
    line3, = ax.plot([center_left[0], center_bottom[0]],
                     [center_left[1], center_bottom[1]],
                     [center_left[2], center_bottom[2]],
                     color ='g')
    
    line4, = ax.plot([center_right[0], center_bottom[0]],
                     [center_right[1], center_bottom[1]],
                     [center_right[2], center_bottom[2]],
                     color ='g')
    
    line5, = ax.plot([center_bottom[0], end[0]],
                     [center_bottom[1], end[1]],
                     [center_bottom[2], end[2]],
                     color ='g')


    return line0,line1,line2,line3,line4,line5,

ani = animation.FuncAnimation(fig,
    animate,
    fargs=(),
    interval=50,
    blit=True)

plt.show()
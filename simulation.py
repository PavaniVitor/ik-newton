import numpy as np
from numpy.linalg import pinv, norm
from Robot import Robot
import matplotlib.pyplot as plt 



def plot_robot(robot, target):
    # call plt plots
    x_arm, y_arm = robot.arm.get_end_pos()
    x = [robot.arm.position[0] , x_arm]
    y = [robot.arm.position[1] , y_arm]
    
    plt.plot(x, y, color='b')

    x_wrist, y_wrist = robot.wrist.get_end_pos()

    x = [robot.wrist.position[0] , x_wrist]
    y = [robot.wrist.position[1] , y_wrist]
    
    plt.plot(x, y, color='r')
    plt.scatter(target[1], target[0], color='g')

    plt.xlim(0, 3)
    plt.ylim(0, 3)
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.show()


def main(x, y):
    
    robot = Robot((0,0), 2, np.pi/4, 1, np.pi/4) # robot initial position
    
    tolerance = 1e-5 

    y_guess = np.array([robot.arm.rotation, robot.wrist.rotation]) # initial theta and phi
    y_new = np.array([0,0])

    F = np.copy(y_guess)

    error = 10e9

    target = (y, x)
    plot_robot(robot, target)

    errors = []


    # Solve loop
    #y_new = y_guess -alpha*inv(J)*F
    while norm(error) > tolerance:
        F[0] = robot.fx(y_guess[0], y_guess[1])
        F[1] = robot.fy(y_guess[0], y_guess[1])
        
        J = robot.Jacobian()

        #y_new = y_guess - alpha * np.matmul(pinv(J), F)
        error = np.array(target) - F
        
        
        y_new = y_guess + (pinv(robot.Jacobian()) @ (error))
        
        y_guess = y_new

        robot.rotate(y_new[0] , y_new[1])
        
        errors += [norm(error)]

        print(y_new)

        print(norm(error))

        
        plot_robot(robot, target)
        
    print (f"target: {target}")
    print(f" func: {F}")
    print(f"pos: {robot.wrist.get_end_pos()}")
    

    plt.plot(errors)
    
    plt.xlabel('Iteração')
    plt.ylabel('Erro')
    
    plt.show()


if __name__ == "__main__":
    main(2.1, 1.7)

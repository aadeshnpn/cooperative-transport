#import matplotlib.pyplot as plt
import cv2
import numpy as np
import matplotlib.pyplot as plt
#Goal = 390,-50
#Object = 100, 410

def get_slope(p1, p2):
    #print (p1,p2)
    try:
        return ((p2[1]-p1[1])*1.0) / (p2[0]-p1[0])
    except ZeroDivisionError:
        return np.NaN

def get_c(m, p1):
    return p1[1] - m*p1[0]

def plot_line(p1, p2):
    m = get_slope(p1, p2)
    c = get_c(m, p1)
    #print (list(range(p1[0], p2[0])))
    #print (list(range(p1[0
    # ], p2[0])))
    if p1[0] < p2[0]:
        x = np.array([*range(p1[0], p2[0])], dtype=np.int)
    else:
        x = np.array([*range(p2[0], p1[0])], dtype=np.int)
    y = m * x + c
    y = y.astype(np.int)
    
    plt.plot(x,y)
    #plt.show()
    return c, m

def cnvtcord(x1,y1):
    return (x1+500, y1+500)

def main():
    """
    p1 = cnvtcord(390,-50)
    p2 = cnvtcord(100, 410)
    p3 = cnvtcord(-200, -310)

    c, m = plot_line(p1, p2)
    y = m*p3[0] + c
    
    p4 = cnvtcord(-200, y)

    plot_line(p3, p4)

    plt.show()
    """
    img = np.zeros((1000, 1000, 3))
    
    goal = cnvtcord(315,-62)
    item = cnvtcord(213,142)
    vp = cnvtcord(177,214)
    robot = cnvtcord(-5,-14)

    cv2.line(img, item, goal, (0,255,0), 5) #green 
    cv2.line(img, robot, vp, (255,0,0), 5) #red
    cv2.circle(img, robot,10,(255,255,255)) #white
    cv2.circle(img, item,10,(0,0,255)) #blue
    cv2.circle(img, goal,10,(123,0,123)) #purple
    cv2.circle(img, vp,10,(0,123,123)) #light blue            
    flip = img[::-1,:,:]
    while True:
        cv2.imshow("flip", flip[:,:,::-1])
        if cv2.waitKey(10) & 0xFF == ord('q'):
            exit()

if __name__ == '__main__':
    main()
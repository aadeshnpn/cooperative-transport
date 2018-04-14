## Go to
import numpy as np
import cv2

class PIctrl:
    def __init__(self, kp = 5, ki = 0.01, kd=0.01, integratorb=(-200, 200), actual=(0,0,0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ibound = integratorb
        self.error = 0
        self.ref_dyna = np.ones((2,1)) * 0.1
        self.ref_point = np.ones((3,1))
        self.actual = actual
        self.derivator = 0
        self.integrator = 0
        self.robot = None
        self.E_k = 0
        self.e_k_1 = 0
    
    def update(self, dt):
        #theta = self.actual[2]
        #self.rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        #self.rotation = np.reshape(self.rotation, (2,2))
        v = 200 #calc_vw(self.robot.right_wheel_speed.speed_mmps, self.robot.left_wheel_speed.speed_mmps)
        diff = self.ref_point - self.actual
        theta_g = np.arctan2(diff[1][0],diff[0][0])
        e_k = theta_g - self.actual[2]
        e_k = np.arctan2(np.sin(e_k), np.cos(e_k))
        e_P = e_k
        #self.error = e_k

        #self.error = np.dot(self.rotation, diff[:2])
        #self.error = diff[:2]
        #self.error = np.linalg.norm(self.error)
        ## PID start
        e_P = e_k
        e_I = self.E_k + e_k*dt
        e_D = (e_k-self.e_k_1)/dt
        w = self.kp * e_P + self.ki*e_I + self.kd*e_D
        self.E_k = e_I
        self.e_k_1 = e_k
        
        #self.p = self.kp * self.error
        #self.d = self.kd * (self.error -self.derivator)
        #self.derivator = self.error
        #self.integrator = self.integrator + self.error
        #self.integrator = self.bound_values(self.integrator)
        #self.i = self.ki * self.integrator
        #self.pid = self.p + self.d + self.i
        #v = self.kp * np.linalg.norm(diff)
        #v = np.linalg.norm(self.pid) * self.kd
        #v = self.bound_values(self.pid)
        
        #w = np.arctan2(diff[1][0], diff[0][0])
        #w = self.pid
        #print (v, w)
        return (v, w, e_P)

    def update3(self):
        diff = self.ref_point - self.actual
        self.error = diff[:2] #/ diff[2]
        v = self.kp * np.linalg.norm(diff)
        w = np.arctan2(self.error[1][0], self.error[0][0])
        return (v, w, self.error)        

    #def update(self):


    def update9(self):
        C = np.array([[np.cos(self.ref_point[2] - self.actual[2])], [1]])
        print ('ref,actual',self.ref_point, self.actual)
        u1 = -self.kp * (self.ref_point[0]-self.actual[0])
        t1 = self.kp * self.ref_dyna[0] * np.sinc(self.ref_point[2]-self.actual[2])
        t2 = self.ref_point[1]-self.actual[1]
        t3 = -self.kd * (self.ref_point[2]-self.actual[2]) 
        u2 = t1*t2+t3
        ctrl = C * self.ref_dyna - np.array([[u1[0]],[u2[0]]])
        
        theta = self.actual[2]
        self.rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        diff = self.ref_point - self.actual
        error = np.linalg.norm(diff)
        return ctrl
    
    def pos_update(self, pose):
        self.actual = pose

    def bound_values(self, val):
        val[val >= self.ibound[1]] = self.ibound[1]
        val[val < self.ibound[0]] = self.ibound[0]        
        return val
        #if val < self.ibound[0]:
        #    val = self.ibound[0]
        #elif val >= self.ibound[1]:
        #    val = self.ibound[1]
        #return val

def to_nppose(pos, angle):
    object_pos = np.array([[pos[0]],[pos[1]],[angle]])
    return object_pos


def calc_wheel_velo(velocity):
    R = 12.7 # mm
    L = 53.975 #mm
    #vd, wd = velocity[0][0], velocity[1]
    vd, wd = velocity[0], velocity[1]    
    vr = (2*vd + wd*L) / (2*R)
    vl = (2*vd - wd*L) / (2*R)

    return vr[0], vl[0]

def calc_vw(vr, vl):
    R = 12.7 # mm
    L = 53.975 #mm
    #vd, wd = velocity[0][0], velocity[1]
    #vd, wd = velocity[0], velocity[1]    
    #vr = (2*vd + wd*L) / (2*R)
    #vl = (2*vd - wd*L) / (2*R)
    v = (R/2.0) * (vr + vl) 
    #w = (R/2.0) * (vr - vl)
    #return np.array([[v], [w]])
    return v

def get_direction(p1, p2):
    err = p1[:2] - p2[:2]
    angle = np.arctan2(err[1][0], err[0][0])
    return angle

def get_slope(p1, p2):
    try:
        return ((p2[1]-p1[1])*1.0) / (p2[0]-p1[0])
    except ZeroDivisionError:
        return np.NaN

def get_c(m, p1):
    return p1[1] - m*p1[0]

def get_mc(p1, p2):
    m = get_slope(p1, p2)
    c = get_c(m , p1)
    return m, c

def plan_virtualpoint(goal, item, robot):
    m, c = get_mc(goal, item)
    vy = item[1] + 300
    vx = ( vy - c ) / m
    #vx = robot[0] + 400
    #vy = m*vx + c
    #vx = goal[0] - 200
    #vy = goal[1] - 200
    vp = (vx+400, vy)
    return vp

def cnvtcord(p):
    return (p[0][0]+500, p[1][0]+500)

def visualize_path(g,i,v,r):
    img = np.zeros((1000, 1000, 3))
    
    #goal = cnvtcord(315,-62)
    #item = cnvtcord(213,142)
    #vp = cnvtcord(177,214)
    #robot = cnvtcord(-5,-14)
    g = g.astype(np.int)
    i = i.astype(np.int)    
    v = v.astype(np.int)        
    r = r.astype(np.int)        
    goal = cnvtcord(g)
    item = cnvtcord(i)
    vp = cnvtcord(v)
    robot = cnvtcord(r)    

    cv2.line(img, item, goal, (0,255,0), 5) #green 
    cv2.line(img, robot, vp, (255,0,0), 5) #red
    cv2.circle(img, robot,10,(255,255,255),4) #white
    cv2.circle(img, item,10,(0,0,255),4) #blue
    cv2.circle(img, goal,10,(123,0,123),4) #purple
    cv2.circle(img, vp,10,(0,123,123),4) #light blue            
    #flip = img[::-1,:,:]
    return img
    #while True:
    #    cv2.imshow("flip", flip[:,:,::-1])
    #    if cv2.waitKey(10) & 0xFF == ord('q'):
    #        exit()

def add_obj_path(img, p1):
    p1 = p1.astype(np.int)        
    rob = cnvtcord(p1)
    cv2.circle(img, rob, 10,(123,123,0),8) #robo 
    return img

def calc_actual_v(robot):
    return robot.left_wheel_speed.speed_mmps+robot.right_wheel_speed.speed_mmps

def main():
    ctrl = PIctrl()
    ctrl.ref_point = np.array([[10],[10],[0.45]])
    ctrl.actual = (0,0,0.23)
    i = 0
    x, y = 0 , 0
    v, w = 0 , 0
    while i < 4500:
        i += 1
        res = ctrl.update()
        #print ('volate', res)
        #vr, vl = calc_wheel_velo(res)
        x += res[0]*np.cos(w)
        y += res[0]*np.sin(w)
        w = res[1][0]
        #x = (res/2.0) + np.random.randn()*2
        #y = (i/2.0) + np.random.randn()*2
        ctrl.pos_update((x,y,0))
        print (i, res, np.linalg.norm(res), x, y)
    

if __name__ == '__main__':
    main()

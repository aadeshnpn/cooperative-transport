## Go to
import numpy as np
import cv2
#import cv2.aruco as aruco

#aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
#parameters =  aruco.DetectorParameters_create()

#params = np.load("calib.npz")
#camera_matrix = params['mtx']
#dist_coeffs = params['dist']

#markerLength = 0.009
#axis = np.float32([[2,0,0], [0,2,0], [0,0,-2]]).reshape(-1,3)

lk_params = dict(winSize  = (21, 21),
                #maxLevel = 3,
                 criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

def featureTracking(image_ref, image_cur, px_ref):
    kp2, st, err = cv2.calcOpticalFlowPyrLK(image_ref, image_cur, px_ref, None, **lk_params)  #shape: [k,2] [k,1] [k,1]
    try:
        st = st.reshape(st.shape[0])
        kp1 = px_ref[st == 1]
        kp2 = kp2[st == 1]

        #u = (kp2 - kp1)
        #vel = np.linalg.norm(u)
        #return kp1, kp2, vel
        return kp1, kp2
    except AttributeError:
        return None, None

##def get_rt(image):
#    corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict, parameters=parameters)
#    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs) # posture estimation from a single marker
#    return rvec, tvec, corners


class PIctrl:
    def __init__(self, kp = 8, ki = 0.01, kd=0.01, integratorb=(-200, 200), actual=(0,0,0)):
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
        v = 300 #calc_vw(self.robot.right_wheel_speed.speed_mmps, self.robot.left_wheel_speed.speed_mmps)
        diff = self.ref_point - self.actual
        theta_g = np.arctan2(diff[1][0],diff[0][0])
        e_k = theta_g - self.actual[2]
        e_k = np.arctan2(np.sin(e_k), np.cos(e_k))
        e_P = e_k
        ## PID start
        e_P = e_k
        e_I = self.E_k + e_k*dt
        e_D = (e_k-self.e_k_1)/dt
        w = self.kp * e_P + self.ki*e_I + self.kd*e_D
        self.E_k = e_I
        self.e_k_1 = e_k
        return (v, w, e_P)
    
    def update_theta(self, dtheta):
        v = 200
        e_k = dtheta - self.actual[2]
        e_k = np.arctan2(np.sin(e_k), np.cos(e_k))
        w = self.kp * e_k
        return (v, w, e_k)

    def pos_update(self, pose):
        self.actual = pose

    def bound_values(self, val):
        val[val >= self.ibound[1]] = self.ibound[1]
        val[val < self.ibound[0]] = self.ibound[0]        
        return val

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
    add = np.random.choice([40,60,80,100])
    vy = item[1] + 250 + add
    vx = ( vy - c ) / m
    #vx = robot[0] + 400
    #vy = m*vx + c
    #vx = goal[0] - 200
    #vy = goal[1] - 200
    vp = (vx+200+add, vy)
    return vp

def cnvtcord(p):
    return (p[0][0]+200, p[1][0]+200)

def visualize_path(g,i,v,r):
    img = np.zeros((1000, 1900, 3))
    
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

def convertc(p):
    x = int(p[0]) + 400
    y = int(p[1]) + 400
    return (x,y)

def visualize_objects(r1,r2,goal):
    r1 = convertc(r1)
    r2 = convertc(r2)    
    goal = convertc(goal)

    img = np.zeros((1000, 1900, 3))
    cv2.circle(img, r1,10,(255,255,255),4) #white
    cv2.circle(img, r2,10,(0,0,255),4) #blue
    cv2.circle(img, goal,10,(123,0,123),4) #purple
    #cv2.circle(img, vp,10,(0,123,123),4) #light blue
    return img

def compute_features(box, img):
    x1,y1 = int(box.left_x), int(box.bottom_y)
    x2,y2 = int(box.right_x), int(box.top_y)
    w,h= int(box.width), int(box.height)    
    cv2.rectangle(img,(x1,y1),(x2,y2),(255,255,0),5)

    mask_image = img.copy()
    #mask_image[:,:] = 255
    #start_point = (x1, y1)
    #end_point = (x2, y2)
    
    #mask_image[y2:y2+h, x1:x1+w] = img[y2:y2+h, x1:x1+w]
    detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
    p0 = detector.detect(mask_image)
    p0 = np.array([x.pt for x in p0], dtype=np.float32)
    return p0, mask_image
    #kpimg = cv2.drawKeypoints(mask_image, p0, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)    

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

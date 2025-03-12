from os import PRIO_PGRP
import rospy
import math
from mavros_msgs.srv import CommandBool
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range
import tf
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
import numpy as np
from geometry_msgs.msg import Vector3Stamped, Point, PointStamped
# всякие говно либы для посадки. если забудешь, можно на сайте tf найти их легко

rospy.init_node('auto_land')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool) # дизарм
set_effect = rospy.ServiceProxy('led/set_effect', srv.SetLEDEffect)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)

ar = "aruco_150" # аруко метка на пылесосе/куда садиться
tfBuffer = tf2_ros.Buffer() # запомнить метку в буфер обмена
listener = tf2_ros.TransformListener(tfBuffer) # запрос и получения координат аруко метки с буфера обмена

l_z = 2

# эт херня которая лазерный дальномер забыл как точно называется
def range_callback(msg):
    global l_z
    l_z = msg.range

rospy.Subscriber('rangefinder/range', Range, range_callback)

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def navigate_wait_fixed(x=0, y=0, z=0, yaw=float('nan'), speed=0.2, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    global platform_found
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        platform_found = False
        platform = get_telemetry(frame_id=ar)
        if str(platform.x) != 'nan':
            print('[DEBUG] found platform')
            platform_found = True
            break

        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

# получить координаты aruco по x,y,z
def get_aruco_pose(frame_id):
    global tfBuffer, listener

    try:
        trans = tfBuffer.lookup_transform(frame_id, ar, rospy.Time())
    except:
        return None
    pnt_l0 = tf2_geometry_msgs.do_transform_point(PointStamped(point=Point(x=0, y=0, z=0)), trans)
    l0 = np.array([pnt_l0.point.x, pnt_l0.point.y, pnt_l0.point.z])
    return l0

# получить координаты самого дрона по body в x,y,z
def get_body_pose(frame_id):
    global tfBuffer, listener

    try:
        trans = tfBuffer.lookup_transform(frame_id, "body", rospy.Time())
    except:
        return None
    pnt_l0 = tf2_geometry_msgs.do_transform_point(PointStamped(point=Point(x=0, y=0, z=0)), trans)
    l0 = np.array([pnt_l0.point.x, pnt_l0.point.y, pnt_l0.point.z])
    return l0

def remove_0_vel(vel):
    if np.linalg.norm(vel[:2]) < 0.045:
        vel[0] = 0
        vel[1] = 0
    return vel

def disarm():
    arming(False)
    print("[DEBUG] DISARMED")
    set_effect(r=255, g=0, b=0)

def main():
    z = 1.7

    navigate_wait_fixed(z=1.7, speed=1, frame_id="body", auto_arm = True)
    rospy.sleep(2)
    set_effect(r=255, g=255, b=0)
    print("[DEBUG] started navigation")
    
    # навигация по маркерам, путь. вообще тут говнокод если честно, можно просто в цикл поместить и по x,y уже следовать
    # ну или копировать значения с самой карты маркеров и тоже помещать в цикл... но на турике никому не нужен такой код ес честно
    navigate_wait(x=0, y=0, z=z, speed=0.7, frame_id='aruco_map')
    
    # Implement lawnmower pattern flight while checking for platform
    direction_forward = True
    for x in range(0, 4, 1):  # Adjust range based on your area size
        if platform_found:  # Check if platform is found before each movement
            break
            
        if direction_forward:
            # Move along increasing y
            navigate_wait_fixed(x=x, y=0, z=z, speed=0.8, frame_id='aruco_map')
            if platform_found:
                break
            navigate_wait_fixed(x=x, y=5, z=z, speed=0.8, frame_id='aruco_map')
        else:
            # Move along decreasing y
            navigate_wait_fixed(x=x, y=5, z=z, speed=0.8, frame_id='aruco_map')
            if platform_found:
                break
            navigate_wait_fixed(x=x, y=0, z=z, speed=0.8, frame_id='aruco_map')
            
        direction_forward = not direction_forward  # Switch direction for next iteration
        
    set_effect(r=0, g=0, b=255)
    print("[DEBUG] navigation to platform")

    # If platform is found, proceed with landing sequence
    if platform_found:
        print("start")
        navigate_wait(x=0, y=0, z=z, speed=0.7, frame_id=ar, yaw=float('nan'), tolerance=0.2)
        print("ended")

        FRQ = 10 # количество апдейтов в секунду или сколько там уже не помню. 10 проверок в секунду. чем больше - дрон будет немного сильнее дергаться
        # ибо он же получает постоянно коордианты аруко с get_aruco_pose и при land может повлиять, да и вообще сама камера будет медленее в listener доходить
        r = rospy.Rate(FRQ)
        prev_vel = None
        prev_pa = None
        prev_t = rospy.get_time()
        st_t = rospy.get_time()
        d = 10
        while d > 0.08 or (rospy.get_time() - st_t < 0.5):
            pb = get_body_pose("aruco_map")
            pa = get_aruco_pose("aruco_map")
            now = rospy.get_time()
            if prev_pa is None: # если не видно aruco метку с пылесоса, на практике такого почти ни разу не было (но всегда минимум 1 раз будет ибо prev_pa = None), после этого нет
                if pb is None: # если не получил x,y,z с дрона - не будет садиться. может быть такое когда raspberry перегрета сильно :/
                    r.sleep()
                    continue
                navigate(x=pb[0], y=pb[1], z=z, speed=0.8, frame_id="aruco_map") # двигаемся по body, ибо не видим aruco пылесоса
                set_effect(r=150, g=0, b=255, effect='blink')
                prev_pa = pa
                prev_t = now
            else:
                if pb is not None:
                    d = np.linalg.norm(pb[:2]-pa[:2]) # https://sparrow.dev/numpy-norm/
                if pa is not None:
                    set_effect(r=150, g=0, b=255)
                    vel = (pa-prev_pa)/(now-prev_t+0.00001) # тут можно тестить значения самих цифр
                    vel = np.clip(vel, -0.7, 0.7) # тут можно тестить значения самих цифр
                    vel = remove_0_vel(vel) 
                    if prev_vel is not None:
                        vel = vel*0.9 + prev_vel*0.1 # тут можно тестить значения самих цифр
                        
                    t = pa[:2] + vel[:2]*(1.0/FRQ)*2.7
                    # если кратко, то весь этот цикл следует за аруко меткой НО ПОКА ЕЩЕ НЕ САДИТСЯ, он просто пытается в ровень по координатам подлететь
                    # к самой аруко на пылесосе, после того как он посчитает уже ровно к самой аруке, то он будет фиксировать эту позицию по координатам через set_position
                    set_position(x=t[0], y=t[1], z=z, frame_id="aruco_map")
                    prev_pa = pa.copy()
                    prev_vel = vel.copy()
                    prev_t = now
                else:
                    navigate(x=pb[0], y=pb[1], z=z, frame_id="aruco_map")
                    set_effect(r=150, g=0, b=255, effect='blink_fast')
                    print("NO pa and vel")
                    
            r.sleep()

        print("lock")
        set_effect(r=255, g=150, b=0)
        z_st = 1.5 # зависит какая у вас высота самого дрона, лучше чтобы было само значение высоты или 0.2 +- погрешность (вроде -0.2 погрешность лучше)

        st_t = rospy.get_time()
        z_vel = 0.5 # тут можно тестить значения самих цифр, но как помню это идеальное для всех (вроде)

        Z = z_st
        r = rospy.Rate(50) # как часто обновлять, тут тоже можно менять само значение, но больше +-70 лучше не ставить, как и меньше 10, быстро будет терять метку
        while l_z > 0.09:
            pb = get_body_pose("aruco_map")
            pa = get_aruco_pose("aruco_map")
            now = rospy.get_time()
            if prev_pa is None: # такой же принцип как в прошлом цикле
                set_position(x=0, y=0, z=-0.01, yaw=float('nan'), frame_id="body")
                set_effect(r=255, g=150, b=0, effect='blink_fast')
                print("NO prev_pa")
                prev_pa = pa
                prev_t = now
            else:
                if pb is not None:
                    d = np.linalg.norm(pb[:2]-pa[:2])
                if pa is not None:
                    set_effect(r=255, g=150, b=0)
                    if np.linalg.norm(pa - prev_pa) < 0.0001 and prev_vel is not None: # тут можно тестить значения самих цифр
                        vel = prev_vel.copy()*0.93 # тут можно тестить значения самих цифр, НАСКОЛЬКО ПОНИМАЮ ПО КОДУ И ПОМНЮ, ЧЕМ БОЛЬШЕ ЗНАЧЕНИЕ - ТЕМ БУДЕТ РОВНЕЕ К САМОЙ АРУКЕ НО КОГДА ВЫ САДИТЕСЬ
                        # ЛУЧШЕ НЕ СТАВИТЬ ЗНАЧЕНИЕ БОЛЬШЕ 1.05 ИБО БОЛЬШЕ 1 ОН БУДЕТ СИЛЬНЕЕ НАПЕРЕД И ЕСЛИ ПЫЛЕСОС ПОВЕРНЕТСЯ ОН НЕ ТОЧНО ПРИЗМЕЛИТСЯ НО ТЕСТИТЕ ВСЕ РАВНО КАК БУДЕТ ЛУЧШЕ ВАМ
                        print('1')
                    else:
                        print('2')
                        vel = (pa-prev_pa)/(now-prev_t+0.000025) # тут можно тестить значения самих цифр
                    if l_z > 0.5:
                        print(vel)
                        vel = np.clip(vel, -0.49, 0.49) # тут можно тестить значения самих цифр ВРОДЕ идеал
                    else:
                        print(vel)
                        vel = np.clip(vel, -0.3, 0.3) # тут можно тестить значения самих цифр ВРОДЕ идеал

                    vel = remove_0_vel(vel)
                    if np.linalg.norm(vel[:2]) < 0.02 and d <= 0.07:
                        Z = -(rospy.get_time()-st_t)*(0.7) + z_st
                        print("FAST")
                    else:
                        Z = -(rospy.get_time()-st_t)*z_vel + z_st
                    t = pa[:2] + vel[:2]*(1.0/FRQ)*2.2 # тут можно тестить значения самих цифр ВРОДЕ идеал
                    set_position(x=t[0], y=t[1], z=Z, yaw=float('nan'), frame_id="aruco_map")

                    prev_pa = pa.copy()
                    print("Z = ", Z)
                    prev_vel = vel.copy()
                    prev_t = now
                else:
                    set_position(x=0, y=0, z=-0.01, frame_id="body")
                    set_effect(r=255, g=150, b=0, effect='blink_fast')
                    print("NO pa and vel") # такого ни разу не было на практике как помню
                    
            r.sleep()
        
        if l_z <= 0.09: # если дистанция до самой аруке меньше 0.09м - дизарм, то есть дизармнется прям когда уже сядет, но можете потестить до 0.25м
            set_effect(r=255, g=0, b=0)
            arming(False)
        else:
            set_effect(r=255, g=0, b=0, effect="blink")
            print("Hmmmm")

if __name__ == "__main__":
    main()

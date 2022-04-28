import serial
import time
import pygame
import math
import waypoints
from numpy import *

arduino = serial.Serial(port = 'COM3', baudrate = 115200, timeout=1)
pygame.init()

WIDTH = 800
HEIGHT = 800

BLOCK_WIDTH = 80

ORIGIN = (WIDTH//2, HEIGHT//2)

ARM_SEGMENT_ONE = 160
ARM_SEGMENT_TWO = 202
ARM_SEGMENT_THREE = 40
MAX_Z = 756

screen = pygame.display.set_mode((WIDTH, HEIGHT))

class Dot(object):
    def __init__(self, pos):
        self.pos = list(pos)
        self.radius = 10
        self.color = (255, 0, 0)

    def draw(self, surface):
        pygame.draw.circle(surface, self.color, self.pos, self.radius)

    def is_moused_over(self, mouse_pos):
        if math.dist(mouse_pos, self.pos)< self.radius:
            return True
        return False

    

def write_data(pointer, a_p, z, c):   
    angles = list(inverse_kinematics(interpret_mouse(pointer.pos)))
    angles.append(a_p[2])

    if isnan(angles[0]) or isnan(angles[1]):
        return

    a_p[0] = angles[0]
    a_p[1] = angles[1]

    return write_direct_data(angles, z, c)

def write_direct_data(angles, height, c):
    computed = (bytes(str(int(angles[0])), "utf-8"), bytes(str(int(angles[1])), "utf-8"), bytes(str(int(angles[2])), "utf-8"))
    #print(computed)
    
    z = bytes(str(int(height)), "utf-8")
    closed = bytes(str(int(c)), "utf-8")
    delim = bytes(';', "utf-8")
    
    arduino.write(computed[0])    
    arduino.write(delim)   
    arduino.write(computed[1])
    arduino.write(delim)
    arduino.write(computed[2])
    arduino.write(delim)
    arduino.write(z)
    arduino.write(delim)
    arduino.write(closed)
    arduino.write(delim)
    
    return 1

def draw_joint(surface, start_pos, angle, length, width):
    x = start_pos[0] + math.cos(math.radians(angle-90)) * length
    y = start_pos[1] + math.sin(math.radians(angle-90)) * length

    end_pos = (x, y)
    
    pygame.draw.line(surface, (0, 0, 0), start_pos, end_pos, width)

    return end_pos

def interpret_mouse(pos):
    x = pos[0]; y = pos[1]

    return (x-(WIDTH//2), HEIGHT-y-(HEIGHT//2))
    #return (x-(WIDTH//2), -(y-(HEIGHT//2)))


def inverse_kinematics(target_pos):
    invert = False
    
    L1 = ARM_SEGMENT_ONE
    L2 = ARM_SEGMENT_TWO
    x = target_pos[0]   
    y = target_pos[1]

    if x >= 0:
        invert = True
        x = -x

    r1 = sqrt(x**2+y**2)  # eqn 1
    phi_1 = arccos((L2**2-L1**2-r1**2)/(-2*L1*r1))  # eqn 2
    phi_2 = arctan2(y, x)  # eqn 3
    theta_1 = (rad2deg(phi_2-phi_1)-90)  # eqn 4 converted to degrees

    phi_3 = arccos((r1**2-L1**2-L2**2)/(-2*L1*L2))
    theta_2 = 180-rad2deg(phi_3)

    if invert:
        theta_1 = -theta_1
        theta_2 = -theta_2

    if y < 0:
        theta_1 = -theta_1

    sign_1 = abs(theta_1)//theta_1
    sign_2 = abs(theta_2)//theta_2

    if abs(theta_1) >= 180:
        theta_1 = (360-abs(theta_1)) * sign_1

    if abs(theta_2) >= 180:
        theta_2 = (360-abs(theta_2)) * sign_2

    return (-theta_1, -theta_2)

def export_waypoints(path, filename):
    with open(filename, "w") as f:   
        for point in path:
            point.write(f)

def load_waypoints(filename):
    with open(filename, "r") as f:
        return waypoints.parse_file(f.readlines())


running = True

pointer = Dot(ORIGIN)

dragging = False

gripped = False

last_point_pos = (0, 0)

clock = pygame.time.Clock()

angles = [0, 0, 0]
z_height = 0

pointer.pos[1] -= 300

saving_path = False
stored_path = []

font = pygame.font.Font(None, 48)
building_path_img = font.render("Building path: 0", True, (20, 20, 20))

#1: manual plot; 2: follow previous path
current_state = 1
arduino_ready = True
current_waypoint = 0

update_arduino_count = 0

changed = False

while running:
    clock.tick(60)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEWHEEL:
            z_height = max(min(z_height - event.y*4, MAX_Z), 0)
            changed = True

        elif event.type == pygame.KEYDOWN:           
            if event.key == pygame.K_b: #Begin path
                saving_path = True
                stored_path = []
                
            elif event.key == pygame.K_s: #Save path
                export_waypoints(stored_path, "C:/dump/example.txt")
                saving_path = False
                stored_path = []
                
            elif event.key == pygame.K_SPACE: #Add waypoint
                if saving_path:
                    stored_path.append(waypoints.Waypoint(angles=angles, height=z_height, grip=gripped))
                    building_path_img = font.render("Building path: "+str(len(stored_path)), True, (20, 20, 20))  

            elif event.key == pygame.K_p and current_state == 1: #Load waypoints
                saving_path = False
                stored_path = load_waypoints("C:/dump/example.txt")
                current_state = 2

            elif event.key == pygame.K_l and current_state == 1:
                saving_path = False
                stored_path = load_waypoints("C:/dump/example.txt")
                current_state = 3

            elif event.key == pygame.K_ESCAPE:
                current_state = 1
                current_waypoint = 0

            elif event.key == pygame.K_LSHIFT:
                gripped = not gripped
                changed = True

        
            
    screen.fill((255, 255, 255))

    keys = pygame.key.get_pressed()

    if keys[pygame.K_LEFT]:
        angles[2] -= 2
        changed = True
    if keys[pygame.K_RIGHT]:
        angles[2] += 2
        changed = True

    if angles[2] < -180:
        angles[2] += 360

    if angles[2] > 180:
        angles[2] -= 360

    if gripped:
        pygame.draw.circle(screen, (0, 0, 0), (WIDTH*0.95, HEIGHT*0.95), 10)

    if saving_path:
        screen.blit(building_path_img, (30, 30))

    for x in range(WIDTH//BLOCK_WIDTH):
        pygame.draw.line(screen, (50, 50, 50), ((x+1)*BLOCK_WIDTH, 0), ((x+1)*BLOCK_WIDTH, HEIGHT))
        
    for y in range(HEIGHT//BLOCK_WIDTH):
        pygame.draw.line(screen, (50, 50, 50), (0, (y+1)*BLOCK_WIDTH), (WIDTH, (y+1)*BLOCK_WIDTH))

    pygame.draw.circle(screen, (0, 0, 0), ORIGIN, 5)

    end = draw_joint(screen, ORIGIN, angles[0], ARM_SEGMENT_ONE, 10)
    end = draw_joint(screen, end, angles[0]+angles[1], ARM_SEGMENT_TWO, 5)
    draw_joint(screen, end, angles[0]+angles[1]+angles[2], ARM_SEGMENT_THREE, 2)

    #draw_joint(screen, ORIGIN, -80, ARM_SEGMENT_ONE, 10)
    
    pointer.draw(screen)
    
    if current_state == 1:
        if pointer.is_moused_over(pygame.mouse.get_pos()) and pygame.mouse.get_pressed()[0]:
            dragging = True
            
        if dragging:
            pointer.pos = pygame.mouse.get_pos()
            
        if not pygame.mouse.get_pressed()[0]:
            dragging = False
        
        if last_point_pos[0] != pointer.pos[0] or last_point_pos[1] != pointer.pos[1] or changed:
            if update_arduino_count >= 5:
                write_data(pointer, angles, z_height, gripped)
                last_point_pos = tuple(pointer.pos)
                update_arduino_count %= 5
            

    elif current_state in (2, 3):
        if arduino_ready:
            angles = list(stored_path[current_waypoint].angles)
            z_height = stored_path[current_waypoint].height
            g = stored_path[current_waypoint].grip
            
            write_direct_data(angles, z_height, g)

            arduino_ready = False

            current_waypoint += 1

            if current_waypoint >= len(stored_path):
                current_waypoint = 0

                if current_state == 2:
                    current_state = 1
    
    if arduino.inWaiting():
        s = arduino.readline()

        s = s.decode()
        print(s.strip())#, ":", inverse_kinematics(interpret_mouse(pointer.pos)))

        if s.startswith("Ready"):
            arduino_ready = True

        elif s.startswith("Replay"):
            saving_path = False
            stored_path = load_waypoints("C:/dump/example.txt")
            current_state = 2

    update_arduino_count += 1
    pygame.display.flip()


pygame.quit()
arduino.close()

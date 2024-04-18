import pygame
import FakeDrone
import numpy as np
import time
# quad = FakeDrone.FakeQuad()
import calibration

x = 0
y = 0
z = 0
thrust = 0
ThrustTilte = 0
emergency_stop = False
engage = False
disengage = False



engaged = False#flag to indicate if the drone is engaged

last_x = 0
last_y = 0
last_z = 0
last_thrust = 0
last_ThrustTilte = 0

pygame.init()
pygame.joystick.init()

#define screen size
# SCREEN_WIDTH = 800
# SCREEN_HEIGHT = 500

#create game window
# screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
# pygame.display.set_caption("Joysticks")

#create empty list to store joysticks
joysticks = []

new_data = False#flag to indicate new data has been received

# display_attitude = [1, 0, 0, 0]
# display_position = [0, 0, 0]
# def draw_box(attitude = None,position=None,color = (255, 255, 255)):
#     if attitude is None:
#         attitude = display_attitude
#     if position is None:
#         position = display_position
#     #project the box on the image using the calibration.prject function
#     dx = 0.40
#     dy = 0.40
#     dz = 0.1
#     vertices = np.array([[-dx/2,-dy/2,-dz/2],
#                             [dx/2,-dy/2,-dz/2],
#                             [dx/2,dy/2,-dz/2],
#                             [-dx/2,dy/2,-dz/2],
#                             [-dx/2,-dy/2,dz/2],
#                             [dx/2,-dy/2,dz/2],
#                             [dx/2,dy/2,dz/2],
#                             [-dx/2,dy/2,dz/2]])
#     #project the vertices
#     projected = []
#     for vertex in vertices:
#         projected.append(calibration.project(vertex, np.array(attitude), np.array(position), np.array([-2,0,0]), np.array([0.5, 0.5, 0.5, 0.5]), 300))

#     #draw the lines
#     for i, j in [(0, 1), (1, 2), (2, 3), (3, 0),
#                     (4, 5), (5, 6), (6, 7), (7, 4),
#                     (0, 4), (1, 5), (2, 6), (3, 7)]:
#             pygame.draw.line(screen,
#                             color,
#                             (round(projected[i][0] + screen.get_width()/2),
#                             round(projected[i][1] + screen.get_height()/2)),
#                             (round(projected[j][0] + screen.get_width()/2),
#                             round(projected[j][1] + screen.get_height()/2)), 1)

def is_new_data():
    global x, y, z, thrust, ThrustTilte, last_x, last_y, last_z, last_thrust, last_ThrustTilte, new_data
    if (x!=last_x or y!=last_y or z!=last_z or thrust!=last_thrust or ThrustTilte!=last_ThrustTilte):
        last_x = x
        last_y = y
        last_z = z
        last_thrust = thrust
        last_ThrustTilte = ThrustTilte
        return True
    return False

#game loop
last_time_run_controller = None
def run_controller():
    global x, y, z, thrust, ThrustTilte, last_x, last_y, last_z, last_thrust, last_ThrustTilte, new_data, emergency_stop, engage, disengage, engaged, last_time_run_controller
    # screen.fill(pygame.Color("midnightblue"))
    # draw_box()
    # pygame.display.flip()
    t = time.time()
    if last_time_run_controller is None:
        last_time_run_controller = t
    dt = t - last_time_run_controller
    last_time_run_controller = t
    
    #show number of connected joysticks
    for joystick in joysticks:
        x = joystick.get_axis(2)
        y = joystick.get_axis(3)
        z = joystick.get_axis(0)
        thrustAxis = -joystick.get_axis(1)
        button6 = joystick.get_button(6)
        button4 = joystick.get_button(4)

        if engaged:
            thrust = 0.1 * thrustAxis
            if button4:
                ThrustTilte += 0.1 * thrustAxis * dt
               

        if button6:
            if thrustAxis < -0.9:
                #cut throttle
                thrust = 0
                ThrustTilte = 0
                disengage = True
                engaged = False
            elif thrustAxis > 0.9:
                #engage
                engage = True
                engaged = True
        if button4 and button6 and thrustAxis < -0.9:
            emergency_stop = True
                
                        
clock = pygame.time.Clock()
FPS = 60


running = True
def run():
    global running
    if not running:
        return
    clock.tick(FPS)
    for event in pygame.event.get():
        if event.type == pygame.JOYDEVICEADDED:
            joy = pygame.joystick.Joystick(event.device_index)
            joysticks.append(joy)
        #quit program
        if event.type == pygame.QUIT:
            print("QUITTING")
            running = False
            pygame.quit()
    if running:
        run_controller()
    

if __name__ == "__main__":
    while True:
        run()
        if is_new_data():
            print(x, y, z, thrust, ThrustTilte)

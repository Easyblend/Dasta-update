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

pygame.init()
pygame.joystick.init()

#define screen size
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 500

#create game window
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Joysticks")

#create empty list to store joysticks
joysticks = []

display_attitude = [1, 0, 0, 0]
display_position = [0, 0, 0]
def draw_box():
    #project the box on the image using the calibration.prject function
    dx = 0.40
    dy = 0.40
    dz = 0.1
    vertices = np.array([[-dx/2,-dy/2,-dz/2],
                            [dx/2,-dy/2,-dz/2],
                            [dx/2,dy/2,-dz/2],
                            [-dx/2,dy/2,-dz/2],
                            [-dx/2,-dy/2,dz/2],
                            [dx/2,-dy/2,dz/2],
                            [dx/2,dy/2,dz/2],
                            [-dx/2,dy/2,dz/2]])
    #project the vertices
    projected = []
    for vertex in vertices:
        projected.append(calibration.project(vertex, np.array(display_attitude), np.array(display_position), np.array([-2,0,-1]), np.array([0.5, 0.5, 0.5, 0.5]), 300))

    #draw the lines
    for i, j in [(0, 1), (1, 2), (2, 3), (3, 0),
                    (4, 5), (5, 6), (6, 7), (7, 4),
                    (0, 4), (1, 5), (2, 6), (3, 7)]:
            pygame.draw.line(screen,
                            (255, 255, 255),
                            (round(projected[i][0] + screen.get_width()/2),
                            round(projected[i][1] + screen.get_height()/2)),
                            (round(projected[j][0] + screen.get_width()/2),
                            round(projected[j][1] + screen.get_height()/2)), 1)

#define player colour
#game loop
def run():
    global x, y, z, thrust, ThrustTilte
    screen.fill(pygame.Color("midnightblue"))
    draw_box()
    pygame.display.flip()

    #show number of connected joysticks
    for joystick in joysticks:
        x = joystick.get_axis(2)
        y = joystick.get_axis(3)
        z = joystick.get_axis(0)
        thrustAxis = -joystick.get_axis(1)
        button6 = joystick.get_button(6)

        if button6:
            ThrustTilte += 0.1 * thrustAxis / 50

        if joystick.get_button(4) and button6:
            if thrustAxis < -0.9:
                #cut throttle
                thrust = 0
                ThrustTilte = 0
            else:
                #emergency stop
                thrust = 0
                ThrustTilte = 0
        else:
            thrust = 0.1 * thrustAxis
    # print("x: ", x, "y: ", y, "z: ", z, "thrust: ", thrust, "ThrustTilte: ", ThrustTilte)
clock = pygame.time.Clock()
FPS = 60

def main():
  running = True
  while running:
      clock.tick(FPS)
      
      for event in pygame.event.get():
          if event.type == pygame.JOYDEVICEADDED:
              joy = pygame.joystick.Joystick(event.device_index)
              joysticks.append(joy)
          #quit program
          if event.type == pygame.QUIT:
              print("QUITTING")
              running = False
      run()

  pygame.quit()

if __name__ == "__main__":
    main()
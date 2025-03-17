import pygame
import math
import numpy as np
WHITE_COLOR = (255, 255, 255)
GREEN_COLOR = (0, 255, 0)
BLACK_COLOR = (0, 0, 0)

class Envir:
    def __init__(self, dim): 
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        
        self.height = dim[0]
        self.width = dim[1]
        
        pygame.display.set_caption("Drive robot")
        self.map = pygame.display.set_mode((self.width, self.height))
        self.font = pygame.font.SysFont("arial", 50)
        self.text = self.font.render("default", True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.textRect.center = (dim[1] - 500, dim[0] - 100)
        self.trail_set = []

    def info(self, vx, vy, theta):
        text = f"Vx = {np.round(vx,2)}, Vy = {np.round(vy,2)}, Theta = {np.round(theta, 2)}"
        self.text1 = self.font.render(text, True, self.black, self.white)
        self.map.blit(self.text1, self.textRect)

    def sensor_info(self, sensor_data):
        text = f"sensor: {sensor_data}"
        self.text2 = self.font.render(text, True, self.black, self.white)
        self.textRect.center = (self.width - 850, self.height - 50)
        self.map.blit(self.text2, self.textRect)

    def trail(self, pos):
        for i in range(0, len(self.trail_set) - 1):
            pygame.draw.line(self.map, self.red, 
                         (self.trail_set[i][0], self.trail_set[i][1]), 
                         (self.trail_set[i+1][0], self.trail_set[i+1][1]))
        if self.trail_set.__sizeof__() > 30000:
            self.trail_set.pop(0)
        self.trail_set.append(pos)

    def robot_frame(self, pos, rotation):
        n = 80
        centerx, centery = pos
        x_axis = (centerx + n * math.cos(-rotation), centery + n * math.sin(-rotation))
        y_axis = (centerx + n * math.cos(-rotation + math.pi / 2), 
                centery + n * math.sin(-rotation + math.pi / 2))

        pygame.draw.line(self.map, self.blue, pos, x_axis, 3)
        pygame.draw.line(self.map, self.green, pos, y_axis, 3)


    def robot_sensor(self, pos, points):
        for point in points:
            pygame.draw.line(self.map, (0, 255, 0), pos, point)
            pygame.draw.circle(self.map, (0, 255, 0), point, 5)
 
class Robot:
    def __init__(self, startpos, Img, width):
        self.w = width
        self.x = startpos[0]
        self.y = startpos[1]
        self.theta = 0


        self.vx = 0 #pixel/s
        self.vy = 0
        self.theta_d = 0

        self.v1 = 0
        self.v2 = 0
        self.v3 = 0

        self.Vgx = 0
        self.Vgy = 0
        self.GTheta_d = 0

        self.sensor_data = [0,0,0,0,0,0]
        self.points = []

        #graphics
        self.img = pygame.image.load(Img)
        self.img = pygame.transform.scale(self.img, (50,50))
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

    def draw(self, map):
        map.blit(self.rotated, self.rect)    

    def move(self, event=None):
        if event is not None:
            if event.type == pygame.KEYDOWN:
                print(dt)
                if event.key == pygame.K_KP4:
                    self.Vgx -= 10
                elif event.key == pygame.K_KP6:
                    self.Vgx += 10
                elif event.key == pygame.K_KP8:
                    self.Vgy -= 10
                elif event.key == pygame.K_KP2:
                    self.Vgy += 10
                elif event.key == pygame.K_KP5:
                    self.GTheta_d -= 0.1
                elif event.key == pygame.K_KP0:
                    self.GTheta_d += 0.1
        

        inv_R = np.array([[np.cos(self.theta), -np.sin(self.theta), 0],
                          [np.sin(self.theta), np.cos(self.theta), 0],
                          [0,0,1]])

        l = 10
        inv_J1 = np.array([[1/np.sqrt(3), 0, -1/np.sqrt(3)],[-1/3, 2/3, -1/3],[-1/(3*l), -1/(3*l), -1/(3*l)]])
        r = 3
        J2 = np.array([[r,0,0],[0,r,0],[0,0,r]])

        V = np.linalg.inv(J2) @ np.linalg.inv(inv_J1) @ np.array([[self.Vgx],[self.Vgy],[self.GTheta_d]])
        self.v1 = V[0,0]
        self.v2 = V[1,0]
        self.v3 = V[2,0]

        AAA = inv_R @ inv_J1 @ J2 @np.array([[self.v1],[self.v2],[self.v3]])
        self.vx = AAA[0,0]
        self.vy = AAA[1,0]
        self.theta_d = AAA[2,0]


        self.x = self.x + self.vx * dt
        self.y = self.y + self.vy * dt
        self.theta = self.theta + self.theta_d * dt

        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(self.theta), 1)
        self.rect = self.rotated.get_rect(center = (self.x, self.y))


    def update_sensor_data(self):
        angles = [self.theta, np.pi/3+self.theta, 2*np.pi/3+self.theta, np.pi+self.theta, 4*np.pi/3+self.theta, 5*np.pi/3+self.theta]
        edge_points = []
        edge_distances = []
        for angle in angles:
            distance = 0
            edge_x, edge_y = (int(self.x), int(self.y))
            # while track_copy.get_at((edge_x, edge_y)) != WHITE_COLOR:
            #     edge_x =  int(self.x + distance * math.cos(angle))
            #     edge_y =  int(self.y + distance * math.sin(angle))
            #     distance += 1

            while 0 <= edge_x < track_copy.get_width() and 0 <= edge_y < track_copy.get_height():
                if track_copy.get_at((edge_x, edge_y)) == WHITE_COLOR:
                    break
                edge_x = int(self.x + distance * math.cos(angle))
                edge_y = int(self.y + distance * math.sin(angle))
                distance += 5

            edge_points.append((edge_x, edge_y))
            edge_distances.append(distance)
        self.sensor_data = edge_distances
        self.points = edge_points

    def check_crash(self):
        edge_x, edge_y = (int(self.x), int(self.y))
        if track_copy.get_at((edge_x, edge_y)) == BLACK_COLOR:
            self.crash = True
        if track_copy.get_at((edge_x, edge_y)) == GREEN_COLOR:
            self.finish = True

#init
pygame.init()
pygame.display.set_mode((913, 914))
track = pygame.image.load('resized_example.jpg')
track_copy = track.copy() 

start = (409, 25)
dims = (914, 913)


running = True 
       
dt = 0
lasttime = pygame.time.get_ticks()
environment = Envir(dims)
robot = Robot(start, "circle_blue_red_infinity.png",1)

#update
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        robot.move(event)

    robot.update_sensor_data()
    print(robot.sensor_data)




    dt = (pygame.time.get_ticks() - lasttime) / 1000
    lasttime = pygame.time.get_ticks()
    pygame.display.update()
    #environment.map.fill(environment.white)
    environment.map.blit(track, (0, 0))
    robot.move()

    robot.draw(environment.map)
    robot.update_sensor_data()
    environment.robot_frame((robot.x, robot.y), robot.theta)
    environment.robot_sensor((robot.x, robot.y), robot.points)
    environment.trail((robot.x, robot.y))
    # environment.info(robot.vx, robot.vy, robot.theta)
    environment.sensor_info(robot.sensor_data)
        





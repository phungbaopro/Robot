import pygame
import math
import numpy as np
BLACK_COLOR = (0,0,0)
GREEN_COLOR = (0,255,0)

class Envir:
    def __init__(self, dim):
        self.black = (0,0,0)
        self.white = (255,255,255)
        self.red = (255, 0, 0)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)

        self.height = dim[0]
        self.width = dim[1]
        #window settings
        pygame.display.set_caption("test robot")
        self.map = pygame.display.set_mode((self.width, self.height))
        self.font = pygame.font.SysFont("Arial", 50)
        self.text = self.font.render("default", True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.textRect.center = (dim[1] - 500, dim[0] - 100)
        self.trail_set = []

    def info(self, vk, vy, theta, v1, v2, v3):
        text = f"Vx= {np.round(vk, 2)}, Vy={np.round(vy, 2)}, Theta={np.round(theta, 2)}"
        self.text = self.font.render(text, True, self.black, self.white)
        self.map.blit(self.text, self.textRect)

    def sensor_info(self, sensor_data):
        text = f"Sensor: {sensor_data}"
        self.text2 = self.font.render(text, True, self.black, self.white)
        self.textRect.center = (self.width - 850, self.height - 50)
        self.map.blit(self.text2, self.textRect)

    def trail(self, pos):
        for i in range(0, len(self.trail_set) - 1):
            pygame.draw.line(self.map, self.red,
                             (self.trail_set[i][0], self.trail_set[i][1]),
                             (self.trail_set[i + 1][0], self.trail_set[i + 1][1]))
        if self.trail_set.__sizeof__() > 30000:
            self.trail_set.pop(0)
        self.trail_set.append(pos)

    def robot_frame(self, pos, rotation):
        centerx, centery = pos
        n = 80  # Chỉnh kích thước cho hợp lý
        x_axis = (centerx + n * math.cos(rotation), centery + n * math.sin(rotation))
        y_axis = (centerx + n * math.cos(rotation + math.pi/2), centery + n * math.sin(rotation + math.pi/2))

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
        self.vx = 0  # pixels/s
        self.vy = 0  # pixels/s
        self.theta_d = 0
        self.v1 = 0  # pixels/s
        self.v2 = 0
        self.v3 = 0
        self.Vgx = 0
        self.Vgy = 0
        self.GTheta_d = 0
        self.sensor_data = [0,0,0,0,0,0]
        self.points = []
        self.crash = False
        self.time = 0
        self.cost_function = 0

        # Graphics
        self.img = pygame.image.load(Img)
        self.img = pygame.transform.scale(self.img, (50, 50))
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
                if event.key == pygame.K_KP5:
                    self.GTheta_d -= 0.1
                elif event.key == pygame.K_KP0:
                    self.GTheta_d += 0.1

        inv_R = np.array([[np.cos(self.theta), -np.sin(self.theta), 0],
                        [np.sin(self.theta), np.cos(self.theta), 0],
                        [0, 0, 1]])

        l = 10
        inv_J1 = np.array([[1/np.sqrt(3), 0, -1/np.sqrt(3)],
                        [-1/3, 2/3, -1/3],
                        [-1/(3*l), -1/(3*l), -1/(3*l)]])

        r = 3
        J2 = np.array([[r, 0, 0],
                    [0, r, 0],
                    [0, 0, r]])

        V = np.linalg.inv(J2) @ np.linalg.inv(inv_J1) @ np.array([[self.Vgx], [self.Vgy], [self.GTheta_d]])

        self.v1 = V[0, 0]
        self.v2 = V[1, 0]
        self.v3 = V[2, 0]

        AAA = inv_R @ inv_J1 @ J2 @ np.array([[self.v1], [self.v2], [self.v3]])

        self.vx = AAA[0, 0]
        self.vy = AAA[1, 0]
        self.theta_d = AAA[2, 0]


        self.x = self.x + self.vx * dt
        self.y = self.y + self.vy * dt
        self.theta = self.theta + self.theta_d * dt

        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
    def update_sensor_data(self):
        angles = [self.theta, np.pi/3+self.theta, 2*np.pi/3+self.theta, np.pi+self.theta,4*np.pi/3+self.theta,5*np.pi/3+self.theta]
        edge_points = []
        edge_distances = []

        for angle in angles:
            distance = 0
            edge_x, edge_y = int(self.x), int(self.y)

            while track_copy.get_at((edge_x, edge_y)) == BLACK_COLOR:
                    edge_x = int(self.x + distance * math.cos(angle))
                    edge_y = int(self.y + distance * math.sin(angle))
                    distance += 1

            # Giới hạn điểm trong phạm vi ảnh
            edge_x = max(0, min(edge_x, track_copy.get_width() - 1))
            edge_y = max(0, min(edge_y, track_copy.get_height() - 1))

            edge_points.append((edge_x, edge_y))
            edge_distances.append(distance)

        self.sensor_data = edge_distances
        self.points = edge_points
    def check_crash(self):
        edge_x, edge_y= (int(self.x),(self.y))
        if track_copy.get_at((edge_x, edge_y)) == BLACK_COLOR:
            self.crash = True 
            self.cost_function = 999999999
        if self.time >= 10:
            self.crash = True
            self.cost_function = self.cost_function*5


# init
pygame.init()
pygame.display.set_mode((1152, 648))
flags = pygame.display.get_surface().get_flags()
if flags & pygame.FULLSCREEN:
    print("Cửa sổ đang ở chế độ toàn màn hình")
else:
    print("Cửa sổ không ở chế độ toàn màn hình")

track = pygame.image.load("vidu.png")
track_copy=track.copy()
start = (461, 48)
dims = (899, 900)


running = True

dt = 0
lasttime = pygame.time.get_ticks()

environment = Envir(dims)
number = 3    
Robots = []
for i in range(number):
    Robots.append(Robot(start, r"C:\Users\2012t\Desktop\ROBOT_CT4\circle_blue_red_infinity.png", 1))
pop_size = number
min_max = [-5, 5]
npar = 120
w = 0.9
c1 = 0.5
c2 = -0.5
max_iteration = 50 
Pbest_position = np.zeros((pop_size, npar))
Gbest_position = np.zeros((1, npar))
Pbest_fitness = np.ones(pop_size)*99999999999
Gbest_fitness = 99999999999
fitness = np.zeros(max_iteration)
robot_available = pop_size
P = np.random.uniform(min_max[0],min_max[1],(pop_size, npar))
V = P*0
def af(x):
    y = np.tanh(x)
    return y
def vidu_nn(X, W, V):
    Net = W.T @ X
    y_h = af(Net)
    output = V.T @y_h
    return output
iteration = 0
while running and iteration < max_iteration:
    for idx, robot in enumerate(Robots):
        robot.x = 354
        robot.y = 25
        robot.theta = np.pi/2
        robot.check_crash()
        robot.update_sensor_data()
        robot.crash = False
        robot.cost_fucntion = 0
        robot.time = 0
    robot_available = pop_size
    pJbest = 0
    while robot_available > 0:
        for idx, robot in enumerate(Robots):
            if robot.crash == True:
                ex = 402 - robot.x
                ey = 763 - robot.y
                etheta = np.pi/2 -robot.theta
                nn_input = np.array([
        [ex], 
        [ey], 
        [etheta], 
        [robot.sensor_data[0]], 
        [robot.sensor_data[1]], 
        [robot.sensor_data[2]],
        [robot.sensor_data[3]],
        [robot.sensor_data[4]],
        [robot.sensor_data[5]]
    ])
                W = P[idx,:90].reshape(9,10)
                V1= P[idx,90:].reshape(10,3)
                YY = vidu_nn(nn_input,W,V1)
                robot.Vgx = YY[0,0]
                robot.Vgy= YY[1,0]
                robot.Gtheta_d = YY[2,0]

                robot.cost_function = robot.cost_function + 0.001*ex**2 +0.001*ey*2 +1000*etheta*2
                if (robot.Vgx < 50):
                    robot.cost_function = robot.cost_function + 5*(50-robot.Vgx)**2
                if (robot.Vgy < 50):
                    robot.cost_function = robot.cost_function + 5*(50-robot.Vgy)**2

                robot.move()
                robot.time = robot.time +dt
                robot.check_crash()
                if robot.crash == True:
                    robot_available = robot_available - 1
                robot.draw(environment.map)
                robot.update_sensor_data()
                environment.robot_frame((robot.x, robot.y), robot.theta)
                environment.robot_sensor((robot.x, robot.y),robot.points)
        dt = (pygame.time.get_ticks() - lasttime) / 1000
        lasttime = pygame.time.get_ticks()
        pygame.display.update()
        environment.map.blit(track, (0, 0))
    for idx, robot in enumerate(Robots):
        J = robot.cost_function
        if (J < Pbest_fitness[idx]):
            Pbest_fitness[idx] = J
            Pbest_position[idx] = P[idx]
        if (J < Gbest_fitness):
            Gbest_fitness = J
            Gbest_position = P[idx]
        fitness[iteration] = Gbest_fitness
    V = W*V + c1*np.random.rand()*(Pbest_position - P)+c2*np.random.rand()*(Gbest_position-P)
    P= P + V
    print(f"Iteration: {iteration}, Best fitness: {Gbest_fitness}")
    iteration = iteration +1
print(Gbest_position)


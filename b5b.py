import pygame
import math
import numpy as np

# Màu sắc
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
        self.height, self.width = dim
        pygame.display.set_caption("Drive Robot")
        self.map = pygame.display.set_mode((self.width, self.height))
        self.font = pygame.font.SysFont("arial", 50)
        self.text = self.font.render("default", True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.textRect.center = (dim[1] - 500, dim[0] - 100)
        self.trail_set = []

    def info(self, vx, vy, theta):
        text = f"Vx = {np.round(vx, 2)}, Vy = {np.round(vy, 2)}, Theta = {np.round(theta, 2)}"
        self.text1 = self.font.render(text, True, self.black, self.white)
        self.map.blit(self.text1, self.textRect)

    def sensor_info(self, sensor_data):
        text = f"sensor: {sensor_data}"
        self.text2 = self.font.render(text, True, self.black, self.white)
        self.textRect.center = (self.width - 850, self.height - 50)
        self.map.blit(self.text2, self.textRect)

    def trail(self, pos):
        for i in range(len(self.trail_set) - 1):
            pygame.draw.line(self.map, self.red, self.trail_set[i], self.trail_set[i + 1])
        if len(self.trail_set) > 3000:  # Giới hạn độ dài dấu vết
            self.trail_set.pop(0)
        self.trail_set.append(pos)

    def robot_frame(self, pos, rotation):
        n = 80
        centerx, centery = pos
        x_axis = (centerx + n * math.cos(-rotation), centery + n * math.sin(-rotation))
        y_axis = (centerx + n * math.cos(-rotation + math.pi / 2), centery + n * math.sin(-rotation + math.pi / 2))
        pygame.draw.line(self.map, self.blue, pos, x_axis, 3)
        pygame.draw.line(self.map, self.green, pos, y_axis, 3)

    def robot_sensor(self, pos, points):
        for point in points:
            pygame.draw.line(self.map, self.green, pos, point)
            pygame.draw.circle(self.map, self.green, point, 5)

class Robot:
    def __init__(self, startpos, img_path, width):
        self.w = width
        self.x = startpos[0]
        self.y = startpos[1]
        self.theta = 0
        self.vx = 0
        self.vy = 0
        self.theta_d = 0
        self.v1 = 0
        self.v2 = 0
        self.v3 = 0
        self.Vgx = 0
        self.Vgy = 0
        self.GTheta_d = 0
        self.sensor_data = [0, 0, 0, 0, 0, 0]
        self.points = []
        self.crash = False
        self.time = 0
        self.cost_function = 0
        self.img = pygame.image.load(img_path).convert_alpha()
        self.img = pygame.transform.scale(self.img, (50, 50))
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

    def draw(self, map):
        map.blit(self.rotated, self.rect)

    def move(self, dt):
        inv_R = np.array([[np.cos(self.theta), -np.sin(self.theta), 0],
                          [np.sin(self.theta), np.cos(self.theta), 0],
                          [0, 0, 1]])
        l = 10
        inv_J1 = np.array([[1 / np.sqrt(3), 0, -1 / np.sqrt(3)],
                           [-1 / 3, 2 / 3, -1 / 3],
                           [-1 / (3 * l), -1 / (3 * l), -1 / (3 * l)]])
        r = 3
        J2 = np.array([[r, 0, 0], [0, r, 0], [0, 0, r]])

        V = np.linalg.inv(J2) @ np.linalg.inv(inv_J1) @ np.array([[self.Vgx], [self.Vgy], [self.GTheta_d]])
        self.v1, self.v2, self.v3 = V[:, 0]
        AAA = inv_R @ inv_J1 @ J2 @ np.array([[self.v1], [self.v2], [self.v3]])
        self.vx = AAA[0, 0]
        self.vy = AAA[1, 0]
        self.theta_d = AAA[2, 0]

        self.x += self.vx * dt
        self.y += self.vy * dt
        self.theta += self.theta_d * dt
        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

    def update_sensor_data(self, track_copy):
        angles = [self.theta, np.pi / 3 + self.theta, 2 * np.pi / 3 + self.theta,
                  np.pi + self.theta, 4 * np.pi / 3 + self.theta, 5 * np.pi / 3 + self.theta]
        edge_points = []
        edge_distances = []
        for angle in angles:
            distance = 0
            edge_x, edge_y = int(self.x), int(self.y)
            while 0 <= edge_x < track_copy.get_width() and 0 <= edge_y < track_copy.get_height():
                edge_x = int(self.x + distance * math.cos(angle))
                edge_y = int(self.y + distance * math.sin(angle))
                if track_copy.get_at((edge_x, edge_y)) != WHITE_COLOR:  # Giả định đường đi là màu trắng
                    break
                distance += 1
            edge_points.append((edge_x, edge_y))
            edge_distances.append(distance)
        self.sensor_data = edge_distances
        self.points = edge_points

    def check_crash(self, track_copy):
        edge_x, edge_y = int(self.x), int(self.y)
        if not (0 <= edge_x < track_copy.get_width() and 0 <= edge_y < track_copy.get_height()) or \
           track_copy.get_at((edge_x, edge_y)) != WHITE_COLOR:
            self.crash = True
        if self.time >= 10:
            self.crash = True

# Khởi tạo
pygame.init()
track = pygame.image.load('vidu.png').convert()
dims = (track.get_width(), track.get_height())  # Cập nhật kích thước theo hình ảnh
print(f"Kích thước track_copy: {track.get_width()}x{track.get_height()}")
environment = Envir(dims)
track_copy = track.copy()
start = (194, 56)  # Đảm bảo nằm trong phạm vi hình ảnh
number = 3
Robots = [Robot(start, "circle_blue_red_infinity.png", 1) for _ in range(number)]

# Tham số PSO
pop_size = number
min_max = [-5, 5]
npar = 120
w = 0.9
c1 = 0.5
c2 = 0.5
max_iteration = 1000

Pbest_position = np.zeros((pop_size, npar))
Gbest_position = np.zeros((1, npar))
Pbest_fitness = np.ones(pop_size) * 9999999
Gbest_fitness = 9999999
P = np.random.uniform(min_max[0], min_max[1], (pop_size, npar))
V = np.zeros((pop_size, npar))

def af(x):
    return np.tanh(x)

def vidu_nn(X, W, V):
    Net = W.T @ X
    y_h = af(Net)
    output = V.T @ y_h
    return output

running = True
lasttime = pygame.time.get_ticks()
iteration = 0

while running:
    iteration += 1
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    dt = (pygame.time.get_ticks() - lasttime) / 1000
    lasttime = pygame.time.get_ticks()

    for robot in Robots:
        robot.x = 200
        robot.y = 300
        robot.theta = np.pi / 2
        robot.Vgx = 0
        robot.check_crash(track_copy)
        robot.update_sensor_data(track_copy)
        robot.crash = False

    robot_available = pop_size
    pJbest = 0
    pbest = None

    while robot_available > 0:
        for idx, robot in enumerate(Robots):
            if not robot.crash:
                ex = 402 - robot.x
                ey = 763 - robot.y
                etheta = -np.pi / 2 - robot.theta
                nn_input = np.array([[ex], [ey], [etheta]] + robot.sensor_data + [[5]])
                W = P[idx, :90].reshape(9, 10)
                V1 = P[idx, 90:].reshape(10, 3)
                YY = vidu_nn(nn_input, W, V1)
                robot.Vgx, robot.Vgy, robot.GTheta_d = YY[:, 0]
                robot.cost_function = 1  # Hàm chi phí tạm thời
                robot.move(dt)
                robot.time += dt
                robot.check_crash(track_copy)
                if robot.crash:
                    robot_available -= 1
                robot.draw(environment.map)
                robot.update_sensor_data(track_copy)
                environment.robot_frame((robot.x, robot.y), robot.theta)
                environment.robot_sensor((robot.x, robot.y), robot.points)

        environment.map.blit(track, (0, 0))
        pygame.display.update()

    for idx, robot in enumerate(Robots):
        if robot.cost_function > pJbest:
            pJbest = robot.cost_function
            pbest = P[idx].reshape(1, npar)
    if pJbest > Gbest_fitness:
        Gbest_fitness = pJbest
        Gbest_position = pbest

    r1, r2 = np.random.rand(2)
    V = w * V + c1 * r1 * (pbest - P) + c2 * r2 * (Gbest_position - P)
    P += V
    print(f"Vòng lặp: {iteration}, pJbest: {pJbest}, Gbest: {Gbest_fitness}")

    if iteration >= max_iteration:
        break

pygame.quit()
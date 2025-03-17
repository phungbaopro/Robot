# import pygame
# import math


# class Car:
#     def __init__(self):
#         self.corners = []
#         self.edge_points = []
#         self.edge_distances = []
#         self.travelled_distance = 0
#         self.angle = 0
#         self.car_center = CAR_CENTER
#         self.car = pygame.image.load("").convert_alpha()
#         self.car = pygame.transform.scale(self.car, CAR_SIZE)
#         self.crashed = False
#         self.update_sensor_data()
    
#     def display_car(self):
#         rotated_car = pygame.transform.rotate(self.car, self.angle)
#         rect = rotated_car.get_rect(center = self.car_center)
#         SCREEN.blit(rotated_car, rect.topleft)

#     def crash_check(self):
#         """
#         check if any corner of the car goes out of the track
#         returns:
#         bool: returns true if the car is alive
#         """
#         for corner in self.corners:
#             if TRACK.get_at(corner) == WHITE_COLOR:
#                 return True
#         return False
    
     
#     def update_sensor_data(self):
#         """
#         Update the points on the edge of the track
#         and the distances between the points and the center of the car
#         """
#         angles = [360 - self.angle, 90 - self.angle, 180 - self.angle]
#         angles = [math.radians(i) for i in angles]
#         edge_points = []
#         edge_distances = []
#         for angle in angles:
#             distance = 0
#             edge_x, edge_y = self.car_center
#             while TRACK_COPY.get_at((edge_x, edge_y)) != WHITE_COLOR:
#                 edge_x = int(self.car_center[0] + distance * math.cos(angle))
#                 edge_y = int(self.car_center[1] + distance * math.sin(angle))
#                 distance += 1
#             edge_points.append((edge_x, edge_y))
#             edge_distances.append(distance)
#         self.edge_points = edge_points
#         self.edge_distances = edge_distances


#     def update_positiom(self):
#         """
#         update the new position of the car
#         """
#         self.car_center = translate_point(
#             self.car_center, 90 - self.angle, DELTA_DISTANCE)
#         self.travelled_distance += DELTA_DISTANCE
#         dist = math.sqrt(CAR_SIZE[0] ** 2 + CAR_SIZE[1] ** 2)/2
#         corners = []
#         corners.append(translate_point(
#             self.car_center, 60 - self.angle, dist))
#         corners.append(translate_point(
#             self.car_center, 120 - self.angle, dist))
#         corners.append(translate_point(
#             self.car_center, 240 - self.angle, dist))
#         corners.append(translate_point(
#             self.car_center, 300 - self.angle, dist))
#         self.corners = corners

#     def run(genomes, config):
#         """
#         run the game for a speccific generation of NEAT Algorithm
#         """
#         global GENERATION
#         GENERATION += 1
#         models = []
#         cars = []
        

#         while True


# class Envir:
#     def __init__(self, dimeentions):
#         #colors
#         self.black = (0, 0, 0)
#         self.white = (255, 255, 255)
#         self.green = (0, 255, 0)
#         self.blue = (0, 0, 255)
#         self.red = (255, 0, 0)
#         self.yel = (255, 255, 0)
#         #map dims
#         self.height = dimeentions[0]
#         self.width = dimeentions[1]
#         #window settings
#         pygame.display.set_caption("Differential drive robot")
#         self.map = pygame.display.set_mode((self.width, self.height))

#         #text variables
#         self.font = pygame.font.Font('freesansbold.ttf', 50)
#         self.text = self.font.render('default', True, self.white, self.black)
#         self.textRect = self.text.get_rect()
#         self.textRect.center = (dimeentions[1] - 600, dimeentions[0] - 100)

#         #trail
#         self.trail_set = []

#     def write_info(self, Vl, Vr, theta):
#         txt = f"Vl = {Vl} Vr = {Vr} theta = {int(math.degrees(theta))}"
#         self.text = self.font.render(txt, True, self.white, self.black)
#         self.map.blit(self.text, self.textRect)

#     def trail(self, pos):
#         for i in range(0, len(self.trail_set) - 1):
#             pygame.draw.line(self.map, self.yel, 
#                          (self.trail_set[i][0], self.trail_set[i][1]), 
#                          (self.trail_set[i+1][0], self.trail_set[i+1][1]))

#         if self.trail_set.__sizeof__() > 10000:
#             self.trail_set.pop(0)
#         self.trail_set.append(pos)


#     def robot_frame(self, pos, rotation):
#         n = 80

#         centerx, centery = pos
#         x_axis = (centerx + n * math.cos(-rotation), centery + n * math.sin(-rotation))
#         y_axis = (centerx + n * math.cos(-rotation + math.pi / 2), 
#                 centery + n * math.sin(-rotation + math.pi / 2))

#         pygame.draw.line(self.map, self.red, (centerx, centery), x_axis, 3)
#         pygame.draw.line(self.map, self.green, (centerx, centery), y_axis, 3)

    

# class Robot:
#     def __init__(self, startpos, robotImg, width):
#         self.m2p = 3779.52 #meters 2 pixels
#         #robot dims
#         self.w = width
#         self.x = startpos[0]
#         self.y = startpos[1]
#         self.theta = 0
#         self.vl = 0.01*self.m2p #meters/s
#         self.vr = 0.01*self.m2p
#         self.maxspeed = 0.02*self.m2p
#         self.minspeed = -0.02*self.m2p
#         #graphics
#         self.img = pygame.image.load(robotImg)
#         self.rotated = self.img
#         self.rect = self.rotated.get_rect(center=(self.x, self.y))

#     def draw(self, map):
#         map.blit(self.rotated, self.rect)    

#     def move(self, event=None):
#         if event is not None:
#             if event.type == pygame.KEYDOWN:
#                 if event.key == pygame.K_KP4:
#                     self.vl += 0.001 * self.m2p
#                 elif event.key == pygame.K_KP1:
#                     self.vl -= 0.001 * self.m2p
#                 elif event.key == pygame.K_KP6:
#                     self.vr += 0.001 * self.m2p
#                 elif event.key == pygame.K_KP3:
#                     self.vr -= 0.001 * self.m2p
#         self.x += ((self.vl + self.vr)/2) * math.cos(self.theta) *dt
#         self.y -= ((self.vl + self.vr)/2) * math.sin(self.theta) *dt
#         self.theta += (self.vr - self.vl) / self.w * dt

#         #reset theta
#         if self.theta > 2 * math.pi or self.theta < -2 * math.pi:
#             self.theta = 0

#         #set max speed
#         self.vr = min (self.vr, self.maxspeed)
#         self.vl = min (self.vr, self.maxspeed)
#         #set min speed
#         self.vr = max (self.vr, self.minspeed)
#         self.vl = max (self.vr, self.minspeed)

#         self.rotated = pygame.transform.rotozoom(self.img, math.degrees(self.theta), 1)
#         self.rect = self.rotated.get_rect(center = (self.x, self.y))




# #initialisation
# pygame.init()

# #start position
# start = (200, 200)

# #dimentions
# dims = (800, 1200)

# #running or not
# running = True

# #the envir
# environment = Envir(dims)

# #the robot
# robot = Robot(start, r"C:\Users\2012t\Desktop\ROBOT_CT4\DDR.png", 0.01*3779.52)

# #dt
# dt = 0
# lasttime = pygame.time.get_ticks()
# #simulation loop
# while running:
#     #activate the quit button
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#         robot.move(event)



#     dt = (pygame.time.get_ticks() - lasttime) / 1000
#     lasttime = pygame.time.get_ticks()
#     pygame.display.update()
#     environment.map.fill(environment.black)
#     robot.move()
#     environment.robot_frame((robot.x, robot.y), robot.theta)
#     environment.write_info(int(robot.vl), int(robot.vr), robot.theta)
#     robot.draw(environment.map)
#     environment.trail((robot.x, robot.y))

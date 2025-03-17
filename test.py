import math
import pygame
import time

# Khởi tạo Pygame
pygame.init()

# Cài đặt cửa sổ hiển thị với kích thước 50x50 pixels
WIDTH, HEIGHT = 50, 50
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Mô phỏng Robot Omnidirectional")

# Màu sắc
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)  # Màu cho bánh xe

class OmniRobot:
    def __init__(self):
        # Thông số robot
        self.circumference = 40.0  # cm, như bạn yêu cầu
        self.radius = self.circumference / (2 * math.pi)  # Bán kính thân robot (cm)
        self.num_wheels = 4  # Sử dụng 4 bánh xe
        self.wheel_diameter = 5.0  # cm (tương đương 50 mm, trung bình của 45-58 mm)
        self.wheel_thickness = 0.8  # cm (tương đương 8 mm, nhỏ hơn 10 cm)
        self.wheel_radius = self.wheel_diameter / 2  # Bán kính bánh xe (cm)

        # Vị trí bánh xe (góc độ theo độ)
        self.wheel_angles = [0, 90, 180, 270]  # Bố trí đều quanh robot
        self.wheel_positions = []  # Lưu vị trí (x, y) của bánh xe (tính bằng pixel)
        
        # Tính toán vị trí bánh xe và tỷ lệ pixel/cm
        self.scale = 2  # Điều chỉnh tỷ lệ để robot vừa với cửa sổ 50x50 (2 pixel = 1 cm)
        self._calculate_wheel_positions()

    def _calculate_wheel_positions(self):
        """Tính toán vị trí (x, y) của bánh xe quanh thân robot."""
        center_x, center_y = WIDTH // 2, HEIGHT // 2  # Trung tâm màn hình 50x50
        for angle in self.wheel_angles:
            rad = math.radians(angle)
            x = center_x + (self.radius * self.scale * math.cos(rad))
            y = center_y - (self.radius * self.scale * math.sin(rad))  # Trừ vì hệ tọa độ Pygame
            self.wheel_positions.append((x, y))

    def draw(self, screen):
        """Vẽ robot và bánh xe lên màn hình."""
        # Vẽ thân robot (hình tròn)
        center_x, center_y = WIDTH // 2, HEIGHT // 2
        pygame.draw.circle(screen, BLACK, (center_x, center_y), int(self.radius * self.scale), 1)

        # Vẽ các bánh xe
        for (x, y) in self.wheel_positions:
            pygame.draw.circle(screen, RED, (int(x), int(y)), int(self.wheel_radius * self.scale), 1)

def main():
    clock = pygame.time.Clock()
    robot = OmniRobot()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Xóa màn hình
        screen.fill(WHITE)

        # Vẽ robot
        robot.draw(screen)

        # Cập nhật hiển thị
        pygame.display.flip()
        clock.tick(60)  # Giới hạn 60 FPS

    pygame.quit()

if __name__ == "__main__":
    main()
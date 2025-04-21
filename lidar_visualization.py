import serial
import pygame
import math
import argparse
import time
import asyncio
from LDS006 import LidarPacketHandler

# Globals
serial_port = None
point_array = []  # 360 angles, lists (dist, timestamp)
FPS = 60

def interpolate_color(dist, min_dist=120, max_dist=3500):
    """Interpolate color from black (min_dist) to yellow (max_dist), similar to cmap='hot'."""
    norm_dist = (dist - min_dist) / (max_dist - min_dist)
    norm_dist = max(0, min(1, norm_dist))  # Clamp to [0, 1]
    # Example hot colormap: black -> red -> yellow
    r = int(255 * min(2 * norm_dist, 1))
    g = int(255 * max(0, 2 * norm_dist - 1))
    b = 0
    return (r, g, b)

def interpolate_hsv(angle):
    """Interpolate color based on angle (0–360°) through HSV for a full spectrum."""
    hue = angle % 360  # Normalize angle to [0, 360]
    color = pygame.Color(0)
    color.hsva = (hue, 100, 100, 100)  # Hue=angle, Saturation=100%, Value=100%, Alpha=100%
    return (color.r, color.g, color.b)

async def main():
    parser = argparse.ArgumentParser(description="Visualize Lidar LDS-006 points from serial port in realtime using Pygame.")
    parser.add_argument('--port', type=str, default='COM6', help="Serial port (e.g., COM6)")
    parser.add_argument('--baudrate', type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument('--point-size', type=float, default=3.0, help="Point size in pixels (default: 3.0)")
    args = parser.parse_args()
    
    # Initialize running
    running = True
    
    # Initialize Pygame
    pygame.init()

    WIDTH = 1280  # WIDESCREEN
    HEIGHT = 960

    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Lidar LDS-006 Point Cloud (Realtime)")
    clock = pygame.time.Clock()
    
    # Display parameters
    scale = 0.1  # Pixels per mm
    rotation = 0.0  # Rotation angle in radians
    center = (WIDTH/2, HEIGHT/2)  # Screen center
    dragging = False
    last_mouse_pos = None
    
    # Button for sending commands
    button_rect = pygame.Rect(WIDTH-100, HEIGHT-50, 90, 30)
    font = pygame.font.SysFont(None, 24)
    button_text = font.render("Command", True, (255, 255, 255))
    button_text_rect = button_text.get_rect(center=button_rect.center)

    # Button to quit the program
    quit_rect = pygame.Rect(WIDTH-200, HEIGHT-50, 90, 30)
    quit_text = font.render("Quit", True, (255, 255, 255))
    quit_text_rect = quit_text.get_rect(center=quit_rect.center)
    xshift = 0
    yshift = 0
    xshift_temp = 0
    yshift_temp = 0
    panning = False
    p_last_mouse_pos = None
    mmb_up = False

    font = pygame.font.SysFont(None, int(args.point_size * 4))

    # Start serial port
    lidar = LidarPacketHandler(port=args.port, baudrate=args.baudrate)
    lidar.start()
    lidar.send_command("$startlds$")

    # Main Pygame loop
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if button_rect.collidepoint(event.pos):
                    pass
                elif quit_rect.collidepoint(event.pos):
                    running = False
                elif event.button == 1:  # Left mouse button
                    dragging = True
                    last_mouse_pos = event.pos
                elif event.button == 2: # Middle mouse button
                    panning = True
                    mmb_up = False
                    p_last_mouse_pos = event.pos
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1: # Left mouse button
                    dragging = False
                elif event.button == 2: # Middle mouse button
                    panning = False
                    mmb_up = True
            elif event.type == pygame.MOUSEMOTION:
                if dragging:
                    if last_mouse_pos:
                        dx = event.pos[0] - last_mouse_pos[0]
                        rotation -= dx * 0.01  # Rotation proportional to mouse movement
                        last_mouse_pos = event.pos
                if panning:
                    if p_last_mouse_pos:
                        # store translation in temp variable
                        px  = event.pos[0]  - p_last_mouse_pos[0]
                        py  = event.pos[1]  - p_last_mouse_pos[1]
                        xshift_temp = px
                        yshift_temp = py
                else:
                    if mmb_up:
                        #store translation once
                        xshift+=xshift_temp
                        yshift+=yshift_temp
                        xshift_temp=0.0
                        yshift_temp=0.0
                        mmb_up = False

                
            elif event.type == pygame.MOUSEWHEEL:
                scale *= 1.1 if event.y > 0 else 0.9  # Zoom with mouse wheel
                scale = max(0.01, min(1.0, scale))
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    scale *= 1.1
                    scale = min(1.0, scale)
                elif event.key == pygame.K_MINUS:
                    scale *= 0.9
                    scale = max(0.01, scale)
                elif event.key == pygame.K_LEFT:
                    rotation += 0.05
                elif event.key == pygame.K_RIGHT:
                    rotation -= 0.05
        
        # Clear screen
        screen.fill((0, 0, 0))

        # Update points
        point_array = lidar.get_points()
        for idx in range(360):
            if point_array[idx][0]:            
                distance = point_array[idx][0]
                angle = idx
            
                # Convert to screen coordinates
                rad = math.radians(angle + math.degrees(rotation))
                x = distance * math.cos(rad) * scale + xshift + xshift_temp
                y = distance * math.sin(rad) * scale - yshift - yshift_temp
                
                screen_x = center[0] + x
                screen_y = center[1] - y  # Invert Y for correct orientation
                
                color = interpolate_hsv(angle)             
             
                pygame.draw.circle(screen, color, (screen_x, screen_y), args.point_size)
    
        # Draw buttons
        pygame.draw.rect(screen, (50, 50, 50), quit_rect)
        screen.blit(quit_text, quit_text_rect)
        
        pygame.display.flip()
        clock.tick(FPS)
        await asyncio.sleep(1.0 / FPS)
    
    # Cleanup
    lidar.send_command("$stoplds")
    lidar.stop() 
    lidar = None
    pygame.quit()

if __name__ == "__main__":
    asyncio.run(main())
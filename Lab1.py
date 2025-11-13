import numpy as np
import matplotlib.pyplot as plt
import pygame as gm
import math


gm.init()

screen = gm.display.set_mode((400, 400))
gm.display.set_caption("Rotate Line with Arrow Keys")

# Parameters
center = (200, 300)  # base of the line
length = 100         # line length
angle = 0            # in degrees, 0 = pointing straight up
new_angle = 0
running = True
clock = gm.time.Clock()

while running:
    for event in gm.event.get():
        if event.type == gm.QUIT:
            running = False
    
    #pressing left and right keys to change the angle type shit     
    keys = gm.key.get_pressed()
    if keys[gm.K_LEFT]:
        if angle >= -80:
            angle -= 1  
    if keys[gm.K_RIGHT]:
        if angle <= 80:
            angle += 1  

    #calculate the end of the line
    radians = math.radians(angle)
    end_x = center[0] + length * math.sin(radians)
    end_y = center[1] - length * math.cos(radians)

    #desenhar o ecrã
    screen.fill((0, 0, 0))
    gm.draw.line(screen, (155, 0, 0), center, (end_x, end_y), 5)
    gm.display.flip()

    #dar print ao angulo atual
    if new_angle != angle:
        print(angle)
    new_angle = angle
    
    clock.tick(60)  # limit to 60 frames per second

gm.quit()
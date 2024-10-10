import pygame
import random
import math
from Player import Player
from Enemy import Enemy

pygame.init()
size = (800, 600)
BGCOLOR = (255, 255, 255)
screen = pygame.display.set_mode(size, pygame.RESIZABLE)
modeFont = pygame.font.Font("fonts/UpheavalPro.ttf", 30)
pygame.display.set_caption("IA Testing")

done = False
player_char = pygame.sprite.GroupSingle(Player(screen.get_size()))
enemies = pygame.sprite.Group()
score = 0
clock = pygame.time.Clock()
behaviour_list = ['Kinematic Wandering', 'Kinematic Arrive', 'Kinematic Seek', 'Kinematic Flee', 
                  'Dynamic Flee', 'Dynamic Seek', 'Dynamic Arrive', 'Align', 'Velocity Matching', 
                  'Face', 'Pursue', 'Evade', 'Dynamic Wander', 'Colition Avoidance', 'Obstacle and Wall Avoidance', 'Separation', 'Pursue While Avoid'
                  ,'Pursue and Avoid']

obstacles = pygame.sprite.Group()

class Obstacle(pygame.sprite.Sprite):
    def __init__(self, x, y, width, height):
        super().__init__()
        self.image = pygame.Surface([width, height])
        self.image.fill((150, 150, 150)) 
        self.rect = self.image.get_rect(topleft=(x, y))

def move_entities(player_char, enemies, obstacles_group, timeDelta):
    score = 0
    player_char.sprite.move(screen.get_size(), timeDelta, obstacles_group)

    # Verificar colisiones entre el jugador y los obstaculos
    if pygame.sprite.spritecollide(player_char.sprite, obstacles_group, False):
        player_char.sprite.velocity[0] = 0 
        player_char.sprite.velocity[1] = 0

    for enemy in enemies:
        enemy.move(enemies, screen.get_size(), timeDelta, player_char.sprite, obstacles_group)
        
        # Verificar colisiones entre enemigos y obstaculos
        if pygame.sprite.spritecollide(enemy, obstacles_group, False):
            enemy.velocity[0] = 0
            enemy.velocity[1] = 0
    
    return score

def render_entities(player_char, enemies):
    player_char.sprite.render(screen)
    for enemy in enemies:
        enemy.render(screen)

def clear_enemies(enemies):
    for sprite in enemies:
        sprite.kill()

# Funcion para crear obstaculos cuadrados de diferentes tamaños y posiciones aleatorias
def create_obstacles(obstacles_group):
    for _ in range(10):
        width = random.randint(50, 100)
        height = random.randint(50, 100)
        x = random.randint(0, size[0] - width)
        y = random.randint(0, size[1] - height)
        obstacle = Obstacle(x, y, width, height)
        obstacles_group.add(obstacle)

def render_obstacles(obstacles_group):
    for obstacle in obstacles_group:
        screen.blit(obstacle.image, obstacle.rect)

def change_mode(enemies, behaviour):

    clear_enemies(enemies)

    behaviour = (behaviour + 1) % len(behaviour_list)
    
    return behaviour


def process_keys(keys, player_char):
    speed = 200.0 
    rotation_speed = 3.0  

    player_char.sprite.velocity[0] = 0
    player_char.sprite.velocity[1] = 0
    player_char.sprite.rotation = 0

    if keys[pygame.K_w]:
        player_char.sprite.velocity[0] = math.cos(player_char.sprite.orientation) * speed
        player_char.sprite.velocity[1] = math.sin(player_char.sprite.orientation) * speed
    if keys[pygame.K_s]:
        player_char.sprite.velocity[0] = -math.cos(player_char.sprite.orientation) * speed
        player_char.sprite.velocity[1] = -math.sin(player_char.sprite.orientation) * speed
    if keys[pygame.K_a]:
        player_char.sprite.rotation = -rotation_speed
    if keys[pygame.K_d]:
        player_char.sprite.rotation = rotation_speed


def spawn_enemy(enemies, behaviour, screensize):
    if behaviour == 14:
        # Aparecer en el lado izquierdo o derecho de la pantalla si es wall and obstacle avoidance
        spawn_side = random.choice(["left", "right"])

        if spawn_side == "left":
            # Spawnear en el lado izquierdo
            pos = (random.randint(20, 100), random.randint(50, screensize[1] - 50))
            # Destino en el lado derecho
            destination = [random.randint(700, 780), random.randint(50, screensize[1] - 50)]
        else:
            # Spawnear en el lado derecho
            pos = (random.randint(700, 780), random.randint(50, screensize[1] - 50))
            # Destino en el lado izquierdo
            destination = [random.randint(20, 100), random.randint(50, screensize[1] - 50)]

        # Crear el enemigo y asignarle el destino
        enemy = Enemy(behaviour, pos)
        enemy.destination = destination
        enemies.add(enemy)

    elif behaviour == 16:

        pos = (random.randint(0, screensize[0]), random.randint(0, screensize[1]))
        enemy = Enemy(behaviour=11, pos=pos)
        enemies.add(enemy)
        pos = (random.randint(0, screensize[0]), random.randint(0, screensize[1]))
        enemy = Enemy(behaviour=10, pos=pos)
        enemies.add(enemy)

    elif behaviour == 17:

        pos = (random.randint(0, screensize[0]), random.randint(0, screensize[1]))
        enemy = Enemy(behaviour=16, pos=pos)
        enemies.add(enemy)
        pos = (random.randint(0, screensize[0]), random.randint(0, screensize[1]))
        enemy = Enemy(behaviour=17, pos=pos)
        enemies.add(enemy)

    else:
        pos = (random.randint(0, screensize[0]), random.randint(0, screensize[1]))
        enemy = Enemy(behaviour, pos)
        enemies.add(enemy)

def game_loop():
    done = False
    player_char = pygame.sprite.GroupSingle(Player(screen.get_size()))
    enemies = pygame.sprite.Group()
    obstacles_group = pygame.sprite.Group()
    behaviour = 0

    while not done:
        timeDelta = clock.get_time() / 1000.0
        keys = pygame.key.get_pressed()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1:
                    behaviour = change_mode(enemies, behaviour)
                    if behaviour == 14:
                        create_obstacles(obstacles_group)
                    else:
                        obstacles_group.empty()
                elif event.key == pygame.K_2:
                    spawn_enemy(enemies, behaviour, screen.get_size())
                elif event.key == pygame.K_3:
                    clear_enemies(enemies)

        screen.fill(BGCOLOR)

        process_keys(keys, player_char)

        move_entities(player_char, enemies, obstacles_group, timeDelta)

        # Renderizar los obstaculos en behaviour 14
        if behaviour == 14:
            render_obstacles(obstacles_group)

        render_entities(player_char, enemies)

        behaviourRender = modeFont.render(behaviour_list[behaviour], True, pygame.Color('black'))
        behaviourRect = behaviourRender.get_rect()
        behaviourRect.right = size[0] - 20
        behaviourRect.top = 20
        screen.blit(behaviourRender, behaviourRect)

        pygame.display.flip()
        clock.tick(120)

    return done

done = game_loop()
while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    keys = pygame.key.get_pressed()
pygame.quit()

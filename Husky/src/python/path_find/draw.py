import pygame

pygame.init()
cell_size = 80

def desenha_mundo(map, destino):
    display = pygame.display.set_mode((len(map)*cell_size, len(map[0])*cell_size))
    display.fill((100,100,100))
    for j in range(len(map)):
        for i in range(len(map[0])):
            if map[j][i] == 1:
                pygame.draw.rect(display, "black", (i*cell_size, j*cell_size, cell_size, cell_size))
            else:
                pygame.draw.rect(display, "green", (i*cell_size, j*cell_size, cell_size, cell_size), width=1)
    pygame.draw.rect(display, "blue", (destino[0]*cell_size, destino[1]*cell_size, cell_size, cell_size))
    pygame.draw.circle(display, "red", (destino[0]*cell_size + cell_size/2, destino[1]*cell_size + cell_size/2),(cell_size/2)*0.6)
    return display


def tartaruga(inicio, lista, display):
    pygame.draw.line(display, "red", list(map(lambda x: x * cell_size + cell_size / 2, inicio)), list(map(lambda x: x * cell_size + cell_size / 2, lista[0])))
    for i in range(0, len(lista)-1):
        pygame.draw.line(display, "red", list(map(lambda x: x*cell_size + cell_size/2, lista[i])), list(map(lambda x: x*cell_size + cell_size/2, lista[i+1])))

    while True:
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                pygame.quit()
                exit()
        pygame.display.update()
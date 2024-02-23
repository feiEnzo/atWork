from draw import *
def solve(mapa, turtle_inicio, turtle_final):
    # atualize o mapa com o wavefront
    # O mapa pode ser do tamnho e proporcao desejada 8x8 9x4 25x100
    # a posicao de inicio e final podem ser alteradas
    mapa[turtle_final[1]][turtle_final[0]] = 2

    chek = [[0, 1], [-1, 0], [1, 0], [0, -1]]

    prox = [turtle_final]
    posx, posy = prox[0]
    while posx != turtle_inicio[0] or posy != turtle_inicio[1]:
        if prox:
            posx, posy = prox.pop(0)
            for i, j in chek:
                if posx + i < 0 or posx + i >= 19 or posy + j < 0 or posy + j >= 19:
                    pass
                elif mapa[posy + j][posx + i] == 0:
                    mapa[posy + j][posx + i] = mapa[posy][posx] + 1
                    prox.append([posx + i, posy + j])
        else:
            break

    # gere uma lista com a trajetória do robô
    caminho = [turtle_inicio]

    while caminho[len(caminho) - 1][0] != turtle_final[0] or caminho[len(caminho) - 1][1] != turtle_final[1]:
        postemp = []
        posx, posy = caminho[len(caminho) - 1]
        for i, j in chek:
            if posx + i < 0 or posx + i >= 19 or posy + j < 0 or posy + j >= 19:
                pass
            elif mapa[posy][posx] > mapa[posy + j][posx + i] > 1:
                postemp.append([posx + i, posy + j])
        caminho.append(postemp[[abs(turtle_final[0] - n[0]) + abs(turtle_final[1] - n[1]) for n in postemp].index(
            min([abs(turtle_final[0] - n[0]) + abs(turtle_final[1] - n[1]) for n in postemp]))])

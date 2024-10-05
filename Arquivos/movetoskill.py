import numpy as np
from reportlab.lib.validators import isBoolean

import VetOp
import testecolisao


def skill(x0, y0, xf, yf, pos, beta, alpha):
    #print(pos)
    vet = VetOp.calcularvetor(xf, yf, x0, y0)
    #print("inicio:")
    if np.linalg.norm(vet) == 0:
        vet = VetOp.normalizar(alpha, vet)
        return vet
    if testarcolisoes(x0, y0, vet, pos, beta):
        #print("colidiuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu")
        vet = pathfinding(x0, y0, xf, yf, pos, beta)
    if isBoolean(vet):
        return vet
    if alpha != -1:
        vet = VetOp.normalizar(alpha, vet)
    return vet



def pathfinding(x0, y0, xf, yf, pos, beta):
    vetores = []
    for i in range(int(len(pos)/2)):
        xr = pos.get("xr" + str(i))
        yr = pos.get("yr" + str(i))

    for i in range(int(len(pos)/2)):
        xr = pos.get("xr" + str(i))
        yr = pos.get("yr" + str(i))



        vet = VetOp.calcularnovovetor(x0, y0, xr, yr, beta[i], 1)
        if not testarcolisoes(x0, y0, vet, pos, beta):
            vetores.append(vet)

        vet = VetOp.calcularnovovetor(x0, y0, xr, yr, beta[i], -1)
        if not testarcolisoes(x0, y0, vet, pos, beta):
            vetores.append(vet)

    vet = VetOp.calcularvetor(xf, yf, x0, y0)
    angulosvet = []
    for i in range(0, len(vetores)):
        angulosvet.append(VetOp.angulovetores(vetores[i], vet))
    j = 0
    for i in range(1, len(vetores)):
        if abs(angulosvet[i]) < abs(angulosvet[j]):
            j = i
    if len(vetores) == 0:
        #print("sem solucoes encontradas") #teoricamente impossivel durante o jogo
        #return False
        return -vet

    #print(vetores)
    vet = vetores[j]

    #print(vet)
    return vet


def testarcolisoes(x0, y0, vet, pos, beta):
    for i in range(0, int(len(pos)/2)):
        xr = pos.get("xr" + str(i))
        yr = pos.get("yr" + str(i))
        if testecolisao.teste(vet, x0, y0, xr, yr, beta[i]):
            return True
    return False




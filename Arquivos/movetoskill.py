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

#PENALTI ---------------------------------------------------------------

# movetoskill.py
import numpy as np

def posicionar_bola_para_penalti(g):
    pos_penalti = (6, 0)
    id_robo_mais_proximo = robo_mais_proximo_da_bola(g)
    
    if id_robo_mais_proximo == -1:
        print("Nenhum robô encontrado.")
        return False
    
    robo = g.jogador_aliado[id_robo_mais_proximo]
    velocidade_baixa = 0.5
    
    while np.sqrt((robo[0] - pos_penalti[0])**2 + (robo[1] - pos_penalti[1])**2) > 0.1:
        robo = g.jogador_aliado[id_robo_mais_proximo]
        direcao_x = pos_penalti[0] - robo[0]
        direcao_y = pos_penalti[1] - robo[1]
        norma = np.sqrt(direcao_x**2 + direcao_y**2)
        direcao_x /= norma
        direcao_y /= norma
        
        mover_robo(id_robo_mais_proximo, velocidade_baixa * direcao_x, velocidade_baixa * direcao_y)
        r.sleep()

    print("Bola posicionada com sucesso!")
    return True

def mover_robo(id_robo, velocidade_x, velocidade_y):
    # Função fictícia para mover o robô. Ajuste-a conforme sua implementação de envio de comandos.
    comando = SSL()  # Definir o comando apropriado de movimentação para o robô
    comando.vel_x = velocidade_x
    comando.vel_y = velocidade_y
    pub[id_robo].publish(comando)

def robo_mais_proximo_da_bola(g): 
    menor_distancia = float('inf')
    id_robo_mais_proximo = -1
    pos_bola = g.p_bola
    
    for i in range(g.num_jogadores):
        robo = g.jogador_aliado[i]
        distancia = np.sqrt((robo[0] - pos_bola[0])**2 + (robo[1] - pos_bola[1])**2)
        
        if distancia < menor_distancia:
            menor_distancia = distancia
            id_robo_mais_proximo = i
    
    return id_robo_mais_proximo







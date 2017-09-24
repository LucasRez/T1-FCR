import sys
from grafo import *
from heap import Heap

def dijkstra(grafo, comeco, fim):
    h = Heap()
    lista = []

    #inicializa valores de distancia e cria a fila de prioridade
    for v in grafo.vertDict.keys():
        if v == comeco:
            lista.append((v, 0))
        else:
            lista.append((v, sys.maxint))
    h.buildHeap(lista)
    predecessores = {h.heapList[1][0]: None}
    visitados = []
    atual = (None,0)

    #calcula as distancias e guarda de onde foi calculada a menor distancia
    while not h.isEmpty():
        atual = h.delMin()
        visitados.append(atual[0])
        vAtual = grafo.getVertice(atual[0])
        for vProximo in vAtual.getConexoes():
            if vProximo.getId() not in visitados:
                novaDist = atual[1] + vAtual.getPeso(vProximo)
                proximo = [x for j, x in enumerate(h.heapList) if x[0] == vProximo.getId()][0]
                if novaDist < proximo[1]:
                    j = h.heapList.index(proximo)
                    h.heapList[j] = (proximo[0],novaDist)
                    h.percolaAcima(j)
                    predecessores[proximo[0]] = atual[0]

    #backtracking para achar o menor caminho
    a = fim
    caminho = []

    while a:
        if a not in predecessores:
            return []
        caminho = [a] + caminho
        a = predecessores[a]
    real_caminho = []
    for nodo in caminho:
        real_caminho.append(grafo.getVertice(nodo))
    return real_caminho


def kpaths(grafo, comeco, fim, k):
    g = grafo
    caminhos = []
    while k > 0:
        arestas = []
        naoRemover = []
        minAresta = None
        caminho = dijkstra(g, comeco, fim)
        for i in range(0, len(caminho) - 1):
            arestas.append(g.getAresta(caminho[i], caminho[i+1]))
        if arestas:
            caminhos.append(caminho)
            for aresta in arestas:
                if len(arestas) == 1:
                    break
                elif (aresta[0] != comeco and len(g.getVertice(aresta[0]).getConexoes()) <= 2) or (aresta[1] != fim and len(g.getVertice(aresta[1]).getConexoes()) <= 2):
                    naoRemover.append(aresta)
                elif aresta[0] == comeco and len(g.getVertice(comeco).getConexoes()) <= 1:
                    naoRemover.append(aresta)
                elif aresta[1] == fim and len(g.getVertice(fim).getConexoes()) <= 1:
                    naoRemover.append(aresta)
                else:
                    pass
            for aresta in naoRemover:
                if len(arestas) > 1:
                    arestas.remove(aresta)
            grafo.removeAresta(arestas[0][0], arestas[0][1])


        else:
            minAresta = None
        k = k - 1
        if caminho == []:
            break
    return caminhos


if __name__ == '__main__':

    g = load_map()

    caminho = dijkstra(g, 'H', 'K')

    for i in caminho:
        print i

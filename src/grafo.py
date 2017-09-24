class Vertice:
    def __init__(self, nodo, x, y, largura, altura):
        self.id = nodo
        self.adjacentes = {}
        self.x = x
        self.y = y
        self.largura = largura
        self.altura = altura

    def __str__(self):
        return self.id.__str__() + ' adjacentes: ' + [x.id for x in self.adjacentes].__str__() + '\nx: ' + self.x.__str__() + ' y: ' + self.y.__str__() + ' largura: ' + self.largura.__str__() + ' altura: ' + self.altura.__str__()

    def addVizinho(self, vizinho, peso = 1):
        self.adjacentes[vizinho] = peso

    def removeVizinho(self, vizinho):
        del self.adjacentes[vizinho]

    def getConexoes(self):
        return self.adjacentes.keys()

    def getId(self):
        return self.id

    def getPeso(self, vizinho):
        return self.adjacentes[vizinho]


class Grafo:
    def __init__(self):
        self.vertDict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vertDict.values())

    def addVertice(self, nodo):
        self.num_vertices = self.num_vertices + 1
        novoVertice = nodo
        self.vertDict[nodo.id] = novoVertice
        return novoVertice

    def getVertice(self, nodo):
        if nodo in self.vertDict:
            return self.vertDict[nodo]
        else:
            return None

    def addAresta(self, de, para, peso = 1):
        if de not in self.vertDict:
            self.addVertice(de)
        if para not in self.vertDict:
            self.addVertice(para)
        self.vertDict[de].addVizinho(self.vertDict[para], peso)
        self.vertDict[para].addVizinho(self.vertDict[de], peso)

    def removeAresta(self, de, para):
        if de in self.vertDict and para in self.vertDict:
            self.vertDict[de].removeVizinho(self.vertDict[para])
            self.vertDict[para].removeVizinho(self.vertDict[de])

    def getAresta(self, de, para):
        if de in self.vertDict and para in self.vertDict:
            return (de, para, self.getVertice(de).getPeso(self.vertDict[para]))
        else:
            return None

    def getVertices(self):
        return self.vertDict.keys()

def load_map():
    d_quad = 1.30 + 0.1
    y_mid = 7.14 + 0.1
    x_left = 10.99 + 0.1
    x_mid_right = 6.65 + 0.1
    x_corridor = 3.47 + 0.1

    mapa = Grafo()

    a = Vertice('A', -27.38, 17.67, d_quad, d_quad)
    b = Vertice('B', -27.20, 8.67, d_quad, y_mid)
    c = Vertice('C', -27.18, -0.19, d_quad, d_quad)
    d = Vertice('D', -14.09, 17.58, x_left, d_quad)
    e = Vertice('E', -14.01, -0.17, x_left, d_quad)
    f = Vertice('F', -0.96, 17.60, d_quad, d_quad)
    g = Vertice('G', -0.78, 8.75, d_quad, y_mid)
    h = Vertice('H', -0.50, -0.22, d_quad, d_quad)
    i = Vertice('I', 7.74, 17.76, x_mid_right, d_quad)
    j = Vertice('J', 8.05, -0.22, x_mid_right, d_quad)
    k = Vertice('K', 19.07, 17.83, x_corridor, d_quad)
    l = Vertice('L', 19.12, 8.69, x_corridor, y_mid)
    m = Vertice('M', 18.98, -0.27, x_corridor, d_quad)
    n = Vertice('N', 30.62, 17.74, x_mid_right, d_quad)
    o = Vertice('O', 30.20, -0.23, x_mid_right, d_quad)
    p = Vertice('P', 38.71, 17.72, d_quad, d_quad)
    q = Vertice('Q', 38.87, 8.86, d_quad, y_mid)
    r = Vertice('R', 38.61, -0.23, d_quad, d_quad)

    mapa.addVertice(a)
    mapa.addVertice(b)
    mapa.addVertice(c)
    mapa.addVertice(d)
    mapa.addVertice(e)
    mapa.addVertice(f)
    mapa.addVertice(g)
    mapa.addVertice(h)
    mapa.addVertice(i)
    mapa.addVertice(j)
    mapa.addVertice(k)
    mapa.addVertice(l)
    mapa.addVertice(m)
    mapa.addVertice(n)
    mapa.addVertice(o)
    mapa.addVertice(p)
    mapa.addVertice(q)
    mapa.addVertice(r)

    mapa.addAresta('A', 'B', 18)
    mapa.addAresta('A', 'D', 26)
    mapa.addAresta('D', 'F', 29)
    mapa.addAresta('F', 'G', 19)
    mapa.addAresta('F', 'I', 20)
    mapa.addAresta('I', 'K', 21)
    mapa.addAresta('K', 'L', 18)
    mapa.addAresta('K', 'N', 24)
    mapa.addAresta('N', 'P', 18)
    mapa.addAresta('P', 'Q', 18)
    mapa.addAresta('Q', 'R', 18)
    mapa.addAresta('R', 'O', 18)
    mapa.addAresta('O', 'M', 24)
    mapa.addAresta('M', 'L', 18)
    mapa.addAresta('M', 'J', 23)
    mapa.addAresta('J', 'H', 20)
    mapa.addAresta('H', 'G', 19)
    mapa.addAresta('H', 'E', 28)
    mapa.addAresta('E', 'C', 27)
    mapa.addAresta('C', 'B', 19)

    return mapa

def qual_nodo(mapa, x, y):
    for v in mapa.vertDict:
        if x > mapa.vertDict[v].x - mapa.vertDict[v].largura and x <= mapa.vertDict[v].x + mapa.vertDict[v].largura and y > mapa.vertDict[v].y - mapa.vertDict[v].altura and y <= mapa.vertDict[v].y + mapa.vertDict[v].altura:
            return mapa.vertDict[v]
    return None


if __name__ == '__main__':
    mapa = load_map()
    
    qual = qual_nodo(mapa, 0, 0)

    if qual:
        print(qual.id)
    else:
        print('unreachable')
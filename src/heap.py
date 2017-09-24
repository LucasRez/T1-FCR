class Heap:
    def __init__(self):
        self.heapList = [(0,0)]
        self.heapTam = 0

    def percolaAcima(self, pos):
        while pos // 2 > 0:
            if self.heapList[pos][1] < self.heapList[pos // 2][1]:
                temp = self.heapList[pos // 2]
                self.heapList[pos // 2] = self.heapList[pos]
                self.heapList[pos] = temp
            pos = pos // 2

    def insere(self, item):
        self.heapList.append(item)
        self.heapTam = self.heapTam + 1
        self.percolaAcima(self.heapTam)

    def menorFilho(self, pos):
        if pos * 2 + 1 > self.heapTam:
            return pos * 2
        elif self.heapList[pos * 2][1] < self.heapList[pos * 2 + 1][1]:
            return pos * 2
        else:
            return pos * 2 + 1

    def percolaAbaixo(self, pos):
        while pos * 2 <= self.heapTam:
            mf = self.menorFilho(pos)
            if self.heapList[pos][1] > self.heapList[mf][1]:
                temp =  self.heapList[pos]
                self.heapList[pos] = self.heapList[mf]
                self.heapList[mf] = temp
            pos = mf

    def delMin(self):
        ret = self.heapList[1]
        self.heapList[1] = self.heapList[self.heapTam]
        self.heapTam = self.heapTam - 1
        self.heapList.pop()
        self.percolaAbaixo(1)
        return ret

    def buildHeap(self, lista):
        i = len(lista) // 2
        self.heapTam = len(lista)
        self.heapList = [(0,0)] + lista[:]
        while(i > 0):
            self.percolaAbaixo(i)
            i = i - 1

    def isEmpty(self):
        return self.heapTam == 0



if __name__ == '__main__':
    h = Heap()

    h.buildHeap([('a', 46),('b', 1),('c', 9),('d', 5),('e', 3),('b', 2)])
    while (h.heapTam):
        print h.delMin()

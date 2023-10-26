import pygame
from maze import SearchSpace
from const import *
import time
import math 

def DFS(g: SearchSpace, sc: pygame.Surface):
    print('Implement DFS algorithm')

    open_set = [g.start.id]
    closed_set = []
    father = [-1]*g.get_length()
    
    g.grid_cells[g.goal.id]._set_color(PURPLE)
    g.grid_cells[g.start.id]._set_color(ORANGE)
    g.draw(sc)

    stack = [g.start.id]
    
    stop = False
    while stack:
        if stop:
           break
        current = stack[-1]

        g.grid_cells[current]._set_color(YELLOW)
        g.draw(sc)
        time.sleep(0.02)

        if current == g.goal.id:
            g.grid_cells[g.goal.id]._set_color(PURPLE)
            g.grid_cells[g.start.id]._set_color(ORANGE)
            g.draw(sc)
            time.sleep(0.01)
            stop = True
            break
        
        current = stack.pop()
        closed_set.append(current)
        open_set.remove(current)

        neighbor = g.get_neighbors(g.grid_cells[current])
        for i in neighbor:
            if i.id not in closed_set:
                g.grid_cells[i.id]._set_color(RED)
                g.draw(sc)
                stack.append(i.id)
                if i.id not in open_set:
                    open_set.append(i.id)
                father[i.id] = current

        g.grid_cells[current]._set_color(BLUE)
        g.draw(sc)
        pygame.display.flip() 
        pygame.time.delay(30)   

    drawPath(g, father, sc)

def BFS(g: SearchSpace, sc: pygame.Surface):
    print('Implement BFS algorithm')

    open_set = [g.start.id]
    closed_set = []
    father = [-1]*g.get_length()

    queue = [g.start.id]

    stop = False
    while queue:
        if stop:
            break
        current = queue[0]
        if current in closed_set:
            queue.pop(0)
            continue
        
        g.grid_cells[current]._set_color(YELLOW)
        g.draw(sc)
        time.sleep(0.02)

        if current == g.goal.id:
            g.grid_cells[g.goal.id]._set_color(PURPLE)
            g.grid_cells[g.start.id]._set_color(ORANGE)
            g.draw(sc)
            time.sleep(0.01)
            stop = True
            break

        closed_set.append(current)
        current = queue.pop(0)
        open_set.remove(current)

        neighbor = g.get_neighbors(g.grid_cells[current])
        for i in neighbor:
            if i.id not in closed_set and i.id not in open_set:
                g.grid_cells[i.id]._set_color(RED)
                g.draw(sc)
                if i.id not in open_set:
                    open_set.append(i.id)
                queue.append(i.id)
                father[i.id] = current

        g.grid_cells[current]._set_color(BLUE)
        g.draw(sc)
        pygame.display.flip()
        pygame.time.delay(30)   

    drawPath(g, father, sc)

def UCS(g: SearchSpace, sc: pygame.Surface):
    print('Implement UCS algorithm')
    # +1 respect if you can implement AStar with a priority queue

    open_set = [(0, g.start.id)]
    closed_set = []
    father = [-1]*g.get_length()
    cost = [100_000]*g.get_length()
    cost[g.start.id] = 0

    g.grid_cells[g.goal.id]._set_color(PURPLE)
    g.grid_cells[g.start.id]._set_color(ORANGE)
    g.draw(sc)

    while open_set:
        current_cost, id = min(open_set)
        open_set.remove((current_cost, id))
        
        if id == g.goal.id:
            g.grid_cells[g.goal.id]._set_color(PURPLE)
            g.grid_cells[g.start.id]._set_color(ORANGE)
            g.draw(sc)
            time.sleep(0.01)
            drawPath(g, father, sc)
            return

        if id in closed_set:
            continue

        g.grid_cells[id]._set_color(YELLOW)
        g.draw(sc)
        time.sleep(0.02)

        for neighbor in g.get_neighbors(g.grid_cells[id]):
            new_cost = cost[id] + 1  
            if new_cost < cost[neighbor.id]:

                g.grid_cells[neighbor.id]._set_color(RED)
                g.draw(sc)
                
                cost[neighbor.id] = new_cost
                open_set.append((new_cost, neighbor.id))
                father[neighbor.id] = id

        g.grid_cells[id]._set_color(BLUE)
        g.draw(sc)
        closed_set.append(id)
        pygame.time.delay(20)   

def Heuristic(id, g: SearchSpace, sc: pygame.Surface):
    return math.sqrt((g.grid_cells[id].rect.x - g.grid_cells[g.goal.id].rect.x)**2 + (g.grid_cells[id].rect.y - g.grid_cells[g.goal.id].rect.y)**2)

def AStar(g: SearchSpace, sc: pygame.Surface):
    print('Implement AStar algorithm')

    # +1 respect if you can implement AStar with a priority queue
    open_set = {}
    open_set[g.start.id] = 0
    closed_set: list[int] = []
    father = [-1]*g.get_length()
    cost = [100_000]*g.get_length()
    cost[g.start.id] = 0

    g.grid_cells[g.goal.id]._set_color(PURPLE)
    g.grid_cells[g.start.id]._set_color(ORANGE)
    g.draw(sc)

    id = g.start.id
    stop = False
    while id != g.goal.id and open_set:
        if stop:
            break

        g.grid_cells[id]._set_color(YELLOW)
        g.draw(sc)
        time.sleep(0.02)

        neighbors = {}
        for i in g.get_neighbors(g.grid_cells[id]):
            if i.id not in closed_set and i.id not in open_set:
                g.grid_cells[i.id]._set_color(RED)
                g.draw(sc)
                cost[i.id] += cost[id] + Heuristic(i.id, g, sc)
                open_set[i.id] = cost[i.id] 
                neighbors[i.id] = cost[i.id]
                father[i.id] = id

        g.grid_cells[id]._set_color(BLUE)
        g.draw(sc)
        pygame.time.delay(20)   
        pos = -1
        min_cost = 100000000000
        for i, val in neighbors.items():
            if val < min_cost and val != 0:
                min_cost = open_set.get(i)
                pos = i
        if pos == -1:
            pos = list(open_set.keys())[0]

        open_set.pop(pos, min_cost)
        closed_set.append(id)

        id = g.grid_cells[pos].id
        if id == g.goal.id:
            g.grid_cells[g.goal.id]._set_color(PURPLE)
            g.grid_cells[g.start.id]._set_color(ORANGE)
            g.draw(sc)
            time.sleep(0.01)
            stop = True
            break

    drawPath(g, father, sc)

def Greedy(g: SearchSpace, sc: pygame.Surface):
    print('Implement Greedy algorithm')

    open_set = {}
    open_set[g.start.id] = Heuristic(g.start.id, g, sc)
    closed_set = []
    father = [-1] * g.get_length()

    g.grid_cells[g.goal.id]._set_color(PURPLE)
    g.grid_cells[g.start.id]._set_color(ORANGE)
    g.draw(sc)

    id = g.start.id
    stop = False
    while id != g.goal.id and open_set:
        if stop:
            break

        g.grid_cells[id]._set_color(YELLOW)
        g.draw(sc)
        time.sleep(0.02)

        neighbors = {}
        neighbor_id = g.get_neighbors(g.grid_cells[id])
        for i in neighbor_id:
            if i.id not in closed_set and i.id not in open_set:

                g.grid_cells[i.id]._set_color(RED)
                g.draw(sc)

                open_set[i.id] = Heuristic(i.id, g, sc)
                neighbors[i.id] = open_set[i.id]
                father[i.id] = id

        g.grid_cells[id]._set_color(BLUE)
        g.draw(sc)
        pygame.time.delay(20)   
        open_set.pop(id, 0)
        closed_set.append(id)
        if open_set:
            id = min(open_set, key=open_set.get)
        else:
            id = g.goal.id

    if id == g.goal.id:
        g.grid_cells[g.goal.id]._set_color(PURPLE)
        g.grid_cells[g.start.id]._set_color(ORANGE)
        g.draw(sc)
        time.sleep(0.01)
        stop = True     
    drawPath(g, father, sc)

def Dijkstra(g: SearchSpace, sc: pygame.Surface):
    print('Implement Dijkstra algorithm')

    open_set = {}
    open_set[g.start.id] = 0
    closed_set = []
    father = [-1] * g.get_length()
    cost = [100_000] * g.get_length()
    cost[g.start.id] = 0

    g.grid_cells[g.goal.id]._set_color(PURPLE)
    g.grid_cells[g.start.id]._set_color(ORANGE)
    g.draw(sc)

    id = g.start.id
    stop = False
    while id != g.goal.id and open_set:
        if stop:
            break

        g.grid_cells[id]._set_color(YELLOW)
        g.draw(sc)
        time.sleep(0.02)

        neighbors = {}
        neighbor_id = g.get_neighbors(g.grid_cells[id])
        for i in neighbor_id:
            if i.id not in closed_set:
                g.grid_cells[i.id]._set_color(RED)
                g.draw(sc)
                if cost[id] + 1 < cost[i.id]:
                    cost[i.id] = cost[id] + 1
                    open_set[i.id] = cost[i.id]
                    neighbors[i.id] = cost[i.id]
                    father[i.id] = id
        g.grid_cells[id]._set_color(BLUE)
        g.draw(sc)
        open_set.pop(id, 0)
        closed_set.append(id)

        if open_set:
            id = min(open_set, key=open_set.get)
        else:
            id = g.goal.id

    if id == g.goal.id:
        g.grid_cells[g.goal.id]._set_color(PURPLE)
        g.grid_cells[g.start.id]._set_color(ORANGE)
        g.draw(sc)
        time.sleep(0.01)
        stop = True

    drawPath(g, father, sc)
    
def drawPath(g: SearchSpace, father, sc: pygame.Surface):
    path = []
    goalState = g.goal.id
    while goalState != g.start.id:
        path.append(goalState)
        goalState = father[goalState]

    path.append(g.start.id)
    path.reverse()
    g.draw(sc)

    for i in range(len(path) - 1):
        pygame.draw.line(sc, WHITE, (g.grid_cells[path[i]].rect.left + g.grid_cells[path[i]].rect.width / 2, g.grid_cells[path[i]].rect.top + g.grid_cells[path[i]].rect.height / 2), (g.grid_cells[path[i + 1]].rect.left + g.grid_cells[path[i + 1]].rect.width / 2, g.grid_cells[path[i + 1]].rect.top + g.grid_cells[path[i + 1]].rect.height / 2), 2)
        pygame.display.flip()
        time.sleep(0.05)
    
    

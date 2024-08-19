# Name: Ruben A Gonzalez
# PID: A17553307
# Short Description: This PA1 implements BFS[Queue], DFS[Stack], 
# UCS Dijkstras[Priority Queue], A* Dijkstras[Priority Queue] w/ Manhattan Distance Heuristics
# Documentation: For understanding of Heuristics https://www.geeksforgeeks.org/a-search-algorithm/
from __future__ import print_function
from heapq import * #Hint: Use heappop and heappush
# Documentation: https://www.geeksforgeeks.org/stack-in-python/ 
# Deque Implementation: https://www.geeksforgeeks.org/deque-in-python/
from collections import deque
#Reference: To A* Heuristic high level understanding inside PA1 Slack discussion H( (x,y) ) = |x-x_goal| + |y-y_goal| 
#Notes added in PA are both from pseudocode provided and personal documentation.

ACTIONS = [(0,1),(1,0),(0,-1),(-1,0)]

class AI:
    def __init__(self, grid, type):
        self.grid = grid
        self.set_type(type)
        self.set_search()

    def set_type(self, type):
        self.final_cost = 0
        self.type = type

    def set_search(self):
        self.final_cost = 0
        self.grid.reset()
        self.finished = False
        self.failed = False
        self.previous = {}

        # Initialization of algorithms goes here
        if self.type == "dfs":
            self.frontier = [self.grid.start]
            self.explored = []

        elif self.type == "bfs": #bfs Queue
            self.frontier = deque([self.grid.start])
            self.explored = []

        elif self.type == "ucs":
            #self.frontier = heappush(self.node,(0,self.grid.start))
            #self.explored = []
            #self.node = [self.grid.start,0] #(initital-state,path-cost)
            #self.frontier = [(0, self.grid.start)]
            self.frontier = []
            heappush(self.frontier,(0,self.grid.start))
            self.explored = []
    
        elif self.type == "astar":
            #self.node = [self.grid.start, 0] #replace 0 with G+H
            #self.frontier = [self.frontier.count,self.node] #pq[G+H,node]
            self.frontier = []
            G = 0
            #Reference: Slack discussion H( (x,y) ) = |x-x_goal| + |y-y_goal| 
            H = abs (self.grid.start[0] - self.grid.goal[0]) + abs (self.grid.start[1] - self.grid.goal[1])
            F = G+H
            heappush(self.frontier,(F, G, self.grid.start))
            self.explored = [] 
            
    def get_result(self):
        total_cost = 0
        current = self.grid.goal
        while not current == self.grid.start:
            total_cost += self.grid.nodes[current].cost()
            current = self.previous[current]
            self.grid.nodes[current].color_in_path = True #This turns the color of the node to red
        total_cost += self.grid.nodes[current].cost()
        self.final_cost = total_cost

    def make_step(self):
        if self.type == "dfs":
            self.dfs_step()
        elif self.type == "bfs":
            self.bfs_step()
        elif self.type == "ucs":
            self.ucs_step()
        elif self.type == "astar":
            self.astar_step()


    #DFS Algorithm [Stack]
    def dfs_step(self):

        #Base Case (Failed or Finished)
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        
        #Set top of stack into current [Add Start to stack]
        current = self.frontier.pop()
        #print(self.frontier)
        # Finishes search if we've found the goal. [While the stack is not empty]
        if current == self.grid.goal:
            self.finished = True
            return
        

        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS] #checks for children
        self.grid.nodes[current].color_checked = True #marks as checked (Visited)
        self.grid.nodes[current].color_frontier = False #frontier is now false

        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range): #n[x][y] Checks to see if it's within range
                if not self.grid.nodes[n].puddle and n not in self.explored: # Can't touch puddles and node is not already checked
                    self.previous[n] = current
                    self.frontier.append(n)
                    self.grid.nodes[n].color_frontier = True
                    self.explored.append(n)


    #BFS Algorithm [Queue]
    def bfs_step(self):
        #import pdb; pdb.set_trace()

        #Base Case
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        
        #Start of Code
        #current = heappop(self.frontier)
        current = self.frontier.popleft()


        if current == self.grid.goal:
            self.finished = True
            return

        #Gets all current's children
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS] 
        #End
        self.explored.append(current)
        self.grid.nodes[current].color_checked = True #Current node is marked as checked
        self.grid.nodes[current].color_frontier = False #Current node that used to be frontier is now false

        #Check neighbors
        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range): #n[x][y] Checks to see if it's within range
                if not self.grid.nodes[n].puddle and n not in self.explored: # Can't touch puddles and node is not already checked
                    self.previous[n] = current
                    self.frontier.append(n)
                    self.grid.nodes[n].color_frontier = True
                    self.explored.append(n)
               

    #UCS Using Dijkstra's Algorithm [Prioritiy Queue]
    def ucs_step(self):
        #import pdb; pdb.set_trace()

        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return

        #Everytime, pop the element with the minimum cost from the frontier.
        current_cost, current = heappop(self.frontier) #self.frontier = [self.grid.start,0]

        #If it is a goal state, terminate.
        if current == self.grid.goal:
            self.finished = True
            return
            
        self.explored.append(current) #add node.STATE to explored
        
        #For each action in problem.ACTION(node.STATE)do
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS] 
        self.grid.nodes[current].color_checked = True #Current node is marked as checked
        self.grid.nodes[current].color_frontier = False #Current node that used to be frontier is now false
        
        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range): #n[x][y] Checks to see if it's within range
                if not self.grid.nodes[n].puddle and n not in self.explored: # Can't touch puddles and node is not already checked
                    self.previous[n] = current
                    cost = current_cost + self.grid.nodes[n].cost() 
                    heappush(self.frontier,(cost,n)) #frontier <- INSERT(child, frontier)
                    self.grid.nodes[n].color_frontier = True
                    self.explored.append(n)
                        
                elif n in self.frontier and current_cost < cost:
                       self.previous[n] = current
                       self.grid.nodes[n].color_frontier = True
                       heapreplace(self.frontier, (current_cost,n))

    # A* Algorithm [Priority Queue]
    def astar_step(self):

        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return

        #Setting up a tuple of three [ F(G+H), G, Current_Node ]
        #Advice given on Slack thread (had some issues when only had two tuples)
        F, current_cost, current = heappop(self.frontier)

        if current == self.grid.goal:
            self.finished = True
            return
            
        self.explored.append(current) #add node.STATE to explored
        
        #For each action in problem.ACTION(node.STATE)do
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS] 
    
        self.grid.nodes[current].color_checked = True #Current node is marked as checked
        self.grid.nodes[current].color_frontier = False #Current node that used to be frontier is now false
        
        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range): #n[x][y] Checks to see if it's within range
                G = current_cost + self.grid.nodes[n].cost() #correct
                #H = self.final_cost
                H = abs (n[0] - self.grid.goal[0]) + abs (n[1]- self.grid.goal[1])
                F = G+H
                if not self.grid.nodes[n].puddle and n not in self.explored: # Can't touch puddles and node is not already checked
                    self.previous[n] = current
                    heappush(self.frontier,(F , G ,n)) #frontier <- INSERT(child, frontier)
                    self.grid.nodes[n].color_frontier = True
                    self.explored.append(n)

                elif n in self.frontier and current_cost < F:
                       self.previous[n] = current
                       self.grid.nodes[n].color_frontier = True
                       heapreplace(self.frontier, (F, G, n))

import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

import networkx as nx
from collections import *

CELLROWS=7
CELLCOLS=14
DIAMETER = 2

class MyRob(CRobLinkAngs):
    graph = nx.MultiGraph()
    not_visited_nodes = deque()
    visited_nodes = []
    path = []
    already_rotated = False
    first_loop = True
    initial_coordinates = []
    last_intersection_coordinates = []
    relative_coords = []
    intersections_count = 0
    go_back = False
    dirs = []
    last_dead_end = False
    
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.labMap = [[" " for col in range(49)] for row in range(21)]
        self.x=0
        self.y=0

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        self.readSensors()

        self.x=self.measures.x
        self.y=self.measures.y

        self.initial_coordinates = [self.measures.x, self.measures.y]

        # add root node
        # self.graph.add_node(nx.number_of_nodes(self.graph), pos = self.relative_to_intersection)

        self.last_intersection_coordinates = self.initial_coordinates

        next_node = None

        while self.not_visited_nodes != []:
            self.readSensors()

            self.MyMap()
            self.printMap()

            

            # detetar direções possiveis para a posição inicial
            # if self.first_loop:
            #     orientation = self.get_orientation(self.measures.compass)
            #     possible_directions = self.get_possible_directions(orientation)
            #     print(possible_directions)
            
            current_relative_coordinates = self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)
            print(f"current coordinates => {current_relative_coordinates}\n")
            # visitar o próximo nó
            # if it arrives to new possible intersection

            o = self.get_orientation(self.measures.compass)

            new_cell = False

            self.relative_coords = self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)
            self.relative_coords = [ float(round(x)) for x in self.relative_coords ]
            # self.relative_coords = [float(round(x)) for x in self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)]

            if o == 'N' and self.relative_coords[0] == self.last_intersection_coordinates[0]+2:
                new_cell = True
            elif o == 'S' and self.relative_coords[0] == self.last_intersection_coordinates[0]-2:
                new_cell = True
            elif o == 'W' and self.relative_coords[1] == self.last_intersection_coordinates[1]+2:
                new_cell = True
            elif o == 'E' and self.relative_coords[1] == self.last_intersection_coordinates[1]-2:
                new_cell = True

            print(f"relative_coords = {self.relative_coords}\n")
            print(f"last_intersection_coordinates = {self.last_intersection_coordinates}\n")
            print(f"new cell? {new_cell}\n")

            if (new_cell) or self.first_loop or (self.get_node_id_by_pos(self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)) in self.not_visited_nodes):
                print("--- NEW INTERSECTION ---")
                self.already_rotated = False
                # se nó ainda não existir
                #   adicionar nó ao grafo e à lista de nós visitados
                # se nó já existir no grafo
                #   adicionar à lista de nós visitados
                if not self.node_exists(self.relative_coords):
                    self.graph.add_node(nx.number_of_nodes(self.graph), pos = self.relative_coords)
                    self.path.append(self.get_node_id_by_pos(self.relative_coords))
                if not self.get_node_id_by_pos(self.relative_coords) in self.visited_nodes:
                    self.visited_nodes.append(self.get_node_id_by_pos(self.relative_coords))
                print("aqui")
                # current_node_id = self.get_node_id_by_pos(current_relative_coordinates)
                # self.visited_nodes.append(current_node_id)
                print("----------------- aqui -----------------")
                print(self.dirs)
                print("----------------------------------")
                if (self.measures.lineSensor == ['0', '0', '0', '0', '0', '0', '0'] and not self.last_dead_end and len(self.dirs) > 1) or self.first_loop:
                    print("1")
                    possible_directions = self.get_possible_directions(self.get_orientation(self.measures.compass), self.first_loop)
                    self.last_dead_end = True
                elif self.intersections_count > 0:
                    print("2")
                    possible_directions = self.dirs
                    if possible_directions == []:
                        print("2 - if")
                        possible_directions = self.get_possible_directions(self.get_orientation(self.measures.compass), self.first_loop)
                else:
                    print("3")
                    possible_directions = self.check_front(self.get_orientation(self.measures.compass))
                self.last_dead_end = False
                print(possible_directions)
                # se existirem nós sucessores
                if len(possible_directions) >= 1:
                    # adicionar os nós ao grafo e à lista de não visitados
                    for d in possible_directions:
                        # calcular atributo 'pos' do nó não visitado
                        if d == 'N':
                            if not self.node_exists([ self.relative_coords[0]+2, self.relative_coords[1] ]):
                                self.graph.add_node(nx.number_of_nodes(self.graph), pos = [ self.relative_coords[0]+2, self.relative_coords[1] ])
                                self.not_visited_nodes.append(nx.number_of_nodes(self.graph)-1)
                                self.graph.add_edge(self.get_node_id_by_pos(self.relative_coords), nx.number_of_nodes(self.graph)-1, dir='N')
                            elif not self.graph.has_edge(self.get_node_id_by_pos(self.relative_coords), self.get_node_id_by_pos([self.relative_coords[0]+2, self.relative_coords[1]])):
                                self.graph.add_edge(self.get_node_id_by_pos(self.relative_coords), self.get_node_id_by_pos([self.relative_coords[0]+2, self.relative_coords[1]]), dir='N')
                        elif d == 'S':
                            if not self.node_exists([ self.relative_coords[0]-2, self.relative_coords[1] ]):
                                self.graph.add_node(nx.number_of_nodes(self.graph), pos = [ self.relative_coords[0]-2, self.relative_coords[1] ])
                                self.not_visited_nodes.append(nx.number_of_nodes(self.graph)-1)
                                self.graph.add_edge(self.get_node_id_by_pos(self.relative_coords), nx.number_of_nodes(self.graph)-1, dir='S')
                            elif not self.graph.has_edge(self.get_node_id_by_pos(self.relative_coords), self.get_node_id_by_pos([self.relative_coords[0]-2, self.relative_coords[1]])):
                                self.graph.add_edge(self.get_node_id_by_pos(self.relative_coords), self.get_node_id_by_pos([self.relative_coords[0]-2, self.relative_coords[1]]), dir='S')
                        elif d == 'W':
                            if not self.node_exists([ self.relative_coords[0], self.relative_coords[1]+2 ]):
                                self.graph.add_node(nx.number_of_nodes(self.graph), pos = [ self.relative_coords[0], self.relative_coords[1]+2 ])
                                self.not_visited_nodes.append(nx.number_of_nodes(self.graph)-1)
                                self.graph.add_edge(self.get_node_id_by_pos(self.relative_coords), nx.number_of_nodes(self.graph)-1, dir='W')
                            elif not self.graph.has_edge(self.get_node_id_by_pos(self.relative_coords), self.get_node_id_by_pos([self.relative_coords[0], self.relative_coords[1]+2])):
                                self.graph.add_edge(self.get_node_id_by_pos(self.relative_coords), self.get_node_id_by_pos([self.relative_coords[0], self.relative_coords[1]+2]), dir='W')
                        elif d == 'E':
                            if not self.node_exists([ self.relative_coords[0], self.relative_coords[1]-2 ]):
                                self.graph.add_node(nx.number_of_nodes(self.graph), pos = [ self.relative_coords[0], self.relative_coords[1]-2 ])
                                self.not_visited_nodes.append(nx.number_of_nodes(self.graph)-1)
                                self.graph.add_edge(self.get_node_id_by_pos(self.relative_coords), nx.number_of_nodes(self.graph)-1, dir='E')
                            elif not self.graph.has_edge(self.get_node_id_by_pos(self.relative_coords), self.get_node_id_by_pos([self.relative_coords[0], self.relative_coords[1]-2])):
                                self.graph.add_edge(self.get_node_id_by_pos(self.relative_coords), self.get_node_id_by_pos([self.relative_coords[0], self.relative_coords[1]-2]), dir='E')

                    print(self.graph.nodes())
                    print(self.graph.edges())
                    print(nx.get_node_attributes(self.graph, 'pos'))
                    
                    print(f"not visited: {self.not_visited_nodes}\n")

                    # escolher o próximo nó, retirar no escolhido dos not visited
                    if len(self.not_visited_nodes) != 0:
                        next_node = self.not_visited_nodes.pop()
                    self.path.append(next_node)
                self.first_loop = False
            # se nao existirem nós sucessores
            else:
                #regressar ao nó predecessor
                print("go back")
                print(f"PATH: {self.path}\n")
                self.go_back = True
                possible_directions = self.get_possible_directions(self.get_orientation(self.measures.compass), self.first_loop)
                print(possible_directions)
                if len(possible_directions) >= 1:
                    # adicionar os nós ao grafo e à lista de não visitados
                    for d in possible_directions:
                        # calcular atributo 'pos' do nó não visitado
                        if d == 'N':
                            if not self.node_exists([ self.relative_coords[0]+2, self.relative_coords[1] ]):
                                self.graph.add_node(nx.number_of_nodes(self.graph), pos = [ self.relative_coords[0]+2, self.relative_coords[1] ])
                                self.not_visited_nodes.append(nx.number_of_nodes(self.graph)-1)
                                self.graph.add_edge(self.get_node_id_by_pos(self.relative_coords), nx.number_of_nodes(self.graph)-1, dir='N')
                            elif not self.graph.has_edge(self.get_node_id_by_pos(self.relative_coords), self.get_node_id_by_pos([self.relative_coords[0]+2, self.relative_coords[1]])):
                                self.graph.add_edge(self.get_node_id_by_pos(self.relative_coords), self.get_node_id_by_pos([self.relative_coords[0]+2, self.relative_coords[1]]), dir='N')
                        elif d == 'S':
                            if not self.node_exists([ self.relative_coords[0]-2, self.relative_coords[1] ]):
                                self.graph.add_node(nx.number_of_nodes(self.graph), pos = [ self.relative_coords[0]-2, self.relative_coords[1] ])
                                self.not_visited_nodes.append(nx.number_of_nodes(self.graph)-1)
                                self.graph.add_edge(self.get_node_id_by_pos(self.relative_coords), nx.number_of_nodes(self.graph)-1, dir='S')
                            elif not self.graph.has_edge(self.get_node_id_by_pos(self.relative_coords), self.get_node_id_by_pos([self.relative_coords[0]-2, self.relative_coords[1]])):
                                self.graph.add_edge(self.get_node_id_by_pos(self.relative_coords), self.get_node_id_by_pos([self.relative_coords[0]-2, self.relative_coords[1]]), dir='S')
                        elif d == 'W':
                            if not self.node_exists([ self.relative_coords[0], self.relative_coords[1]+2 ]):
                                self.graph.add_node(nx.number_of_nodes(self.graph), pos = [ self.relative_coords[0], self.relative_coords[1]+2 ])
                                self.not_visited_nodes.append(nx.number_of_nodes(self.graph)-1)
                                self.graph.add_edge(self.get_node_id_by_pos(self.relative_coords), nx.number_of_nodes(self.graph)-1, dir='W')
                            elif not self.graph.has_edge(self.get_node_id_by_pos(self.relative_coords), self.get_node_id_by_pos([self.relative_coords[0], self.relative_coords[1]+2])):
                                self.graph.add_edge(self.get_node_id_by_pos(self.relative_coords), self.get_node_id_by_pos([self.relative_coords[0], self.relative_coords[1]+2]), dir='W')
                        elif d == 'E':
                            if not self.node_exists([ self.relative_coords[0], self.relative_coords[1]-2 ]):
                                self.graph.add_node(nx.number_of_nodes(self.graph), pos = [ self.relative_coords[0], self.relative_coords[1]-2 ])
                                self.not_visited_nodes.append(nx.number_of_nodes(self.graph)-1)
                                self.graph.add_edge(self.get_node_id_by_pos(self.relative_coords), nx.number_of_nodes(self.graph)-1, dir='E')
                            elif not self.graph.has_edge(self.get_node_id_by_pos(self.relative_coords), self.get_node_id_by_pos([self.relative_coords[0], self.relative_coords[1]-2])):
                                self.graph.add_edge(self.get_node_id_by_pos(self.relative_coords), self.get_node_id_by_pos([self.relative_coords[0], self.relative_coords[1]-2]), dir='E')
                    if len(self.not_visited_nodes)>0:
                        next_node = self.not_visited_nodes.pop()
                        self.path.append(next_node)
                        print(self.graph.nodes())
                        print(self.graph.edges())
                        print(nx.get_node_attributes(self.graph, 'pos'))
                    else:
                        print("acabou")
                        self.driveMotors(0, 0)
                        self.finish()
                        break
                        
                self.go_back = False
            self.last_intersection_coordinates = [float(round(x)) for x in self.relative_coords]
            # self.last_intersection_coordinates = self.relative_coords
            print(f"not visited: {self.not_visited_nodes}\n")
            print(f"visited: {self.visited_nodes}\n")
            print(f"PATH: {self.path}\n")
            print(f"NEXT_NODE: {next_node}\n")
            print("------------------------")

            

            
            self.moving(self.relative_coords, next_node, self.get_orientation(self.measures.compass))
        
            # self.wander()
            # if self.measures.endLed:
            #     print(self.rob_name + " exiting")
            #     quit()

            # if state == 'stop' and self.measures.start:
            #     state = stopped_state

            # if state != 'stop' and self.measures.stop:
            #     stopped_state = state
            #     state = 'stop'

            # if state == 'run':
            #     if self.measures.visitingLed==True:
            #         state='wait'
            #     if self.measures.ground==0:
            #         self.setVisitingLed(True);
            #     self.wander()
            # elif state=='wait':
            #     self.setReturningLed(True)
            #     if self.measures.visitingLed==True:
            #         self.setVisitingLed(False)
            #     if self.measures.returningLed==True:
            #         state='return'
            #     self.driveMotors(0.0,0.0)
            # elif state=='return':
            #     if self.measures.visitingLed==True:
            #         self.setVisitingLed(False)
            #     if self.measures.returningLed==True:
            #         self.setReturningLed(False)
            #     self.wander()

    
    def get_node_id_by_pos(self, pos):
        for id in list(nx.get_node_attributes(self.graph, 'pos').keys()):
            if pos == nx.get_node_attributes(self.graph, 'pos')[id]:
                return id

    def get_node_pos_by_id(self, id):
        return nx.get_node_attributes(self.graph, 'pos')[id]

    def turn_right(self):
        self.driveMotors(0.01, -0.01)

    def turn_left(self):
        self.driveMotors(-0.01, 0.01)

    def turn_north(self):
        while self.measures.compass != 0:
            if (self.measures.compass <= 90) and (self.measures.compass >= 50):
                self.driveMotors(0.15, -0.15)
            elif (self.measures.compass < 50) and (self.measures.compass >= 20):
                self.driveMotors(0.04, -0.04)
            elif (self.measures.compass < 20) and (self.measures.compass >= 10):
                self.driveMotors(0.01, -0.01)
            elif (self.measures.compass < 10) and (self.measures.compass >= 0):
                self.driveMotors(0.005, -0.005)
            # elif (self.measures.compass < 5) and (self.measures.compass >= 0):
            #     self.driveMotors(0.001, -0.001)
            else:
                self.driveMotors(0.15, -0.15)
            print(self.measures.compass)
            self.readSensors()
        self.readSensors()
        print("turn_north")
        print(self.measures.compass)

    def turn_east(self):
        while self.measures.compass != -90:
            if (self.measures.compass <= 0) and (self.measures.compass >= -40):
                self.driveMotors(0.15, -0.15)
            elif (self.measures.compass < -40) and (self.measures.compass >= -70):
                self.driveMotors(0.04, -0.04)
            elif (self.measures.compass < -70) and (self.measures.compass >= -80):
                self.driveMotors(0.01, -0.01)
            elif (self.measures.compass < -80) and (self.measures.compass >= -90):
                self.driveMotors(0.005, -0.005)
            # elif (self.measures.compass < -85) and (self.measures.compass >= -90):
            #     self.driveMotors(0.001, -0.001)
            else:
                self.driveMotors(0.15, -0.15)
            self.readSensors()
        self.readSensors()
        print("turn_east")
        print(self.measures.compass)
    
    def turn_south(self):
        while self.measures.compass != 180 and self.measures.compass != -180:
            if (self.measures.compass <= -90) and (self.measures.compass >= -130):
                self.driveMotors(0.15, -0.15)
            elif (self.measures.compass < -130) and (self.measures.compass >= -160):
                self.driveMotors(0.04, -0.04)
            elif (self.measures.compass < -160) and (self.measures.compass >= -170):
                self.driveMotors(0.01, -0.01)
            elif (self.measures.compass < -170) and (self.measures.compass >= -180):
                self.driveMotors(0.005, -0.005)
            # elif (self.measures.compass < -175) and (self.measures.compass >= -180):
            #     self.driveMotors(0.001, -0.001)
            else:
                self.driveMotors(0.15, -0.15)
            self.readSensors()
        self.readSensors()
        print("turn_south")
        print(self.measures.compass)
    
    def turn_west(self):
        while self.measures.compass != 90:
            if (self.measures.compass <= 180) and (self.measures.compass >= 140):
                self.driveMotors(0.15, -0.15)
            elif (self.measures.compass < 140) and (self.measures.compass >= 110):
                self.driveMotors(0.04, -0.04)
            elif (self.measures.compass < 110) and (self.measures.compass >= 100):
                self.driveMotors(0.01, -0.01)
            elif (self.measures.compass < 100) and (self.measures.compass >= 90):
                self.driveMotors(0.005, -0.005)
            # elif (self.measures.compass < 95) and (self.measures.compass >= 90):
            #     self.driveMotors(0.001, -0.001)
            else:
                self.driveMotors(0.15, -0.15)
            self.readSensors()
        self.readSensors()
        print("turn_west")
        print(self.measures.compass)

    def moving(self, initial_pos, target_node, orientation):
        current_pos = self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)
        self.dirs = []
        print(self.get_node_id_by_pos(initial_pos))
        print(target_node)
        path = nx.shortest_path(self.graph, self.get_node_id_by_pos(initial_pos), target_node)
        print("path")
        print(path)

        aux_last_coords = []
        self.intersections_count = 0
        # path.pop(0)
        while path != []:
            
            aux_last_coords = [float(round(x)) for x in self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)]
            # aux_last_coords = self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)
            print(f"aux_last_coords => {aux_last_coords}\n")
            self.MyMap()

            tmp_dirs = []

            next_node = path.pop(0)
            target_pos = self.get_node_pos_by_id(next_node)
            print(f"next_node: {next_node} | target_pos: {target_pos}\n")
            print(f"poped path {path}\n")
            print(self.measures.compass)
            if target_pos[0] > aux_last_coords[0]:
                if self.measures.compass < 0 and self.measures.compass > -8:
                    print("rotating left")
                    while self.measures.compass != 0:
                        self.driveMotors(-0.01, 0.01)
                        self.readSensors()
                else:
                    print("rotating right")
                    self.turn_north()
                if self.measures.compass != 0:
                    while self.measures.compass > 0:
                        self.driveMotors(0.01, -0.01)
                        self.readSensors()
                        print(self.measures.compass)
                    while self.measures.compass < 0:
                        self.driveMotors(-0.01, 0.01)
                        self.readSensors()
                        print(self.measures.compass)
                    print(self.measures.compass)
                while target_pos[0]-0.5 > self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)[0]:
                    self.driveMotors(0.15, 0.15)
                    self.readSensors()
                    self.MyMap()
                    self.intersections_count += self.detect_intersection()
                    tmp_dirs.append(self.det_intersection(self.get_orientation(self.measures.compass)))
                while target_pos[0]-0.1 > self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)[0]:
                    self.driveMotors(0.05, 0.05)
                    self.readSensors()
                    self.MyMap()
                    self.intersections_count += self.detect_intersection()
                    tmp_dirs.append(self.det_intersection(self.get_orientation(self.measures.compass)))
                while target_pos[0]-0.05 > self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)[0]:
                    self.driveMotors(0.01, 0.01)
                    self.readSensors()
                    self.MyMap()
                    self.intersections_count += self.detect_intersection()
                    tmp_dirs.append(self.det_intersection(self.get_orientation(self.measures.compass)))
                    tmp_dirs.append(self.check_front(self.get_orientation(self.measures.compass)))
                while target_pos[0]-0.05 > self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)[0]:
                    self.driveMotors(0.005, 0.005)
                    self.readSensors()
                    self.MyMap()
                    self.intersections_count += self.detect_intersection()
                    tmp_dirs.append(self.det_intersection(self.get_orientation(self.measures.compass)))
                    tmp_dirs.append(self.check_front(self.get_orientation(self.measures.compass)))
            elif target_pos[0] < aux_last_coords[0]:
                # if self.measures.compass < 180 and self.measures.compass > 175:
                #     print("rotating left")
                    # while self.measures.compass != 180 and self.measures.compass != -180:
                    #     self.driveMotors(-0.01, 0.01)
                    #     self.readSensors()
                # else:
                self.turn_south()
                if self.measures.compass != 180 or self.measures.compass != -180:
                    if self.measures.compass < 0:
                        while self.measures.compass > -180:
                            self.driveMotors(0.01, -0.01)
                            self.readSensors()
                            print(self.measures.compass)
                    else:
                        while self.measures.compass < 180:
                            self.driveMotors(-0.01, 0.01)
                            self.readSensors()
                            print(self.measures.compass)
                while target_pos[0]+0.5 < self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)[0]:
                    self.driveMotors(0.15, 0.15)
                    self.readSensors()
                    self.MyMap()
                    self.intersections_count += self.detect_intersection()
                    tmp_dirs.append(self.det_intersection(self.get_orientation(self.measures.compass)))
                while target_pos[0]+0.1 < self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)[0]:
                    self.driveMotors(0.05, 0.05)
                    self.readSensors()
                    self.MyMap()
                    self.intersections_count += self.detect_intersection()
                    tmp_dirs.append(self.det_intersection(self.get_orientation(self.measures.compass)))
                while target_pos[0]+0.05 < self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)[0]:
                    self.driveMotors(0.01, 0.01)
                    self.readSensors()
                    self.MyMap()
                    self.intersections_count += self.detect_intersection()
                    tmp_dirs.append(self.det_intersection(self.get_orientation(self.measures.compass)))
                    tmp_dirs.append(self.check_front(self.get_orientation(self.measures.compass)))
                    tmp_dirs.append(self.check_front(self.get_orientation(self.measures.compass)))
                while target_pos[0] < self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)[0]:
                    self.driveMotors(0.005, 0.005)
                    self.readSensors()
                    self.MyMap()
                    self.intersections_count += self.detect_intersection()
                    tmp_dirs.append(self.det_intersection(self.get_orientation(self.measures.compass)))
                    tmp_dirs.append(self.check_front(self.get_orientation(self.measures.compass)))
            elif target_pos[1] > aux_last_coords[1]:
                if self.measures.compass < 90 and self.measures.compass > 82:
                    print("rotating left")
                    while self.measures.compass != 90:
                        self.driveMotors(-0.01, 0.01)
                        self.readSensors()
                else:
                    self.turn_west()
                if self.measures.compass != 90:
                    while self.measures.compass > 90:
                        self.driveMotors(0.01, -0.01)
                        self.readSensors()
                    while self.measures.compass < 90:
                        self.driveMotors(-0.01, 0.01)
                        self.readSensors()
                        print(self.measures.compass)
                while target_pos[1]-0.5 > self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)[1]:
                    self.driveMotors(0.15, 0.15)
                    self.readSensors()
                    self.MyMap()
                    self.intersections_count += self.detect_intersection()
                    tmp_dirs.append(self.det_intersection(self.get_orientation(self.measures.compass)))
                while target_pos[1]-0.1 > self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)[1]:
                    self.driveMotors(0.05, 0.05)
                    self.readSensors()
                    self.MyMap()
                    self.intersections_count += self.detect_intersection()
                    tmp_dirs.append(self.det_intersection(self.get_orientation(self.measures.compass)))
                while target_pos[1]-0.05 > self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)[1]:
                    self.driveMotors(0.01, 0.01)
                    self.readSensors()
                    self.MyMap()
                    self.intersections_count += self.detect_intersection()
                    tmp_dirs.append(self.det_intersection(self.get_orientation(self.measures.compass)))
                    tmp_dirs.append(self.check_front(self.get_orientation(self.measures.compass)))
                while target_pos[1] > self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)[1]:
                    self.driveMotors(0.005, 0.005)
                    self.readSensors()
                    self.MyMap()
                    self.intersections_count += self.detect_intersection()
                    tmp_dirs.append(self.det_intersection(self.get_orientation(self.measures.compass)))
                    tmp_dirs.append(self.check_front(self.get_orientation(self.measures.compass)))
            elif target_pos[1] < aux_last_coords[1]:
                if self.measures.compass < -92 and self.measures.compass > -90:
                    print("rotating left")
                    while self.measures.compass != -90:
                        self.driveMotors(-0.01, 0.01)
                        self.readSensors()
                else:
                    self.turn_east()
                if self.measures.compass != -90:
                    while self.measures.compass > -90:
                        self.driveMotors(0.01, -0.01)
                        self.readSensors()
                        print(self.measures.compass)
                    while self.measures.compass < -90:
                        self.driveMotors(-0.01, 0.01)
                        self.readSensors()
                        print(self.measures.compass)
                    print(self.measures.compass)
                while target_pos[1]+0.5 < self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)[1]:
                    self.driveMotors(0.15, 0.15)
                    self.readSensors()
                    self.MyMap()
                    self.intersections_count += self.detect_intersection()
                    tmp_dirs.append(self.det_intersection(self.get_orientation(self.measures.compass)))
                while target_pos[1]+0.1 < self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)[1]:
                    self.driveMotors(0.05, 0.05)
                    self.readSensors()
                    self.MyMap()
                    self.intersections_count += self.detect_intersection()
                    tmp_dirs.append(self.det_intersection(self.get_orientation(self.measures.compass)))
                while target_pos[1]+0.05 < self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)[1]:
                    self.driveMotors(0.01, 0.01)
                    self.readSensors()
                    self.MyMap()
                    self.intersections_count += self.detect_intersection()
                    tmp_dirs.append(self.det_intersection(self.get_orientation(self.measures.compass)))
                    tmp_dirs.append(self.check_front(self.get_orientation(self.measures.compass)))
                while target_pos[1] < self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)[1]:
                    self.driveMotors(0.005, 0.005)
                    self.readSensors()
                    self.MyMap()
                    self.intersections_count += self.detect_intersection()
                    tmp_dirs.append(self.det_intersection(self.get_orientation(self.measures.compass)))
                    tmp_dirs.append(self.check_front(self.get_orientation(self.measures.compass)))

            for x in tmp_dirs:
                for e in x:
                    if e not in self.dirs:
                        self.dirs.append(e)
            
            self.driveMotors(0,0)
            self.readSensors()
            print("moving")
            print(self.measures.compass)

            aux_last_coords = [float(round(x)) for x in self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)]
            # aux_last_coords = self.relative_coordinates(self.initial_coordinates, self.measures.x, self.measures.y)

            print("intersection count")
            print(self.intersections_count)
            print("AQUI")
            print(self.dirs)
            print("!!!!!!!!!!!!!")
        
    
    def check_front(self, orientation):
        if self.measures.lineSensor[3] == '1' or self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1':
            print(f"check front: {orientation}\n")
            return [orientation]
        return []
        
    def get_possible_directions(self, orientation, first_loop):
        res = []
        print("aqui")
        
        # self.driveMotors(0.12, 0.12)
        # self.readSensors()
        print(orientation)
        print(self.measures.compass)
        if orientation == 'N':
            # if self.measures.compass != 0:
            #     while self.measures.compass > 0:
            #         self.driveMotors(0.005, -0.005)
            #         self.readSensors()
            #         print(self.measures.compass)
            #     while self.measures.compass < 0:
            #         self.driveMotors(-0.005, 0.005)
            #         self.readSensors()
            #         print(self.measures.compass)
            #     print(self.measures.compass)
            #verificar N
            for i in range(0,2):
                if self.measures.lineSensor[3] == '1' or self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1':
                    res.append('N')
                self.readSensors()
            print("aqui")
            # verficar E
            self.turn_east()
            self.readSensors()
            for i in range(0,2):
                if self.measures.lineSensor[3] == '1' or self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1':
                    res.append('E')
                self.readSensors()
            if first_loop:
                # verificar S
                self.turn_south()
                self.readSensors()
                for i in range(0,2):
                    if self.measures.lineSensor[3] == '1' or self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1':
                        res.append('S')
                    self.readSensors()
            else:
                res.append('S')
                self.readSensors()
            # verificar W
            self.turn_west()
            self.readSensors()
            for i in range(0,2):
                if self.measures.lineSensor[3] == '1' or self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1':
                    res.append('W')
                self.readSensors()
            # voltar para direção inicial (N)
            # self.turn_north()
            # self.readSensors()
            # if self.measures.compass != 0:
            #     while self.measures.compass > 0:
            #         self.driveMotors(0.005, -0.005)
            #         self.readSensors()
            #         print(self.measures.compass)
            #     while self.measures.compass < 0:
            #         self.driveMotors(-0.005, 0.005)
            #         self.readSensors()
            #         print(self.measures.compass)
            #     print("returned to initial position(N)")
            #     print(self.measures.compass)
        elif orientation == 'E':
            # if self.measures.compass != -90:
            #     while self.measures.compass > -90:
            #         self.driveMotors(0.005, -0.005)
            #         self.readSensors()
            #         print(self.measures.compass)
            #     while self.measures.compass < -90:
            #         self.driveMotors(-0.005, 0.005)
            #         self.readSensors()
            #         print(self.measures.compass)
            #verificar E
            for i in range(0,2):
                if self.measures.lineSensor[3] == '1' or self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1':
                    res.append('E')
                self.readSensors()
            # verificar S
            self.turn_south()
            self.readSensors()
            for i in range(0,2):
                if self.measures.lineSensor[3] == '1' or self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1':
                    res.append('S')
                self.readSensors()
            if first_loop:
            # verificar W
                self.turn_west()
                self.readSensors()
                for i in range(0,2):
                    if self.measures.lineSensor[3] == '1' or self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1':
                        res.append('W')
                    self.readSensors()
            else:
                res.append('W')
                self.readSensors()
            # verficar N
            self.turn_north()
            self.readSensors()
            for i in range(0,2):
                if self.measures.lineSensor[3] == '1' or self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1':
                    res.append('N')
                self.readSensors()
            # voltar para direção inicial (E)
            # self.turn_east()
            # self.readSensors()
            # if self.measures.compass != -90:
            #     while self.measures.compass > -90:
            #         self.driveMotors(0.005, -0.005)
            #         self.readSensors()
            #         print(self.measures.compass)
            #     while self.measures.compass < -90:
            #         self.driveMotors(-0.005, 0.005)
            #         self.readSensors()
            #         print(self.measures.compass)
            #     print("returned to initial position(E)")
            #     print(self.measures.compass)

        elif orientation == 'S':
            #verificar S
            for i in range(0,2):
                if self.measures.lineSensor[3] == '1' or self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1':
                    res.append('S')
                self.readSensors()
            # verificar W
            self.turn_west()
            self.readSensors()
            for i in range(0,2):
                if self.measures.lineSensor[3] == '1' or self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1':
                    res.append('W')
                self.readSensors()
            if first_loop:
                # verificar N
                self.turn_north()
                self.readSensors()
                for i in range(0,2):
                    if self.measures.lineSensor[3] == '1' or self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1':
                        res.append('N')
                    self.readSensors()
            else:
                res.append('N')
                self.readSensors()
            # verficar E
            self.turn_east()
            self.readSensors()
            for i in range(0,2):
                if self.measures.lineSensor[3] == '1' or self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1':
                    res.append('E')
                self.readSensors()
            # # voltar para direção inicial (S)
            # self.turn_south()
            # self.readSensors()
        elif orientation == 'W':
            # if self.measures.compass != 90:
            #     while self.measures.compass > 90:
            #         self.driveMotors(-0.005, 0.005)
            #         self.readSensors()
            #         print(self.measures.compass)
            #     while self.measures.compass < 90:
            #         self.driveMotors(-0.005, 0.005)
            #         self.readSensors()
            #         print(self.measures.compass)
            #verificar W
            for i in range(0,2):
                if self.measures.lineSensor[3] == '1' or self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1':
                    res.append('W')
                self.readSensors()
            # verificar N
            self.turn_north()
            self.readSensors()
            for i in range(0,2):
                if self.measures.lineSensor[3] == '1' or self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1':
                    res.append('N')
                self.readSensors()
            if first_loop:
                # verficar E
                self.turn_east()
                self.readSensors()
                for i in range(0,2):
                    if self.measures.lineSensor[3] == '1' or self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1':
                        res.append('E')
                    self.readSensors()
            else:
                res.append('E')
                self.readSensors()
            # verificar S
            self.turn_south()
            self.readSensors()
            for i in range(0,2):
                if self.measures.lineSensor[3] == '1' or self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1':
                    res.append('S')
                self.readSensors()
            # voltar para direção inicial (W)
            # self.turn_west()
            # self.readSensors()
            # if self.measures.compass != 90:
            #     while self.measures.compass > 90:
            #         self.driveMotors(-0.005, 0.005)
            #         self.readSensors()
            #         print(self.measures.compass)
            #     while self.measures.compass < 90:
            #         self.driveMotors(-0.005, 0.005)
            #         self.readSensors()
            #         print(self.measures.compass)
            #     print("returned to initial position(N)")
            #     print(self.measures.compass)

        return list(set(res))


    def wander(self):
        # center_id = 0
        # left_id = 1
        # right_id = 2
        # back_id = 3
        # if    self.measures.irSensor[center_id] > 5.0\
        #    or self.measures.irSensor[left_id]   > 5.0\
        #    or self.measures.irSensor[right_id]  > 5.0\
        #    or self.measures.irSensor[back_id]   > 5.0:
        #     #print('Rotate left')
        #     self.driveMotors(-0.1,+0.1)
        # elif self.measures.irSensor[left_id]> 2.7:
        #     #print('Rotate slowly right')
        #     self.driveMotors(0.1,0.0)
        # elif self.measures.irSensor[right_id]> 2.7:
        #     #print('Rotate slowly left')
        #     self.driveMotors(0.0,0.1)
        # else:
        #     #print('Go')
        #     self.driveMotors(0.1,0.1)

        # flag=False

        ## new implementation
        
        # if self.measures.lineSensor[6] == '1':
        #     self.driveMotors(0.15, -0.1)
        #     # self.graph.add_node(nx.number_of_nodes(self.graph) + 1, pos=(self.rel_pos[0], self.rel_pos[1]))

        # elif self.measures.lineSensor[0] == '1':
        #     self.driveMotors(-0.1, 0.15)
        # else:
        #     self.driveMotors(0.09, 0.09)

        ##


        # if self.next_move == "":
        if self.measures.lineSensor[6] == '1':
            self.driveMotors(0.15,-0.1)
        elif self.measures.lineSensor[0] == '1':
            self.driveMotors(-0.1,0.15)
        elif self.measures.lineSensor[0] == '0' and self.measures.lineSensor[1] == '1' :              
            self.driveMotors(0.09,0.08)
        elif self.measures.lineSensor[3] == '1' and self.measures.lineSensor[4] == '1' :               
            self.driveMotors(0.08, 0.09)
        elif self.measures.lineSensor == ['0','0','0','0','0','0','0']:                                                         
            self.driveMotors(-0.14,-0.14)
            # self.dead_ends+=1
            # if self.dead_ends>= 3:
            #     self.driveMotors(0.15,-0.1)
            #     self.dead_ends-=1
        else:
            self.driveMotors(0.09,0.09)


    # returns the possible directions based on the lineSensor
    def detect_intersection(self):
        res = []
        if self.measures.lineSensor[0] == '1' and self.measures.lineSensor[1] == '1':
            res.append('left')
        if self.measures.lineSensor[6] == '1' and self.measures.lineSensor[5] == '1':
            res.append('right')

        # if self.measures.lineSensor[3] == '1' and (self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1'):
        #     res.append('front')
        if len(res) != 0:
            return 1
        return 0

    def det_intersection(self, orientation):
        res = []
        print(self.measures.lineSensor)
        if orientation == 'N':
            if self.measures.lineSensor[0] == '1' and self.measures.lineSensor[1] == '1':
                res.append('W')
            if self.measures.lineSensor[6] == '1' and self.measures.lineSensor[5] == '1':
                res.append('E')

        if orientation == 'S':
            if self.measures.lineSensor[0] == '1' and self.measures.lineSensor[1] == '1':
                res.append('E')
            if self.measures.lineSensor[6] == '1' and self.measures.lineSensor[5] == '1':
                res.append('W')

        if orientation == 'W':
            if self.measures.lineSensor[0] == '1' and self.measures.lineSensor[1] == '1':
                res.append('S')
            if self.measures.lineSensor[6] == '1' and self.measures.lineSensor[5] == '1':
                res.append('N')

        if orientation == 'E':
            if self.measures.lineSensor[0] == '1' and self.measures.lineSensor[1] == '1':
                res.append('N')
            if self.measures.lineSensor[6] == '1' and self.measures.lineSensor[5] == '1':
                res.append('S')


        return list(set(res))

    # returns the relative to intersections coordinates
    def relative_to_intersection(self, init_coords, x, y, compass):
        # orientation = self.get_orientation(compass)
        # if orientation == 'N' or orientation == 'W':
        #     return [round((x-init_coords[0]-0.5)/DIAMETER), round((y-init_coords[1]-0.5)/DIAMETER)]
        # else:
        #     return [round((x-init_coords[0]+0.5)/DIAMETER), round((y-init_coords[1]+0.5)/DIAMETER)]
        return [ int((x-init_coords[0])/DIAMETER), int((y-init_coords[1])/DIAMETER) ]

    def relative_coordinates(self, init_coords, x, y):
        return [ float(round((x-init_coords[0]),1)), float(round((y-init_coords[1]),1)) ]

    # returns orientation of the robot based on ['N', 'W', 'S', 'E']
    # it rounds the sensor reads to give the orientation
    def get_orientation(self, compass):
        quarter = round(compass/90)
        if quarter == 0:
            return 'N'
        elif quarter == 1:
            return 'W'
        elif quarter == -1:
            return 'E'
        else:
            return 'S'

    # returns the orientation based on the current robot's orientation and the given direction
    def relative_orientation(self, direction, orientation):
        if direction == 'front':
            return orientation
        else:
            if orientation == 'N':
                if direction == 'left':
                    return 'W'
                elif direction == 'right':
                    return 'E'
            elif orientation == 'W':
                if direction == 'left':
                    return 'S'
                elif direction == 'right':
                    return 'N'
            elif orientation == 'S':
                if direction == 'left':
                    return 'E'
                elif direction == 'right':
                    return 'W'
            elif orientation == 'E':
                if direction == 'left':
                    return 'N'
                elif direction == 'right':
                    return 'S'

    def node_exists(self, pos):
        return pos in list(nx.get_node_attributes(self.graph, 'pos').values())

    #funtion to get the gps coordinates and convert to matrix coordinates
    def MyMap(self):
        xval=int(round(self.measures.x-self.x))
        print(xval)
        yval=int(round(self.measures.y-self.y))
        print(yval)
        self.labMap[10][24]="I"
        if(yval+10)%2 == 0:
                if(xval+24-1)%2 ==0:
                    carater = "-"
                else:
                    carater = " "
        else:
            carater = "|"
        self.labMap[yval+10][xval+24]=carater
        self.write_map()
    
    def printMap(self):
        print('|----------------------MAP------------------------|')
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))
        print('|----------------------MAP-----------------------|')

    def write_map(self):
        f = open("out_map.txt", "w")
        for l in reversed(self.labMap):
            s = ''.join([str(l) for l in l])
            f.write(s+'\n')
        f.close()

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1

rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()

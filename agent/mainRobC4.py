import itertools
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

from decimal import Decimal
import networkx as nx
from queue import *
from collections import *

CELLROWS = 7
CELLCOLS = 14


class MyRob(CRobLinkAngs):

    # graph variables
    graph = nx.MultiGraph()
    not_visited_nodes = []
    visited_nodes = []
    # not_visited_nodes = deque()
    not_visited_nodes = []

    current_node = None
    last_node = None
    next_node = None

    # movement model variables
    already_reseted_movement_model = 0

    # movement algorithm variables
    priority = None
    rotate_half = None
    path = []
    back_to_keep_line = False
    target_angle = None

    # detecting intersection variables
    rotating = False
    rotating_to = None
    waiting_on_line = 0

    state = 2

    # beacons variables
    num_beacons = None
    beacons_pos = {0: (0, 0)}

    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.labMapOld = [[" " for col in range(49)] for row in range(21)]
        self.labMap = [[" " for col in range(49)] for row in range(21)]
        self.px = 0
        self.py = 0
        self.pa = 0

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

        self.add_node((self.px, self.py))
        self.not_visited_nodes.pop()
        self.visited_nodes.append(nx.number_of_nodes(self.graph)-1)
        self.current_node = nx.number_of_nodes(self.graph)-1
        self.last_node = self.current_node

        self.readSensors()
        initial_coordinates = [self.measures.x, self.measures.y]

        while True:
            self.counter = 0
            self.readSensors()

            self.num_beacons = int(self.nBeacons)

            if int(self.simTime) - int(self.measures.time) <= 1: 
                self.myMap2()
                self.shortest_path_with_beacons()
                self.finish()

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed == True:
                    state = 'wait'
                if self.measures.ground == 0:
                    self.setVisitingLed(True)
            elif state == 'wait':
                self.setReturningLed(True)
                if self.measures.visitingLed == True:
                    self.setVisitingLed(False)
                if self.measures.returningLed == True:
                    state = 'return'
                # self.driveMotors(0.0,0.0)
            elif state == 'return':
                if self.measures.visitingLed == True:
                    self.setVisitingLed(False)
                if self.measures.returningLed == True:
                    self.setReturningLed(False)

                

                print("-------------------")
                print(f"[CURRENT COORDINATES]: {(self.px, self.py)}")
                print(f'Real Coordinates: {self.measures.x-initial_coordinates[0]}, {self.measures.y-initial_coordinates[1]}')
                print(
                    f'[CURRENT ANGLE]: {self.radians_to_degrees(self.pa)} => radians: {self.pa}\n')
                print(f'Compass: {self.measures.compass}\n')
                # print(f'Converted Compass: {self.convert_compass()}\n')
                print("--- printing graph ---")
                print(self.graph.nodes())
                print(self.graph.edges())
                print(nx.get_node_attributes(self.graph, 'pos'))
                print("--- end of graph ---")
                print(
                    f"[CURRENT NODE]: {self.current_node} pos: {self.get_node_pos_by_id(self.current_node)}\n")
                print(
                    f'[LAST NODE]: {self.last_node} pos: {self.get_node_pos_by_id(self.last_node)}\n')
                print(
                    f"[NEXT NODE]: {self.next_node} pos: {self.get_node_pos_by_id(self.next_node)}\n")
                print(f"[PATH]: {self.path}\n")
                print(f'[NOT VISITED NODES]: {self.not_visited_nodes}\n')
                print(f'[VISITED NODES]: {self.visited_nodes}\n')
                print(f'[STATE]: {self.state}\n')
                print(f'[BEACONS POS]: {self.beacons_pos}\n')
                print("-------------------")

                # 1) go in front until reset movement model
                if self.state == 0:
                    self.movement()
                # 2) when reset movement model reduce velocity and detect sides
                elif self.state == 1:
                    self.centering()
                    self.detect_sides()

                # 3) center and check front
                elif self.state == 2:
                    # self.convert_compass()
                    # if self.target_angle != None:
                    #     self.rotate()
                    for i in range(2):
                        self.check_front()
                        self.readSensors()
                    self.adjust_angle()
                    if len(self.path) > 0:
                        self.next_node = self.path.pop(0)
                    elif len(self.not_visited_nodes) > 0:
                        # aux = self.not_visited_nodes.pop()

                        # chose the closest not visited node
                        possible_paths = []
                        for node in self.not_visited_nodes:
                            possible_paths.append(nx.shortest_path(self.graph, self.current_node, node))
                        # get the index of the shortest path, if there are more than one minimum path, get the last one
                        # index = possible_paths.index(min(possible_paths, key=len))
                        index = len(possible_paths) - 1 - possible_paths[::-1].index(min(possible_paths, key=len))
                        aux = self.not_visited_nodes.pop(index)
                        print(f'aux: {aux}')
                        print(f'shortest path: {possible_paths[index]}')

                        self.path = nx.shortest_path(self.graph, self.current_node, aux)
                        print(self.path)
                        self.path.pop(0)  # remove current node
                        self.next_node = self.path.pop(0)
                    elif len(self.path) == 0 and len(self.not_visited_nodes) == 0 and self.current_node != 0:
                        self.path = nx.shortest_path(self.graph, self.current_node, 0)
                        self.path.pop(0)
                        self.next_node = self.path.pop(0)
                    else:
                        self.put_beacons_on_map()
                        self.myMap2()
                        self.calcPos(0, 0)
                        self.shortest_path_with_beacons()
                        self.finish()
                        break
                    self.state = 3
                    self.set_target_angle()
                # 4) rotate to next node
                elif self.state == 3:
                    self.rotate()

    def adjust_angle(self):
        current_orientation = self.get_orientation(self.measures.compass)

        if current_orientation == 'N':
            self.pa = self.degrees_to_radians(0)
        elif current_orientation == 'W':
            self.pa = self.degrees_to_radians(90)
        elif current_orientation == 'S':
            self.pa = self.degrees_to_radians(180)
        elif current_orientation == 'E':
            self.pa = self.degrees_to_radians(270)

    def shortest_path_with_beacons(self):
        paths = []
        best_path = []
        beacons = []
        for beacon_pos in self.beacons_pos.values():
            beacons.append(self.get_node_id_by_pos(beacon_pos))
        combinations = itertools.permutations(beacons, r=self.num_beacons)
        for combination in combinations:
            path = [0]
            start = 0
            beacons_comb = [0] + list(combination)
            for b in beacons_comb:
                end = b
                tmp = nx.shortest_path(self.graph, start, end)
                tmp.pop(0)
                path += tmp
                start = end
            end = 0
            aux = nx.shortest_path(self.graph, start, end)
            aux.pop(0)
            path += aux
            paths.append(path)

        for path in paths:
            if len(path) < len(best_path) or len(best_path) == 0:
                best_path = path

        f = open("solution.path", "w")
        for node in best_path:
            pos = self.get_node_pos_by_id(node)
            f.write(str(pos[0]) + ' ' + str(pos[1]) + '\n')
        f.close()

    def set_target_angle(self):
        current_node_pos = self.get_node_pos_by_id(self.current_node)
        next_node_pos = self.get_node_pos_by_id(self.next_node)

        if next_node_pos[0] > current_node_pos[0]:
            self.target_angle = 0
        elif next_node_pos[0] < current_node_pos[0]:
            self.target_angle = 180
        elif next_node_pos[1] > current_node_pos[1]:
            self.target_angle = 90
        elif next_node_pos[1] < current_node_pos[1]:
            self.target_angle = 270

    def rotate(self):
        print("--- Rotating ---")
        print(f"[TARGET ANGLE]: {self.target_angle}\n")
        print(f"[CURRENT ANGLE]: {round(self.radians_to_degrees(self.pa))}\n")
        if self.target_angle != round(self.radians_to_degrees(self.pa)) and self.target_angle != round(self.radians_to_degrees(self.pa)-1):
            if self.target_angle > round(self.radians_to_degrees(self.pa)):
                if round(self.radians_to_degrees(self.pa)) < self.target_angle - 20:
                    self.calcPos(-0.15, 0.15)
                else:
                    self.calcPos(-0.01, 0.01)
            else:
                if round(self.radians_to_degrees(self.pa)) > self.target_angle + 20:
                    self.calcPos(0.15, -0.15)
                else:
                    self.calcPos(0.01, -0.01)
        else:
            self.state = 0
            self.calcPos(0, 0)

    def degrees_to_radians(self, degrees):
        return degrees * (pi/180)

    def check_front(self):
        current_node_pos = self.get_node_pos_by_id(self.current_node)
        current_orientation = self.get_orientation(self.measures.compass)

        if (self.measures.lineSensor[2] == '1' or self.measures.lineSensor[4] == '1') and self.measures.lineSensor[3] == '1':
            if current_orientation == 'N':
                self.add_node((current_node_pos[0]+2, current_node_pos[1]))
                self.add_edge(self.current_node, self.get_node_id_by_pos((current_node_pos[0]+2, current_node_pos[1])))
                # if not self.graph.has_edge(self.current_node, nx.number_of_nodes(self.graph)-1):
                #     self.graph.add_edge(self.current_node, nx.number_of_nodes(self.graph)-1)
            elif current_orientation == 'S':
                self.add_node((current_node_pos[0]-2, current_node_pos[1]))
                self.add_edge(self.current_node, self.get_node_id_by_pos((current_node_pos[0]-2, current_node_pos[1])))
                # if not self.graph.has_edge(self.current_node, nx.number_of_nodes(self.graph)-1):
                #     self.graph.add_edge(self.current_node, nx.number_of_nodes(self.graph)-1)
            elif current_orientation == 'E':
                self.add_node((current_node_pos[0], current_node_pos[1]-2))
                self.add_edge(self.current_node, self.get_node_id_by_pos((current_node_pos[0], current_node_pos[1]-2)))
                # if not self.graph.has_edge(self.current_node, nx.number_of_nodes(self.graph)-1):
                #     self.graph.add_edge(self.current_node, nx.number_of_nodes(self.graph)-1)
            elif current_orientation == 'W':
                self.add_node((current_node_pos[0], current_node_pos[1]+2))
                self.add_edge(self.current_node, self.get_node_id_by_pos((current_node_pos[0], current_node_pos[1]+2)))
                # if not self.graph.has_edge(self.current_node, nx.number_of_nodes(self.graph)-1):
                #     self.graph.add_edge(self.current_node, nx.number_of_nodes(self.graph)-1)

    def detect_sides(self):
        current_node_pos = self.get_node_pos_by_id(self.current_node)
        current_orientation = self.get_orientation(self.measures.compass)
        if self.measures.lineSensor[0] == '1' and self.measures.lineSensor[1] == '1':
            self.readSensors()
            if self.measures.lineSensor[0] == '1' and self.measures.lineSensor[1] == '1':
                if current_orientation == 'N':
                    self.add_node((current_node_pos[0], current_node_pos[1]+2))

                    self.add_edge(self.current_node, self.get_node_id_by_pos((current_node_pos[0], current_node_pos[1]+2)))

                    # if not self.graph.has_edge(self.current_node, nx.number_of_nodes(self.graph)-1):
                    #     self.graph.add_edge(self.current_node, nx.number_of_nodes(self.graph)-1)
                elif current_orientation == 'S':
                    self.add_node((current_node_pos[0], current_node_pos[1]-2))

                    self.add_edge(self.current_node, self.get_node_id_by_pos(
                        (current_node_pos[0], current_node_pos[1]-2)))
                    # if not self.graph.has_edge(self.current_node, nx.number_of_nodes(self.graph)-1):
                    #     self.graph.add_edge(self.current_node, nx.number_of_nodes(self.graph)-1)
                elif current_orientation == 'E':
                    self.add_node((current_node_pos[0]+2, current_node_pos[1]))

                    self.add_edge(self.current_node, self.get_node_id_by_pos(
                        (current_node_pos[0]+2, current_node_pos[1])))
                    # if not self.graph.has_edge(self.current_node, nx.number_of_nodes(self.graph)-1):
                    #     self.graph.add_edge(self.current_node, nx.number_of_nodes(self.graph)-1)
                elif current_orientation == 'W':
                    self.add_node((current_node_pos[0]-2, current_node_pos[1]))

                    self.add_edge(self.current_node, self.get_node_id_by_pos(
                        (current_node_pos[0]-2, current_node_pos[1])))
                    # if not self.graph.has_edge(self.current_node, nx.number_of_nodes(self.graph)-1):
                    #     self.graph.add_edge(self.current_node, nx.number_of_nodes(self.graph)-1)
            else:
                print("False positive")

        if self.measures.lineSensor[5] == '1' and self.measures.lineSensor[6] == '1':
            self.readSensors()
            if self.measures.lineSensor[5] == '1' and self.measures.lineSensor[6] == '1':
                if current_orientation == 'N':
                    self.add_node((current_node_pos[0], current_node_pos[1]-2))

                    self.add_edge(self.current_node, self.get_node_id_by_pos(
                        (current_node_pos[0], current_node_pos[1]-2)))
                    # if not self.graph.has_edge(self.current_node, nx.number_of_nodes(self.graph)-1):
                    #     self.graph.add_edge(self.current_node, nx.number_of_nodes(self.graph)-1)
                elif current_orientation == 'S':
                    self.add_node((current_node_pos[0], current_node_pos[1]+2))

                    self.add_edge(self.current_node, self.get_node_id_by_pos(
                        (current_node_pos[0], current_node_pos[1]+2)))
                    # if not self.graph.has_edge(self.current_node, nx.number_of_nodes(self.graph)-1):
                    #     self.graph.add_edge(self.current_node, nx.number_of_nodes(self.graph)-1)
                elif current_orientation == 'E':
                    self.add_node((current_node_pos[0]-2, current_node_pos[1]))

                    self.add_edge(self.current_node, self.get_node_id_by_pos(
                        (current_node_pos[0]-2, current_node_pos[1])))
                    # if not self.graph.has_edge(self.current_node, nx.number_of_nodes(self.graph)-1):
                    #     self.graph.add_edge(self.current_node, nx.number_of_nodes(self.graph)-1)
                elif current_orientation == 'W':
                    self.add_node((current_node_pos[0]+2, current_node_pos[1]))

                    self.add_edge(self.current_node, self.get_node_id_by_pos(
                        (current_node_pos[0]+2, current_node_pos[1])))
                    # if not self.graph.has_edge(self.current_node, nx.number_of_nodes(self.graph)-1):
                    # self.graph.add_edge(self.current_node, nx.number_of_nodes(self.graph)-1)
            else:
                print("fake side detected")

    def centering(self):
        current_orientation = self.get_orientation(self.measures.compass)
        # print(f"Coordinates centering x: {round(self.px,1)} y:{round(self.py,1)} orientation: {current_orientation}\n")
        # print(f"Coordinates centering x: {self.px} y:{self.py} orientation: {current_orientation}\n")

        if (current_orientation == 'N' and (round(self.px, 1)) % 2 == 0) or (current_orientation == 'S' and (round(self.px, 1)) % 2 == 0) or (current_orientation == 'E' and (round(self.py, 1)) % 2 == 0) or (current_orientation == 'W' and (round(self.py, 1)) % 2 == 0):
            self.calcPos(0, 0)
            self.state = 2
            if self.current_node not in self.visited_nodes:
                self.visited_nodes.append(self.current_node)
            if self.measures.ground != -1 and self.get_node_pos_by_id(self.current_node) not in self.beacons_pos:
                # print("beacon new pos => " +
                #       str(self.get_node_pos_by_id(self.current_node)))
                self.beacons_pos[self.measures.ground] = self.get_node_pos_by_id(self.current_node)
            # print(f'Beacon? {self.measures.ground}')
            # self.px = round(self.px/2)*2
            # self.py = round(self.py/2)*2
            # self.check_front()
        else:
            self.calcPos(0.02, 0.02)

    # makes the robot follow the line

    def movement(self):
        # rotate left softly
        if self.measures.lineSensor[1] == '1' or self.measures.lineSensor[0] == '1':
            self.calcPos(0.05, 0.08)
            self.rotating = False
        # rotate right softly
        elif self.measures.lineSensor[5] == '1' or self.measures.lineSensor[6] == '1':
            self.calcPos(0.08, 0.05)
            self.rotating = False
        # # rotate left
        # elif self.measures.lineSensor[0] == '1':
        #     self.calcPos(-0.01, 0.03)
        #     self.rotating_to = 'left'
        #     self.rotating = True
        # # rotate right
        # elif self.measures.lineSensor[6] == '1':
        #     self.calcPos(0.03, -0.01)
        #     self.rotating_to = 'right'
        #     self.rotating = True
        # elif self.measures.lineSensor == ['0', '0', '0', '0', '0', '0', '0']:
        #     self.calcPos(-0.14, -0.14)
        # go in front
        else:
            
            self.rotating = False
            self.rotating_to = None
            self.calcPos(0.08, 0.08)

    def move(self):
        print("-------------------")
        print(f"[CURRENT COORDINATES]: {(self.px, self.py)}")
        print(
            f'[CURRENT ANGLE]: {self.radians_to_degrees(self.pa)} => radians: {self.pa}\n')
        print("--- printing graph ---")
        print(self.graph.nodes())
        print(self.graph.edges())
        print(nx.get_node_attributes(self.graph, 'pos'))
        print("--- end of graph ---")
        print(
            f"[CURRENT NODE]: {self.current_node} pos: {self.get_node_pos_by_id(self.current_node)}\n")
        print(
            f'[LAST NODE]: {self.last_node} pos: {self.get_node_pos_by_id(self.last_node)}\n')
        print(
            f"[NEXT NODE]: {self.next_node} pos: {self.get_node_pos_by_id(self.next_node)}\n")
        print(f"[PATH]: {self.path}\n")
        print(f'[NOT VISITED NODES]: {self.not_visited_nodes}\n')
        print(f'[VISITED NODES]: {self.visited_nodes}\n')
        print("-------------------")

        # se chegar a um nó diferente
        if self.last_node != self.current_node:
            # se for um nó não visitado e se houver nós descendentes do nó atual ir para o último da lista de nós não visitados
            if len(self.not_visited_nodes) > 0 and self.current_node not in self.visited_nodes:
                self.last_node = self.current_node
                self.next_node = self.not_visited_nodes.pop()

                # ir para o próximo nó
                current_orientation = self.get_orientation(
                    self.measures.compass)

                current_node_pos = self.get_node_pos_by_id(self.current_node)
                next_node_pos = self.get_node_pos_by_id(self.next_node)

                if current_orientation == 'N':
                    if next_node_pos[0] > current_node_pos[0]:
                        self.priority = 'front'
                    elif next_node_pos[0] < current_node_pos[0]:
                        self.priority = 'back'
                        self.rotate_half = 180
                    elif next_node_pos[1] > current_node_pos[1]:
                        self.priority = 'left'
                    elif next_node_pos[1] < current_node_pos[1]:
                        self.priority = 'right'
                elif current_orientation == 'S':
                    if next_node_pos[0] > current_node_pos[0]:
                        self.priority = 'back'
                        self.rotate_half = 0
                    elif next_node_pos[0] < current_node_pos[0]:
                        self.priority = 'front'
                    elif next_node_pos[1] > current_node_pos[1]:
                        self.priority = 'right'
                    elif next_node_pos[1] < current_node_pos[1]:
                        self.priority = 'left'
                elif current_orientation == 'E':
                    if next_node_pos[0] > current_node_pos[0]:
                        self.priority = 'right'
                    elif next_node_pos[0] < current_node_pos[0]:
                        self.priority = 'left'
                    elif next_node_pos[1] > current_node_pos[1]:
                        self.priority = 'front'
                    elif next_node_pos[1] < current_node_pos[1]:
                        self.priority = 'back'
                        self.rotate_half = 90
                elif current_orientation == 'W':
                    if next_node_pos[0] > current_node_pos[0]:
                        self.priority = 'left'
                    elif next_node_pos[0] < current_node_pos[0]:
                        self.priority = 'right'
                    elif next_node_pos[1] > current_node_pos[1]:
                        self.priority = 'back'
                        self.rotate_half = 270
                    elif next_node_pos[1] < current_node_pos[1]:
                        self.priority = 'front'
            elif len(self.not_visited_nodes) == 0:
                self.last_node = self.current_node
                # get shortest path to start node
                self.path = nx.shortest_path(self.graph, self.current_node, 0)
                # self.next_node = self.start_node

                print("no not visited nodes =>  make shortest path")
                print(f"shortest path: {self.path}")
                print("no descendent nodes =>  make shortest path")

        print("-------------")
        print("-------------")
        print("-------------")
        print(f"Priority: {self.priority}\n")
        print("-------------")
        print("-------------")
        print("-------------")

        self.go_to_next_node()

        # if self.not_visited_nodes != []:
        #     if self.last_node != self.current_node or self.next_node == None:
        #         if self.current_node == self.next_node or self.next_node == None or self.last_node in self.visited_nodes:
        #             self.last_node = self.current_node
        #             self.next_node = self.not_visited_nodes.pop()
        #             print("aqui oh borro")
        #             current_orientation = self.get_orientation(self.measures.compass)

        #             current_node_pos = self.get_node_pos_by_id(self.current_node)
        #             next_node_pos = self.get_node_pos_by_id(self.next_node)

        #             if current_orientation == 'N':
        #                 if next_node_pos[0] > current_node_pos[0]:
        #                     self.priority = 'front'
        #                 elif next_node_pos[0] < current_node_pos[0]:
        #                     self.priority = 'back'
        #                     self.rotate_half = 180
        #                 elif next_node_pos[1] > current_node_pos[1]:
        #                     self.priority = 'left'
        #                 elif next_node_pos[1] < current_node_pos[1]:
        #                     self.priority = 'right'
        #             elif current_orientation == 'S':
        #                 if next_node_pos[0] > current_node_pos[0]:
        #                     self.priority = 'back'
        #                     self.rotate_half = 0
        #                 elif next_node_pos[0] < current_node_pos[0]:
        #                     self.priority = 'front'
        #                 elif next_node_pos[1] > current_node_pos[1]:
        #                     self.priority = 'right'
        #                 elif next_node_pos[1] < current_node_pos[1]:
        #                     self.priority = 'left'
        #             elif current_orientation == 'E':
        #                 if next_node_pos[0] > current_node_pos[0]:
        #                     self.priority = 'right'
        #                 elif next_node_pos[0] < current_node_pos[0]:
        #                     self.priority = 'left'
        #                 elif next_node_pos[1] > current_node_pos[1]:
        #                     self.priority = 'front'
        #                 elif next_node_pos[1] < current_node_pos[1]:
        #                     self.priority = 'back'
        #                     self.rotate_half = 90
        #             elif current_orientation == 'W':
        #                 if next_node_pos[0] > current_node_pos[0]:
        #                     self.priority = 'left'
        #                 elif next_node_pos[0] < current_node_pos[0]:
        #                     self.priority = 'right'
        #                 elif next_node_pos[1] > current_node_pos[1]:
        #                     self.priority = 'back'
        #                     self.rotate_half = 270
        #                 elif next_node_pos[1] < current_node_pos[1]:
        #                     self.priority = 'front'

        #     print("-------------")
        #     print("-------------")
        #     print("-------------")
        #     print(f"Priority: {self.priority}\n")
        #     print("-------------")
        #     print("-------------")
        #     print("-------------")

        #     self.go_to_next_node()

    def detect_intersection(self):
        # print(f'Rotating_to: {self.rotating_to}\n')

        if self.rotating_to == 'left' and self.measures.lineSensor[6] == '1' and self.measures.lineSensor[5] == '1':
            self.readSensors()
            if self.measures.lineSensor[6] == '1' and self.measures.lineSensor[5] == '1':
                print("intersection in front (rotating to left)")
                current_node_pos = self.get_node_pos_by_id(self.current_node)
                current_orientation = self.get_orientation(
                    self.measures.compass)
                if current_orientation == 'N':
                    self.add_node((current_node_pos[0]+2, current_node_pos[1]))
                elif current_orientation == 'S':
                    self.add_node((current_node_pos[0]-2, current_node_pos[1]))
                elif current_orientation == 'E':
                    self.add_node((current_node_pos[0], current_node_pos[1]-2))
                elif current_orientation == 'W':
                    self.add_node((current_node_pos[0], current_node_pos[1]+2))
            else:
                print("fake intersection in front (rotating to left)")
        elif self.rotating_to == 'right' and self.measures.lineSensor[0] == '1' and self.measures.lineSensor[1] == '1':
            self.readSensors()
            if self.measures.lineSensor[0] == '1' and self.measures.lineSensor[1] == '1':
                print("intersection in front (rotating to right)")
                current_node_pos = self.get_node_pos_by_id(self.current_node)
                current_orientation = self.get_orientation(
                    self.measures.compass)
                if current_orientation == 'N':
                    self.add_node((current_node_pos[0]+2, current_node_pos[1]))
                elif current_orientation == 'S':
                    self.add_node((current_node_pos[0]-2, current_node_pos[1]))
                elif current_orientation == 'E':
                    self.add_node((current_node_pos[0], current_node_pos[1]-2))
                elif current_orientation == 'W':
                    self.add_node((current_node_pos[0], current_node_pos[1]+2))
            else:
                print("fake intersection in front (rotating to right)")
        elif self.rotating_to == None:
            if self.measures.lineSensor[0] == '1' and self.measures.lineSensor[1] == '1' and self.measures.lineSensor[6] == '1' and self.measures.lineSensor[5] == '1':
                current_node_pos = self.get_node_pos_by_id(self.current_node)
                current_orientation = self.get_orientation(
                    self.measures.compass)
                print('intersection on both sides')
                if current_orientation == 'N' or current_orientation == 'S':
                    self.add_node((current_node_pos[0], current_node_pos[1]+2))
                    self.add_node((current_node_pos[0], current_node_pos[1]-2))
                elif current_orientation == 'E' or current_orientation == 'W':
                    self.add_node((current_node_pos[0]+2, current_node_pos[1]))
                    self.add_node((current_node_pos[0]-2, current_node_pos[1]))
            elif self.measures.lineSensor[0] == '1' and self.measures.lineSensor[1] == '1':
                print("intersection on left")
                current_node_pos = self.get_node_pos_by_id(self.current_node)
                current_orientation = self.get_orientation(
                    self.measures.compass)
                if current_orientation == 'N':
                    self.add_node((current_node_pos[0], current_node_pos[1]+2))
                elif current_orientation == 'S':
                    self.add_node((current_node_pos[0], current_node_pos[1]-2))
                elif current_orientation == 'E':
                    self.add_node((current_node_pos[0]+2, current_node_pos[1]))
                elif current_orientation == 'W':
                    self.add_node((current_node_pos[0]-2, current_node_pos[1]))
            elif self.measures.lineSensor[6] == '1' and self.measures.lineSensor[5] == '1':
                print("intersection on right")
                current_node_pos = self.get_node_pos_by_id(self.current_node)
                current_orientation = self.get_orientation(
                    self.measures.compass)
                if current_orientation == 'N':
                    self.add_node((current_node_pos[0], current_node_pos[1]-2))
                elif current_orientation == 'S':
                    self.add_node((current_node_pos[0], current_node_pos[1]+2))
                elif current_orientation == 'E':
                    self.add_node((current_node_pos[0]-2, current_node_pos[1]))
                elif current_orientation == 'W':
                    self.add_node((current_node_pos[0]+2, current_node_pos[1]))

    def go_to_next_node(self):
        if self.priority == 'left':
            if self.back_to_keep_line:
                self.calcPos(0.15, -0.15)
                self.back_to_keep_line = False
            # rotate left
            elif self.measures.lineSensor[0] == '1':
                self.calcPos(-0.01, 0.05)
                self.rotating_to = 'left'
            # rotate left softly
            elif self.measures.lineSensor[1] == '1' and self.measures.lineSensor[5] == '0':
                self.calcPos(0.03, 0.05)
            # rotate right softly
            elif self.measures.lineSensor[1] == '0' and self.measures.lineSensor[5] == '1':
                self.calcPos(0.05, 0.03)
            # go back to keep in line
            elif self.measures.lineSensor == ['0', '0', '0', '0', '0', '0', '0'] and not self.back_to_keep_line:
                self.calcPos(-0.14, -0.14)
                self.back_to_keep_line = True
            # go in front
            else:
                self.calcPos(0.05, 0.05)
        elif self.priority == 'right':
            if self.back_to_keep_line:
                self.calcPos(-0.15, 0.15)
                self.back_to_keep_line = False
            # rotate right
            elif self.measures.lineSensor[6] == '1':
                self.calcPos(0.05, -0.01)
                self.rotating_to = 'right'
            # rotate left softly
            elif self.measures.lineSensor[1] == '1' and self.measures.lineSensor[5] == '0':
                self.calcPos(0.03, 0.05)
            # rotate right softly
            elif self.measures.lineSensor[1] == '0' and self.measures.lineSensor[5] == '1':
                self.calcPos(0.05, 0.03)
            # go back to keep in line
            elif self.measures.lineSensor == ['0', '0', '0', '0', '0', '0', '0'] and not self.back_to_keep_line:
                self.calcPos(-0.14, -0.14)
                self.back_to_keep_line = True
            # go in front
            else:
                self.calcPos(0.05, 0.05)
        elif self.priority == 'front':
            if self.back_to_keep_line:
                self.calcPos(-0.15, -0.15)
                self.back_to_keep_line = False
            # rotate left softly
            elif self.measures.lineSensor[1] == '1' and self.measures.lineSensor[5] == '0':
                self.calcPos(0.03, 0.05)
            # rotate right softly
            elif self.measures.lineSensor[1] == '0' and self.measures.lineSensor[5] == '1':
                self.calcPos(0.05, 0.03)
            # go back to keep in line
            elif self.measures.lineSensor == ['0', '0', '0', '0', '0', '0', '0'] and not self.back_to_keep_line:
                self.calcPos(-0.14, -0.14)
                self.back_to_keep_line = True
            # go in front
            else:
                self.calcPos(0.05, 0.05)

        elif self.priority == 'back':
            if self.back_to_keep_line:
                self.calcPos(-0.15, -0.15)
                self.back_to_keep_line = False
            elif self.rotate_half != round(self.radians_to_degrees(self.pa)) and self.rotate_half != round(self.radians_to_degrees(self.pa)-1):
                if self.rotate_half > round(self.radians_to_degrees(self.pa)):
                    self.calcPos(-0.01, 0.01)
                else:
                    self.calcPos(0.01, -0.01)
            else:
                # rotate left softly
                if self.measures.lineSensor[1] == '1' and self.measures.lineSensor[5] == '0':
                    self.calcPos(0.03, 0.05)
                # rotate right softly
                elif self.measures.lineSensor[1] == '0' and self.measures.lineSensor[5] == '1':
                    self.calcPos(0.05, 0.03)
                # go back to keep in line
                elif self.measures.lineSensor == ['0', '0', '0', '0', '0', '0', '0'] and not self.back_to_keep_line:
                    self.calcPos(-0.14, -0.14)
                    self.back_to_keep_line = True
                # go in front
                else:
                    self.calcPos(0.05, 0.05)
        else:
            self.calcPos(0.05, 0.05)

    def radians_to_degrees(self, radians):
        return (radians * 180) / pi

    def add_node(self, pos):
        pos = (round(pos[0]/2)*2, round(pos[1]/2)*2)
        if not self.node_exists(pos):
            # print("adding node: " + str(pos))
            self.graph.add_node(nx.number_of_nodes(self.graph), pos=pos)
            # print("--- printig graph ---")
            # print(self.graph.nodes())
            # print(self.graph.edges())
            # print(nx.get_node_attributes(self.graph, 'pos'))
            # print("--- end of graph ---")
            if self.get_node_id_by_pos(pos) not in self.not_visited_nodes:
                self.not_visited_nodes.append(self.get_node_id_by_pos(pos))

            if not self.graph.has_edge(self.current_node, nx.number_of_nodes(self.graph)-1) and nx.number_of_nodes(self.graph) > 1:
                self.graph.add_edge(self.current_node,
                                    nx.number_of_nodes(self.graph)-1)
                # print(
                #     f"added edge => {self.current_node} to {nx.number_of_nodes(self.graph)-1}")
            return True

        return False

    def add_edge(self, node1, node2):
        if not self.graph.has_edge(node1, node2):
            self.graph.add_edge(node1, node2)
            return True
        return False

    def node_exists(self, pos):
        return pos in list(nx.get_node_attributes(self.graph, 'pos').values())

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

    def calcPos(self, l, r):
        self.driveMotors(l, r)
        lin = (l+r)/2

        self.px = self.px+lin*cos((self.pa))

        self.py = self.py+lin*sin((self.pa))

        rot = r-l

        # aux = round(self.measures.compass/90)*90
        # if aux == 360:
        #     aux = 270
        # elif aux == -90:
        #     aux = 270
        # elif aux == -180:
        #     aux = 180
        # self.pa = self.degrees_to_radians(aux) * 0.3 + self.pa * 0.7

        self.pa = self.pa + rot

        # self.pa = 0.2 * self.degrees_to_radians(self.measures.compass) + 0.8 * (self.pa + rot)

        current_orientation = self.get_orientation(self.measures.compass)

        if self.already_reseted_movement_model == 0:
            if ((Decimal((self.px + 0.438 - 0.1) % 2).quantize(Decimal('0.1')) >= 1.9) and (Decimal((self.px + 0.438 + 0.1) % 2).quantize(Decimal('0.1')) <= 0.1) and current_orientation == 'N'):
                self.px = round(self.px) - 0.438
                self.py = round(self.py)
                self.already_reseted_movement_model = 10
                self.add_node((self.px+0.438, self.py))
                # self.add_visited_node(nx.number_of_nodes(self.graph)-1)
                if not self.graph.has_edge(self.get_node_id_by_pos([self.px+0.438, self.py]), self.get_node_id_by_pos([self.px+0.438-2, self.py])):
                    new_node_pos = self.get_node_pos_by_id(
                        self.get_node_id_by_pos((self.px+0.438, self.py)))
                    self.graph.add_edge(self.get_node_id_by_pos(
                        [self.px+0.438, self.py]), self.get_node_id_by_pos([new_node_pos[0]-2, new_node_pos[1]]))

                self.current_node = self.get_node_id_by_pos((self.px, self.py))
                self.state = 1
                self.waiting_on_line = 3

            elif ((Decimal((self.px - 0.438 - 0.1) % 2).quantize(Decimal('0.1')) >= 1.9) and (Decimal((self.px - 0.438 + 0.1) % 2).quantize(Decimal('0.1')) <= 0.1) and current_orientation == 'S'):
                self.px = round(self.px) + 0.438
                self.py = round(self.py)
                self.already_reseted_movement_model = 10
                self.add_node((self.px-0.438, self.py))
                # self.add_visited_node(nx.number_of_nodes(self.graph)-1)
                if not self.graph.has_edge(self.get_node_id_by_pos([self.px-0.438, self.py]), self.get_node_id_by_pos([self.px-0.438+2, self.py])):
                    new_node_pos = self.get_node_pos_by_id(
                        self.get_node_id_by_pos((self.px-0.438, self.py)))
                    self.graph.add_edge(self.get_node_id_by_pos(
                        [self.px-0.438, self.py]), self.get_node_id_by_pos([new_node_pos[0]+2, new_node_pos[1]]))
                self.current_node = self.get_node_id_by_pos((self.px, self.py))
                self.state = 1
                self.waiting_on_line = 3

            if ((Decimal((self.py + 0.438 - 0.1) % 2).quantize(Decimal('0.1')) >= 1.9) and (Decimal((self.py + 0.438 + 0.1) % 2).quantize(Decimal('0.1')) <= 0.1) and current_orientation == 'W'):
                self.py = round(self.py) - 0.438
                self.px = round(self.px)
                self.already_reseted_movement_model = 10
                self.add_node((self.px, self.py+0.438))
                # self.add_visited_node(nx.number_of_nodes(self.graph)-1)
                if not self.graph.has_edge(self.get_node_id_by_pos([self.px, self.py+0.438]), self.get_node_id_by_pos([self.px, self.py+0.438-2])):
                    new_node_pos = self.get_node_pos_by_id(
                        self.get_node_id_by_pos((self.px, self.py+0.438)))
                    self.graph.add_edge(self.get_node_id_by_pos(
                        [self.px, self.py+0.438]), self.get_node_id_by_pos([new_node_pos[0], new_node_pos[1]-2]))
                self.current_node = self.get_node_id_by_pos((self.px, self.py))
                self.state = 1
                self.waiting_on_line = 3

            elif ((Decimal((self.py - 0.438 - 0.1) % 2).quantize(Decimal('0.1')) >= 1.9) and (Decimal((self.py - 0.438 + 0.1) % 2).quantize(Decimal('0.1')) <= 0.1) and current_orientation == 'E'):
                self.py = round(self.py) + 0.438
                self.px = round(self.px)
                self.already_reseted_movement_model = 10
                self.add_node((self.px, self.py-0.438))
                # self.add_visited_node(nx.number_of_nodes(self.graph)-1)
                if not self.graph.has_edge(self.get_node_id_by_pos([self.px, self.py-0.438]), self.get_node_id_by_pos([self.px, self.py-0.438+2])):
                    new_node_pos = self.get_node_pos_by_id(
                        self.get_node_id_by_pos((self.px, self.py-0.438)))
                    self.graph.add_edge(self.get_node_id_by_pos(
                        [self.px, self.py-0.438]), self.get_node_id_by_pos([new_node_pos[0], new_node_pos[1]+2]))

                self.current_node = self.get_node_id_by_pos((self.px, self.py))
                self.state = 1
                self.waiting_on_line = 3

            # print(f'Visited nodes: {self.visited_nodes}')
            # print(f'Not visited nodes: {self.not_visited_nodes}')

        else:
            self.already_reseted_movement_model -= 1

    def get_node_id_by_pos(self, pos):
        pos = (round(pos[0]/2)*2, round(pos[1]/2)*2)
        for id in list(nx.get_node_attributes(self.graph, 'pos').keys()):
            if pos == nx.get_node_attributes(self.graph, 'pos')[id]:
                return id

    def get_node_pos_by_id(self, id):
        if id != None:
            return nx.get_node_attributes(self.graph, 'pos')[id]
        return None

    def myMap2(self):
        for nodeA, nodeB in self.graph.edges():
            (nodeA_x, nodeA_y) = self.get_node_pos_by_id(nodeA)
            (nodeB_x, nodeB_y) = self.get_node_pos_by_id(nodeB)
            if nodeA_x == nodeB_x:
                if nodeA_y > nodeB_y:
                    self.labMap[int((nodeA_y+nodeB_y)/2)+10][int(nodeA_x)+24] = "|"
                else:
                    self.labMap[int((nodeA_y+nodeB_y)/2)+10][int(nodeA_x)+24] = "|"
            else:
                if nodeA_x > nodeB_x:
                    self.labMap[int(nodeA_y)+10][int((nodeA_x+nodeB_x)/2)+24] = "-"
                else:
                    self.labMap[int(nodeA_y)+10][int((nodeA_x+nodeB_x)/2)+24] = "-"
            self.put_beacons_on_map()
        self.write_map2()

    def write_map2(self):
        f = open("solution.map", "w")
        for l in reversed(self.labMap):
            s = ''.join([str(l) for l in l])
            f.write(s+'\n')
        f.close()

    def put_beacons_on_map(self):
        # for i in range(len(self.beacons_pos)):
        #     xval = self.beacons_pos[i][0]
        #     yval = self.beacons_pos[i][1]
        #     self.labMap[yval+10][xval+24] = i
        for b in self.beacons_pos:
            xval = self.beacons_pos[b][0]
            yval = self.beacons_pos[b][1]
            self.labMap[yval+10][xval+24] = b

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()

        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1)]
        i = 1
        for child in root.iter('Row'):
            line = child.attrib['Pattern']
            row = int(child.attrib['Pos'])
            if row % 2 == 0:  # this line defines vertical lines
                for c in range(len(line)):
                    if (c+1) % 3 == 0:
                        if line[c] == '|':
                            self.labMap[row][(c+1)//3*2-1] = '|'
                        else:
                            None
            else:  # this line defines horizontal lines
                for c in range(len(line)):
                    if c % 3 == 0:
                        if line[c] == '-':
                            self.labMap[row][c//3*2] = '-'
                        else:
                            None

            i = i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv), 2):
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
    rob = MyRob(rob_name, pos, [0.0, 60.0, -60.0, 180.0], host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()

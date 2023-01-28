import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

import networkx as nx

from queue import *

from decimal import Decimal

import matplotlib.pyplot as plt
import os

from collections import *

CELLROWS = 7
CELLCOLS = 14


class MyRob(CRobLinkAngs):
    # movement variables
    initial_compass = 0
    # tuple => (rotating to[left, right], initial compass value when starts rotating)
    rotating_to = None
    already_rotating = False
    already_reseted_movement_model = 0

    # removing noise variables
    normalized_measures = [
        ['0', '0', '1', '1', '1', '0', '0'],
        ['0', '0', '1', '1', '1', '0', '0'],
        ['0', '0', '1', '1', '1', '0', '0']
    ]

    # beacons position
    beacons_pos = []
    num_beacons = 0

    # graph variables
    graph = nx.MultiGraph()
    not_visited_nodes = []
    visited_nodes = []
    not_visited_nodes = deque()

    next_node = None
    current_node = None
    target_node = None
    path = []

    target_angle = 0

    state_movement = 'moving'

    rotate_half = None
    priority = None

    back_to_keep_line = False

    last_node = None

    already_centered = 0

    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
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

        self.readSensors()

        # get number of beacons
        self.initial_compass = self.measures.compass

        self.add_node((self.px, self.py))
        self.visited_nodes.append(nx.number_of_nodes(self.graph)-1)
        self.beacons_pos.append({"id": self.measures.ground, "pos": (self.px, self.py)})
        self.current_node = nx.number_of_nodes(self.graph)-1
        self.check_front()
        self.next_node = self.not_visited_nodes.pop()
        self.last_node = self.current_node

        while True:
            self.readSensors()
            self.num_beacons = self.nBeacons

            self.MyMap()

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
                self.movement()
            elif state == 'wait':
                self.setReturningLed(True)
                if self.measures.visitingLed == True:
                    self.setVisitingLed(False)
                if self.measures.returningLed == True:
                    state = 'return'
                self.calcPos(0.0, 0.0)
            elif state == 'return':
                if self.measures.visitingLed == True:
                    self.setVisitingLed(False)
                if self.measures.returningLed == True:
                    self.setReturningLed(False)
                self.move()

                # self.movement()
            # self.movement()

            if self.rotating_to != None:
                self.detect_intersection_in_front()

    '''
        STATE MACHINE
            - Possible States:
                1) centetring   -> centering the robot on the intersection
                2) rotating     -> aligning the robot in the next node direction
                3) moving       -> moving the robot forward until reach the new cell

            1) Centering:
                a) while not centered go in front and detect intersection on the sides
                    - add new not visited nodes
                b) when the robot is centered check the intersection in front
                    - add current node to the visited_nodes list
                    - add new not visited node if there are intersection in front
                    - update the target_node
                    - update state to rotating
            2) Rotating:
                a) when the robot is aligned
                    - update state to moving
            3) Moving:
                a) when the robot is in the new cell
                    - update state to centering
    '''

    def move(self):
        print("-------------------")
        print(f"[CURRENT COORDINATES]: {(self.px, self.py)}")
        print("--- printing graph ---")
        print(self.graph.nodes())
        print(self.graph.edges())
        print(nx.get_node_attributes(self.graph, 'pos'))
        print("--- end of graph ---")
        print(f"[STATE]: {self.state_movement}\n")
        print(f"[CURRENT NODE]: {self.current_node} pos: {self.get_node_pos_by_id(self.current_node)}\n")
        print(f"[NEXT NODE]: {self.next_node} pos: {self.get_node_pos_by_id(self.next_node)}\n")
        print(f"[PATH]: {self.path}\n")
        print("-------------------")

        # if self.state_movement == 'moving':
        #     self.move_forward()
        # elif self.state_movement == 'centering':
        #     self.detect_intersection_sides()
        #     self.centering()
        # elif self.state_movement == 'rotating':
        #     self.rotate()

        print(f"CENTERING {self.already_centered}\n")
        if self.already_centered == 1:
            print("CENTERING")
            self.centering()
        else:

            if self.not_visited_nodes != []:
                # erro aqui !!! last nao é diferente do current porque ja vai no terceiro estado em que o current e o next sao iguais
                if self.last_node != self.current_node:
                    if self.current_node == self.next_node and self.already_centered == 0:
                        self.already_centered = 1
                        print("NEW LAST NODE BOOOOYOYYY")
                        print("aqui0")
                        print("aqui0")
                        print("aqui0")
                        print("aqui0")
                        print("aqui0")
                        print("aqui0")
                        print("aqui0")
                        print("aqui0")
                        print("aqui0")
                    
                    elif self.current_node == self.next_node and self.already_centered == 2:
                        self.check_front()
                        print("aqui2")
                        print("aqui2")
                        print("aqui2")
                        print("aqui2")
                        print("aqui2")
                        print("aqui2")
                        print("aqui2")
                        print("aqui2")
                        print("aqui2")
                        print(f"NEW NEXT NODE => {self.not_visited_nodes[-1]}\n")
                        self.next_node = self.not_visited_nodes.pop()

                    current_orientation = self.get_orientation(self.measures.compass)

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

                self.last_node = self.current_node

                print("-------------")
                print("-------------")
                print("-------------")
                print(f"Priority: {self.priority}\n")
                print("-------------")
                print("-------------")
                print("-------------")

                self.go_to_next_node()



    def go_to_next_node(self):
        if self.priority == 'left':
            if self.back_to_keep_line:
                self.calcPos(0.15, -0.15)
                self.back_to_keep_line = False
            # rotate left
            elif self.measures.lineSensor[0] == '1':
                self.calcPos(-0.01, 0.05)
            # rotate left softly
            elif self.measures.lineSensor[1] == '1' and self.measures.lineSensor[5] == '0':
                self.detect_intersection_sides()
                self.calcPos(0.03, 0.05)
            # rotate right softly
            elif self.measures.lineSensor[1] == '0' and self.measures.lineSensor[5] == '1':
                self.detect_intersection_sides()
                self.calcPos(0.05, 0.03)
            # go back to keep in line
            elif self.measures.lineSensor == ['0', '0', '0', '0', '0', '0', '0'] and not self.back_to_keep_line:
                self.calcPos(-0.14, -0.14)
                self.back_to_keep_line = True
            # go in front
            else:
                self.detect_intersection_sides()
                self.calcPos(0.05, 0.05)
        elif self.priority == 'right':
            if self.back_to_keep_line:
                self.calcPos(-0.15, 0.15)
                self.back_to_keep_line = False
            # rotate right
            elif self.measures.lineSensor[6] == '1':
                self.calcPos(0.05, -0.01)
            # rotate left softly
            elif self.measures.lineSensor[1] == '1' and self.measures.lineSensor[5] == '0':
                self.detect_intersection_sides()
                self.calcPos(0.03, 0.05)
            # rotate right softly
            elif self.measures.lineSensor[1] == '0' and self.measures.lineSensor[5] == '1':
                self.detect_intersection_sides()
                self.calcPos(0.05, 0.03)
            # go back to keep in line
            elif self.measures.lineSensor == ['0', '0', '0', '0', '0', '0', '0'] and not self.back_to_keep_line:
                self.calcPos(-0.14, -0.14)
                self.back_to_keep_line = True
            # go in front
            else:
                self.calcPos(0.05, 0.05)
                self.detect_intersection_sides()
        elif self.priority == 'front':
            if self.back_to_keep_line:
                self.calcPos(-0.15, -0.15)
                self.back_to_keep_line = False
            # rotate left softly
            elif self.measures.lineSensor[1] == '1' and self.measures.lineSensor[5] == '0':
                self.detect_intersection_sides()
                self.calcPos(0.03, 0.05)
            # rotate right softly
            elif self.measures.lineSensor[1] == '0' and self.measures.lineSensor[5] == '1':
                self.detect_intersection_sides()
                self.calcPos(0.05, 0.03)
            # go back to keep in line
            elif self.measures.lineSensor == ['0', '0', '0', '0', '0', '0', '0'] and not self.back_to_keep_line:
                self.calcPos(-0.14, -0.14)
                self.back_to_keep_line = True
            # go in front
            else:
                self.detect_intersection_sides()
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
                    self.detect_intersection_sides()
                    self.calcPos(0.03, 0.05)
                # rotate right softly
                elif self.measures.lineSensor[1] == '0' and self.measures.lineSensor[5] == '1':
                    self.detect_intersection_sides()
                    self.calcPos(0.05, 0.03)
                # go back to keep in line
                elif self.measures.lineSensor == ['0', '0', '0', '0', '0', '0', '0'] and not self.back_to_keep_line:
                    self.calcPos(-0.14, -0.14)
                    self.back_to_keep_line = True
                # go in front
                else:
                    self.detect_intersection_sides()
                    self.calcPos(0.05, 0.05)
        else:
            self.calcPos(0.05, 0.05)

    # def go_to(self):
    #     print("-------------------")
    #     print("--- printing graph ---")
    #     print(self.graph.nodes())
    #     print(self.graph.edges())
    #     print(nx.get_node_attributes(self.graph, 'pos'))
    #     print("--- end of graph ---")
    #     print(f"[STATE]: {self.state_movement}\n")
    #     print(
    #         f"[CURRENT NODE]: {self.current_node} pos: {self.get_node_pos_by_id(self.current_node)}\n")
    #     print(
    #         f"[NEXT NODE]: {self.next_node} pos: {self.get_node_pos_by_id(self.next_node)}\n")
    #     print(
    #         f"[TARGET NODE]: {self.target_node} pos: {self.get_node_pos_by_id(self.target_node)}\n")
    #     print(f"[PATH]: {self.path}\n")
    #     print("-------------------")

    #     if self.target_node == None:
    #         self.detect_intersection_sides()
    #         if len(self.not_visited_nodes) > 0:
    #             self.target_node = self.not_visited_nodes.pop()
    #             print("AQUI ====================")
    #             print("AQUI ====================")
    #             print(nx.shortest_path(self.graph,
    #                   self.current_node, self.target_node))
    #             self.path = nx.shortest_path(
    #                 self.graph, self.current_node, self.target_node)
    #             self.path.pop(0)
    #             self.next_node = self.path.pop(0)
    #             print("AQUI ====================")
    #         else:
    #             self.calcPos(0.05, 0.05)
    #     else:
    #         if self.current_node == self.target_node:
    #             print("AQUI ====================")
    #             print("AQUI ====================")
    #             self.detect_intersection_sides()
    #             self.target_node = self.not_visited_nodes.pop()
    #             print(nx.shortest_path(self.graph,
    #                   self.current_node, self.target_node))
    #             self.path = nx.shortest_path(
    #                 self.graph, self.current_node, self.target_node)
    #             self.path.pop(0)
    #             self.next_node = self.path.pop(0)
    #             print("AQUI ====================")

    #         if self.state_movement == 'centering':
    #             self.detect_intersection_sides()
    #             self.centering()
    #         elif self.state_movement == 'rotating':
    #             self.rotate()
    #         elif self.state_movement == 'moving':
    #             # self.detect_intersection_sides()
    #             self.move_forward()

    def move_forward(self):
        self.current_node = self.get_node_id_by_pos((self.px, self.py))
        if self.current_node != self.next_node:
            self.calcPos(0.05, 0.05)
        else:
            self.current_node = self.next_node
            if len(self.path) > 0:
                if len(self.not_visited_nodes) > 0:
                    self.next_node = self.path.pop(0)

            self.state_movement = 'centering'
            self.calcPos(0, 0)



    def check_front(self):
        if self.measures.lineSensor[2] == '1' and self.measures.lineSensor[3] == '1' and self.measures.lineSensor[4] == '1':
            current_orientation = self.get_orientation(self.measures.compass)
            rounded_coords = [round(self.px/2)*2, round(self.py/2)*2]
            if current_orientation == 'N':
                self.add_node((self.px+2, self.py))
                self.add_not_visited_node(nx.number_of_nodes(self.graph)-1)
                if not self.graph.has_edge(self.get_node_id_by_pos(rounded_coords), self.get_node_id_by_pos([rounded_coords[0]+2, rounded_coords[1]])) and self.get_node_id_by_pos(rounded_coords) != None:
                    self.graph.add_edge(self.get_node_id_by_pos(
                        rounded_coords), self.get_node_id_by_pos([rounded_coords[0]+2, rounded_coords[1]]))
            elif current_orientation == 'S':
                self.add_node((self.px-2, self.py))
                self.add_not_visited_node(nx.number_of_nodes(self.graph)-1)
                if not self.graph.has_edge(self.get_node_id_by_pos(rounded_coords), self.get_node_id_by_pos([rounded_coords[0]-2, rounded_coords[1]])) and self.get_node_id_by_pos(rounded_coords) != None:
                    self.graph.add_edge(self.get_node_id_by_pos(
                        rounded_coords), self.get_node_id_by_pos([rounded_coords[0]-2, rounded_coords[1]]))
            elif current_orientation == 'E':
                self.add_node((self.px, self.py-2))
                self.add_not_visited_node(nx.number_of_nodes(self.graph)-1)
                if not self.graph.has_edge(self.get_node_id_by_pos(rounded_coords), self.get_node_id_by_pos([rounded_coords[0], rounded_coords[1]-2])) and self.get_node_id_by_pos(rounded_coords) != None:
                    self.graph.add_edge(self.get_node_id_by_pos(
                        rounded_coords), self.get_node_id_by_pos([rounded_coords[0], rounded_coords[1]-2]))
            elif current_orientation == 'W':
                self.add_node((self.px, self.py+2))
                self.add_not_visited_node(nx.number_of_nodes(self.graph)-1)
                if not self.graph.has_edge(self.get_node_id_by_pos(rounded_coords), self.get_node_id_by_pos([rounded_coords[0], rounded_coords[1]+2])) and self.get_node_id_by_pos(rounded_coords) != None:
                    self.graph.add_edge(self.get_node_id_by_pos(
                        rounded_coords), self.get_node_id_by_pos([rounded_coords[0], rounded_coords[1]+2]))
            print("check front -> new node added")
            print("new node id: ", nx.number_of_nodes(self.graph)-1)
            print("new node pos: ", self.get_node_pos_by_id(nx.number_of_nodes(self.graph)-1))

    def centering(self):
        current_orientation = self.get_orientation(self.measures.compass)
        print(f"Coordinates centering x: {round(self.px,1)} y:{round(self.py,1)} orientation: {current_orientation}\n")
        
        if ((current_orientation == 'N' or current_orientation == 'S') and round(self.px, 1) % 2 == 0) or ((current_orientation == 'E' or current_orientation == 'W') and round(self.py, 1) % 2 == 0):
            # self.check_front()
            # self.current_node = self.get_node_id_by_pos((round(self.px, 1), round(self.py, 1)))
            # self.add_visited_node(self.current_node)
            # self.next_node = self.visited_nodes.pop()
            # self.set_target_angle()
            # self.state_movement = 'rotating'
            self.calcPos(0, 0)
            self.already_centered = 2
        else:
            self.calcPos(0.05, 0.05)

    def set_target_angle(self):
        current_orientation = self.get_orientation(self.measures.compass)
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
                self.calcPos(-0.01, 0.01)
            else:
                self.calcPos(0.01, -0.01)
        else:
            self.state_movement = 'moving'
            self.calcPos(0, 0)

    def radians_to_degrees(self, radians):
        return (radians * 180) / pi

    def add_node(self, pos):
        pos = (round(pos[0]/2)*2, round(pos[1]/2)*2)
        if not self.node_exists(pos):
            print("adding node: " + str(pos))
            self.graph.add_node(nx.number_of_nodes(self.graph), pos=pos)
            print("--- printig graph ---")
            print(self.graph.nodes())
            print(self.graph.edges())
            print(nx.get_node_attributes(self.graph, 'pos'))
            print("--- end of graph ---")
            return True
        return False

    def node_exists(self, pos):
        return pos in list(nx.get_node_attributes(self.graph, 'pos').values())

    # makes the robot follow the line
    def movement(self):
        # checks if there is an intersection with both left and right possible paths (T intersection)
        if not self.already_rotating:
            if self.measures.lineSensor[0] == '1' and self.measures.lineSensor[6] == '1' and self.measures.lineSensor[1] == '1' and self.measures.lineSensor[5] == '1':
                # print("both sides")
                pass

        # rotate left
        if self.measures.lineSensor[0] == '1':
            self.calcPos(-0.01, 0.05)
            if not self.already_rotating:
                self.detect_intersection_sides()
                self.rotating_to = ('left', self.relative_compass())
            self.already_rotating = True
        # rotate right
        elif self.measures.lineSensor[6] == '1':
            self.calcPos(0.05, -0.01)
            if not self.already_rotating:
                self.detect_intersection_sides()
                self.rotating_to = ('right', self.relative_compass())
            self.already_rotating = True
        # rotate left softly
        elif self.measures.lineSensor[1] == '1' and self.measures.lineSensor[5] == '0':
            self.calcPos(0.03, 0.05)
        # rotate right softly
        elif self.measures.lineSensor[1] == '0' and self.measures.lineSensor[5] == '1':
            self.calcPos(0.05, 0.03)
        # go back to keep in line
        elif self.measures.lineSensor == ['0', '0', '0', '0', '0', '0', '0']:
            self.calcPos(-0.14, -0.14)
            self.rotating_to = None
            self.already_rotating = False
        # go in front
        else:
            self.calcPos(0.05, 0.05)
            self.rotating_to = None
            self.already_rotating = False

    # detects intersection on sides while not rotating
    def detect_intersection_sides(self):
        if self.rotating_to == None:
            rounded_coords = [round(self.px/2)*2, round(self.py/2)*2]
            if self.measures.lineSensor[0] == '1':
                print("left intersection")
                # add node and edge to graph
                if self.get_orientation(self.measures.compass) == 'N':
                    self.add_node((self.px, self.py+2))
                    self.add_not_visited_node(nx.number_of_nodes(self.graph)-1)
                    if not self.graph.has_edge(self.get_node_id_by_pos(rounded_coords), self.get_node_id_by_pos([rounded_coords[0], rounded_coords[1]+2])) and self.get_node_id_by_pos(rounded_coords) != None:
                        self.graph.add_edge(self.get_node_id_by_pos(
                            rounded_coords), self.get_node_id_by_pos([rounded_coords[0], rounded_coords[1]+2]))
                    self.next_node = self.get_node_id_by_pos([rounded_coords[0], rounded_coords[1]+2])
                    self.path = nx.shortest_path(self.graph, self.get_node_id_by_pos(rounded_coords), self.next_node)
                    self.next_node = self.not_visited_nodes.pop()
                elif self.get_orientation(self.measures.compass) == 'S':
                    self.add_node((self.px, self.py-2))
                    self.add_not_visited_node(nx.number_of_nodes(self.graph)-1)
                    if not self.graph.has_edge(self.get_node_id_by_pos(rounded_coords), self.get_node_id_by_pos([rounded_coords[0], rounded_coords[1]-2])) and self.get_node_id_by_pos(rounded_coords) != None:
                        self.graph.add_edge(self.get_node_id_by_pos(
                            rounded_coords), self.get_node_id_by_pos([rounded_coords[0], rounded_coords[1]-2]))
                    self.next_node = self.get_node_id_by_pos([rounded_coords[0], rounded_coords[1]-2])
                    self.path = nx.shortest_path(self.graph, self.get_node_id_by_pos(rounded_coords), self.next_node)
                    self.next_node = self.not_visited_nodes.pop()
                elif self.get_orientation(self.measures.compass) == 'E':
                    self.add_node((self.px+2, self.py))
                    self.add_not_visited_node(nx.number_of_nodes(self.graph)-1)
                    if not self.graph.has_edge(self.get_node_id_by_pos(rounded_coords), self.get_node_id_by_pos([rounded_coords[0]+2, rounded_coords[1]])) and self.get_node_id_by_pos(rounded_coords) != None:
                        self.graph.add_edge(self.get_node_id_by_pos(
                            rounded_coords), self.get_node_id_by_pos([rounded_coords[0]+2, rounded_coords[1]]))
                    self.next_node = self.get_node_id_by_pos([rounded_coords[0]+2, rounded_coords[1]])
                    self.path = nx.shortest_path(self.graph, self.get_node_id_by_pos(rounded_coords), self.next_node)
                    self.next_node = self.not_visited_nodes.pop()
                elif self.get_orientation(self.measures.compass) == 'W':
                    self.add_node((self.px-2, self.py))
                    self.add_not_visited_node(nx.number_of_nodes(self.graph)-1)
                    if not self.graph.has_edge(self.get_node_id_by_pos(rounded_coords), self.get_node_id_by_pos([rounded_coords[0]-2, rounded_coords[1]])) and self.get_node_id_by_pos(rounded_coords) != None:
                        self.graph.add_edge(self.get_node_id_by_pos(
                            rounded_coords), self.get_node_id_by_pos([rounded_coords[0]-2, rounded_coords[1]]))
                    self.next_node = self.get_node_id_by_pos([rounded_coords[0]-2, rounded_coords[1]])
                    self.path = nx.shortest_path(self.graph, self.get_node_id_by_pos(rounded_coords), self.next_node)
                    self.next_node = self.not_visited_nodes.pop()

            if self.measures.lineSensor[6] == '1':
                print("right intersection")
                # add node and edge to graph
                if self.get_orientation(self.measures.compass) == 'N':
                    self.add_node((self.px, self.py-2))
                    self.add_not_visited_node(nx.number_of_nodes(self.graph)-1)
                    if not self.graph.has_edge(self.get_node_id_by_pos(rounded_coords), self.get_node_id_by_pos([rounded_coords[0], rounded_coords[1]-2])) and self.get_node_id_by_pos(rounded_coords) != None:
                        self.graph.add_edge(self.get_node_id_by_pos(
                            rounded_coords), self.get_node_id_by_pos([rounded_coords[0], rounded_coords[1]-2]))
                    self.next_node = self.get_node_id_by_pos([rounded_coords[0], rounded_coords[1]-2])
                    self.path = nx.shortest_path(self.graph, self.get_node_id_by_pos(rounded_coords), self.next_node)
                    self.next_node = self.not_visited_nodes.pop()
                elif self.get_orientation(self.measures.compass) == 'S':
                    self.add_node((self.px, self.py+2))
                    self.add_not_visited_node(nx.number_of_nodes(self.graph)-1)
                    if not self.graph.has_edge(self.get_node_id_by_pos(rounded_coords), self.get_node_id_by_pos([rounded_coords[0], rounded_coords[1]+2])) and self.get_node_id_by_pos(rounded_coords) != None:
                        self.graph.add_edge(self.get_node_id_by_pos(
                            rounded_coords), self.get_node_id_by_pos([rounded_coords[0], rounded_coords[1]+2]))
                    self.next_node = self.get_node_id_by_pos([rounded_coords[0], rounded_coords[1]+2])
                    self.path = nx.shortest_path(self.graph, self.get_node_id_by_pos(rounded_coords), self.next_node)
                    self.next_node = self.not_visited_nodes.pop()
                elif self.get_orientation(self.measures.compass) == 'E':
                    self.add_node((self.px-2, self.py))
                    self.add_not_visited_node(nx.number_of_nodes(self.graph)-1)
                    if not self.graph.has_edge(self.get_node_id_by_pos(rounded_coords), self.get_node_id_by_pos([rounded_coords[0]-2, rounded_coords[1]])) and self.get_node_id_by_pos(rounded_coords) != None:
                        self.graph.add_edge(self.get_node_id_by_pos(
                            rounded_coords), self.get_node_id_by_pos([rounded_coords[0]-2, rounded_coords[1]]))
                    self.next_node = self.get_node_id_by_pos([rounded_coords[0]-2, rounded_coords[1]])
                    self.path = nx.shortest_path(self.graph, self.get_node_id_by_pos(rounded_coords), self.next_node)
                    self.next_node = self.not_visited_nodes.pop()
                elif self.get_orientation(self.measures.compass) == 'W':
                    self.add_node((self.px+2, self.py))
                    self.add_not_visited_node(nx.number_of_nodes(self.graph)-1)
                    if not self.graph.has_edge(self.get_node_id_by_pos(rounded_coords), self.get_node_id_by_pos([rounded_coords[0]+2, rounded_coords[1]])) and self.get_node_id_by_pos(rounded_coords) != None:
                        self.graph.add_edge(self.get_node_id_by_pos(
                            rounded_coords), self.get_node_id_by_pos([rounded_coords[0]+2, rounded_coords[1]]))
                    self.next_node = self.get_node_id_by_pos([rounded_coords[0]+2, rounded_coords[1]])
                    self.path = nx.shortest_path(self.graph, self.get_node_id_by_pos(rounded_coords), self.next_node)
                    self.next_node = self.not_visited_nodes.pop()


    # add to not visited nodes if not in visited nodes

    def add_not_visited_node(self, id):
        if id not in self.not_visited_nodes:
            self.not_visited_nodes.append(id)

    # add node to visited nodes and remove from not visited nodes
    def add_visited_node(self, id):
        if id not in self.visited_nodes:
            self.visited_nodes.append(id)
            if id in self.not_visited_nodes:
                self.not_visited_nodes.remove(id)

    # detects the intersection in front when the robot is rotating (L intersection)
    def detect_intersection_in_front(self):
        if self.rotating_to[0] == 'right' and self.measures.lineSensor[0] == '1':
            print("intersection in front")
        elif self.rotating_to[0] == 'left' and self.measures.lineSensor[6] == '1':
            print("intersection in front")

    def relative_compass(self):
        return self.measures.compass - self.initial_compass

    def relative_orientation(self):
        quarter = round(self.relative_compass() / 90)
        if quarter == 0:
            return 'N'
        elif quarter == 1:
            return 'W'
        elif quarter == -1:
            return 'E'
        else:
            return 'S'

    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        if self.measures.irSensor[center_id] > 5.0\
           or self.measures.irSensor[left_id] > 5.0\
           or self.measures.irSensor[right_id] > 5.0\
           or self.measures.irSensor[back_id] > 5.0:
            print('Rotate left')
            self.driveMotors(-0.1, +0.1)
        elif self.measures.irSensor[left_id] > 2.7:
            print('Rotate slowly right')
            self.driveMotors(0.1, 0.0)
        elif self.measures.irSensor[right_id] > 2.7:
            print('Rotate slowly left')
            self.driveMotors(0.0, 0.1)
        else:
            print('Go')
            self.driveMotors(0.05, 0.05)

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

        self.pa = self.pa + rot

        current_orientation = self.get_orientation(self.measures.compass)

        if self.already_reseted_movement_model == 0:
            if ((Decimal((self.px + 0.438 - 0.1) % 2).quantize(Decimal('0.1')) >= 1.9) and (Decimal((self.px + 0.438 + 0.1) % 2).quantize(Decimal('0.1')) <= 0.1) and current_orientation == 'N'):
                self.px = round(self.px) - 0.438
                self.py = round(self.py)
                self.already_reseted_movement_model = 3
                self.add_node((self.px+0.438, self.py))
                # self.add_visited_node(nx.number_of_nodes(self.graph)-1)
                if not self.graph.has_edge(self.get_node_id_by_pos([self.px+0.438, self.py]), self.get_node_id_by_pos([self.px+0.438-2, self.py])):
                    print(f"coords => ({self.px+0.438}, {self.py})\n")
                    print("First node: ", self.get_node_id_by_pos((self.px+0.438, self.py)))
                    print("Second node: ", self.get_node_id_by_pos((self.px+0.438-2, self.py)))
                    new_node_pos = self.get_node_pos_by_id(self.get_node_id_by_pos((self.px+0.438, self.py)))
                    self.graph.add_edge(self.get_node_id_by_pos([self.px+0.438, self.py]), self.get_node_id_by_pos([new_node_pos[0]-2, new_node_pos[1]]))
                if self.measures.ground != -1 and self.measures.ground not in [x["id"] for x in self.beacons_pos]:
                    self.beacons_pos.append(
                        {"id": self.measures.ground, "pos": (self.px+0.438, self.py)})
                self.current_node = self.get_node_id_by_pos((self.px , self.py))
            elif ((Decimal((self.px - 0.438 - 0.1) % 2).quantize(Decimal('0.1')) >= 1.9) and (Decimal((self.px - 0.438 + 0.1) % 2).quantize(Decimal('0.1')) <= 0.1) and current_orientation == 'S'):
                self.px = round(self.px) + 0.438
                self.py = round(self.py)
                self.already_reseted_movement_model = 3
                self.add_node((self.px-0.438, self.py))
                # self.add_visited_node(nx.number_of_nodes(self.graph)-1)
                if not self.graph.has_edge(self.get_node_id_by_pos([self.px-0.438, self.py]), self.get_node_id_by_pos([self.px-0.438+2, self.py])):
                    print(f"coords => ({self.px-0.438}, {self.py})\n")
                    print("First node: ", self.get_node_id_by_pos((self.px-0.438, self.py)))
                    print("Second node: ", self.get_node_id_by_pos((self.px-0.438+2, self.py)))
                    new_node_pos = self.get_node_pos_by_id(self.get_node_id_by_pos((self.px-0.438, self.py)))
                    self.graph.add_edge(self.get_node_id_by_pos([self.px-0.438, self.py]), self.get_node_id_by_pos([new_node_pos[0]+2, new_node_pos[1]]))
                if self.measures.ground != -1 and self.measures.ground not in [x["id"] for x in self.beacons_pos]:
                    self.beacons_pos.append(
                        {"id": self.measures.ground, "pos": (self.px-0.438, self.py)})
                self.current_node = self.get_node_id_by_pos((self.px , self.py))

            if ((Decimal((self.py + 0.438 - 0.1) % 2).quantize(Decimal('0.1')) >= 1.9) and (Decimal((self.py + 0.438 + 0.1) % 2).quantize(Decimal('0.1')) <= 0.1) and current_orientation == 'W'):
                self.py = round(self.py) - 0.438
                self.px = round(self.px)
                self.already_reseted_movement_model = 3
                self.add_node((self.px, self.py+0.438))
                # self.add_visited_node(nx.number_of_nodes(self.graph)-1)
                if not self.graph.has_edge(self.get_node_id_by_pos([self.px, self.py+0.438]), self.get_node_id_by_pos([self.px, self.py+0.438-2])):
                    print(f"coords => ({self.px}, {self.py+0.438})\n")
                    print("First node: ", self.get_node_id_by_pos((self.px, self.py+0.438)))
                    print("Second node: ", self.get_node_id_by_pos((self.px, self.py+0.438-2)))
                    new_node_pos = self.get_node_pos_by_id(self.get_node_id_by_pos((self.px, self.py+0.438)))
                    self.graph.add_edge(self.get_node_id_by_pos([self.px, self.py+0.438]), self.get_node_id_by_pos([new_node_pos[0], new_node_pos[1]-2]))
                if self.measures.ground != -1 and self.measures.ground not in [x["id"] for x in self.beacons_pos]:
                    self.beacons_pos.append(
                        {"id": self.measures.ground, "pos": (self.px, self.py+0.438)})
                self.current_node = self.get_node_id_by_pos((self.px , self.py))

            elif ((Decimal((self.py - 0.438 - 0.1) % 2).quantize(Decimal('0.1')) >= 1.9) and (Decimal((self.py - 0.438 + 0.1) % 2).quantize(Decimal('0.1')) <= 0.1) and current_orientation == 'E'):
                self.py = round(self.py) + 0.438
                self.px = round(self.px)
                self.already_reseted_movement_model = 3
                self.add_node((self.px, self.py-0.438))
                # self.add_visited_node(nx.number_of_nodes(self.graph)-1)
                if not self.graph.has_edge(self.get_node_id_by_pos([self.px, self.py-0.438]), self.get_node_id_by_pos([self.px, self.py-0.438+2])):
                    print(f"coords => ({self.px}, {self.py-0.438})\n")
                    print("First node: ", self.get_node_id_by_pos((self.px, self.py-0.438)))
                    print("Second node: ", self.get_node_id_by_pos((self.px, self.py-0.438+2)))
                    new_node_pos = self.get_node_pos_by_id(self.get_node_id_by_pos((self.px, self.py-0.438)))
                    self.graph.add_edge(self.get_node_id_by_pos([self.px, self.py-0.438]), self.get_node_id_by_pos([new_node_pos[0], new_node_pos[1]+2]))
                if self.measures.ground != -1 and self.measures.ground not in [x["id"] for x in self.beacons_pos]:
                    self.beacons_pos.append(
                        {"id": self.measures.ground, "pos": (self.px, self.py-0.438)})
                self.current_node = self.get_node_id_by_pos((self.px , self.py))

            print(f'Visited nodes: {self.visited_nodes}')
            print(f'Not visited nodes: {self.not_visited_nodes}')

        else:
            self.already_reseted_movement_model -= 1

        # print('x:'+str(self.px)+' y:'+str(self.py)+' a:'+str(self.pa)+' cos:'+str(cos((self.pa))))

        # dar instervalos  de erro com  apos calculo[+/-0.01]-0.438
    def get_node_id_by_pos(self, pos):
        pos = (round(pos[0]/2)*2, round(pos[1]/2)*2)
        for id in list(nx.get_node_attributes(self.graph, 'pos').keys()):
            if pos == nx.get_node_attributes(self.graph, 'pos')[id]:
                return id

    def get_node_pos_by_id(self, id):
        if id != None:
            return nx.get_node_attributes(self.graph, 'pos')[id]
        return None

    # funtion to get the gps coordinates and convert to matrix coordinates
    def MyMap(self):
        xval = int(round(self.px))
        yval = int(round(self.py))
        self.labMap[10][24] = "I"
        if (xval, yval) in [x['pos'] for x in self.beacons_pos]:
            carater = [x['id']
                       for x in self.beacons_pos if x['pos'] == (xval, yval)][0]
        else:
            if(yval+10) % 2 == 0:
                if(xval+24-1) % 2 == 0:
                    carater = "-"
                else:
                    carater = " "
            else:
                carater = "|"
        self.labMap[yval+10][xval+24] = carater
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
    def get_node_id_by_pos(self, pos):
        pos = (round(pos[0]/2)*2, round(pos[1]/2)*2)
        for id in list(nx.get_node_attributes(self.graph, 'pos').keys()):
            if pos == nx.get_node_attributes(self.graph, 'pos')[id]:
                return id

    def get_node_pos_by_id(self, id):
        if id != None:
            return nx.get_node_attributes(self.graph, 'pos')[id]
        return None

    # funtion to get the gps coordinates and convert to matrix coordinates
    def MyMap(self):
        xval = int(round(self.px))
        yval = int(round(self.py))
        self.labMap[10][24] = "I"
        if (xval, yval) in [x['pos'] for x in self.beacons_pos]:
            carater = [x['id']
                       for x in self.beacons_pos if x['pos'] == (xval, yval)][0]
        else:
            if(yval+10) % 2 == 0:
                if(xval+24-1) % 2 == 0:
                    carater = "-"
                else:
                    carater = " "
            else:
                carater = "|"
        self.labMap[yval+10][xval+24] = carater
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

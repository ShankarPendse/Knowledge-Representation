# -*- coding: utf-8 -*-
"""
Created on Tue Oct 20 18:57:08 2020

@author: shank
"""

import os, sys, inspect

current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

parent_dir = os.path.dirname(current_dir)

sys.path.insert(0, parent_dir)

from agents import *
from search import *
from logic import *
from utils import *
import numpy as np
import time

class TrafficSignal(Thing):
    pass

class TurnRightSignBoard(Thing):
    pass

class TurnLeftSignBoard(Thing):
    pass

class City(GraphicEnvironment):
    def percept(self, agent):
        '''return a list of things that are in our agent's location'''
        things = self.list_things_at(agent.location)
        return things
    
    def execute_action(self, agent, action):
        '''changes the state of the environment based on what the agent does.'''
        # agent.update_state(action)
        
        if action == 'goal':
            print("    Goal reached, and current location {} is equal to goal location {} ".format(agent.location, agent.goal_location))
            return 
        
        elif action == 'turnright':
            print('    {} decided to {} at location: {}'.format(str(agent)[1:-1], action, agent.location))
            if agent.turnright():
                print('    {} turned right and current location is: {}'.format(str(agent)[1:-1], agent.location))
            else:
                print("    {} cannot turn right".format(agent))
            
        elif action == 'turnleft':
            print('    {} decided to {} at location: {}'.format(str(agent)[1:-1], action, agent.location))
            if agent.turnleft():
                print('    {} turned left and current location is: {}'.format(str(agent)[1:-1], agent.location))
            else:
                print("    {} cannnot turn left".format(agent))                           
        
        
        elif action == "moveahead":
            if agent.direction == "D":
                print('    {} decided to move down at location {}'.format(str(agent)[1:-1], agent.location))
            else:
                print('    {} decided to {} at location: {}'.format(str(agent)[1:-1], action, agent.location))
            if agent.moveahead():
                if agent.direction == "D":
                    print('    {} moved down and current location is: {}'.format(str(agent)[1:-1], agent.location))
                else:    
                    print('    {} decided to {} and current location is: {}'.format(str(agent)[1:-1], action, agent.location))
            else:
                print("    {} cannot move further".format(agent))
        
        elif action == "wait":
            print('    {} decided to {} for {} seconds due to traffic signal'.format(str(agent)[1:-1], action, agent.wait_time))
            if agent.wait():
                print("    {} did wait for {} seconds due to traffic signal".format(str(agent)[1:-1], agent.wait_time))
    
    #This will be used for search implementation
    def getCityMapAndLocations(self):
        # city_map = UndirectedGraph(dict(
        #                                 college = dict(Block1 = 3, Block2 = 2, Block3 = 3.5),
        #                                 home = dict(Block10 = 2, Block11 = 1),
        #                                 forest = dict(Block4 = 6, home = 2),
        #                                 theatre = dict(home = 3, Block7 = 4, Block8 = 5, Block9 = 6),
        #                                 Block4 = dict(Block1 = 1.5),
        #                                 Block5 = dict(Block1 = 1, Block2 = 2),
        #                                 Block6 = dict(Block2 = 1.5, Block10 = 2.5, Block11 = 2),
        #                                 Block3 = dict(Block7 = 1, Block8 = 1, Block9 = 4)
        #                                 )
        #                             )
        
        city_map = UndirectedGraph(dict(
                                        college = dict(Block1 = 3, Block2 = 2, Block3 = 3.5),
                                        Block1 = dict(Block4 = 1.5, Block5 = 1),
                                        Block2 = dict(Block5 = 2, Block6 = 1.5),
                                        Block3 = dict(Block7 = 1, Block8 = 1, Block9 = 4),
                                        # Block4 = dict(forest = 6),
                                        Block4 = dict(home = 6),
                                        Block6 = dict(Block10 = 2.5, Block11 = 2),
                                        Block7 = dict(theatre = 4),
                                        Block8 = dict(theatre = 5),
                                        Block9 = dict(theatre = 6),
                                        Block10 = dict(home = 2),
                                        Block11 = dict(home = 1),
                                        theatre = dict(home = 3)
                                        )
                                    )
        
        
        # city_map.locations = dict(
        #                           college = (0,0), Block1 = (1,1), Block2 = (1,2),
        #                           Block3 = (1,3), Block4 = (2,1), Block5 = (3,1),
        #                           Block6 = (5,1), Block7 = (5,2), Block8 = (5,3),
        #                           Block9 = (5,4), Block10 = (4,6), Block11 = (4,5),
        #                           home = (6,6), forest = (7,7), theatre = (6,8),
        #                           )
        
        city_map.locations = dict(
                                  college = (0,0), Block1 = (1,1), Block2 = (1,2),
                                  Block3 = (1,3), Block4 = (2,1), Block5 = (3,1),
                                  Block6 = (5,1), Block7 = (5,2), Block8 = (5,3),
                                  Block9 = (5,4), Block10 = (4,6), Block11 = (4,5),
                                  home = (6,6), theatre = (6,8)
                                  )
        
        return city_map, city_map.locations

    def is_done(self):
        # Goal is to reach the specified destination, so either we consider that or check the fuel in the agent
        if (self.agents[0].location == self.agents[0].goal_location).all() or (self.agents[0].fuel <= 0):
            return True
        return False

class AutonomousDriver(Agent):
    
    # start_location = np.array([0,0])
    location = np.array([0,0])
    goal_location = np.array([6,6])
    wait_time = 1
    fuel = 100
    performance = 100
    # current_location = start_location
    direction = "U"
    states = {}
    
    # def __init__(self, fuel, performance, program, start_location, goal_location = None):
    #     super().__init__(program)
    #     self.start_location = np.array(start_location)
    #     self.goal_location = np.array(goal_location)
    #     self.wait_time = 10 #seconds to wait for traffic signal
    #     self.fuel = fuel
    #     self.performance = performance
    #     self.current_location = self.start_location
    #     self.direction = "U" # ( permitted values: {U, D, L, R}, by default it will be U (for move ahead or upward))
    
    def update_state(self, action):
        self.states[tuple(self.location)] = action
    
    def check_current_state(self):
      # if (self.current_location == self.goal_location).all():.
      if (self.location == self.goal_location).all():
          return True
      return False
        
    def turnright(self):
        if self.direction == "U":
            # self.current_location[1] += 1
            self.location[1] += 1
            self.fuel -= 5
            self.performance -= 5
            self.direction = "R"
            return True
        
        elif self.direction == "L":
            # self.current_location[0] += 1
            self.location[0] += 1
            self.fuel -= 5
            self.performance = -5
            self.direction = "U"
            return True
        
        elif self.direction == "R":
            # self.current_location[0] -= 1
            self.location[0] -= 1
            self.performance += 5
            self.direction = "D"
            return True
        
        elif self.direction == "D":
            # self.current_location[1] -= 1
            self.location[1] -= 1
            self.fuel -= 5
            self.performance -= 5
            self.direction = "L"
            return True
        
    def turnleft(self):
        if self.direction == "U":
            # self.current_location[1] -= 1
            self.location[1] -= 1
            self.fuel -= 5
            self.performance -= 5
            self.direction = "L"
            return True
        
        elif self.direction == "R":
            # self.current_location[0] += 1
            self.location[0] += 1
            self.fuel -= 5
            self.performance -= 5
            self.direction = "U"
            return True
        
        elif self.direction == "L":
            # self.current_location[0] -= 1
            self.location[0] -= 1
            self.performance += 5
            self.direction = "D"
            return True
        
        elif self.direction == "D":
            # self.current_location[1] += 1
            self.location[1] += 1
            self.fuel -= 5
            self.performance -= 5
            self.direction = "R"
            return True
    
    def moveahead(self):
        if self.direction == "U":
            # self.current_location[0] += 1
            self.location[0] += 1
            self.fuel -= 5
            self.performance -= 5
            return True
        
        elif self.direction == "R":
            # self.current_location[1] += 1
            self.location[1] += 1
            self.fuel -= 5
            self.performance -= 5
            return True
        
        elif self.direction == "L":
            # self.current_location[1] -= 1
            self.location[1] -= 1
            self.fuel -= 5
            self.performance -= 5
            return True
        
        elif self.direction == "D":
            # self.current_location[0] -= 1
            self.location[0] -= 1
            self.performance += 5
            return True
    
    def movediagonal(self):
        # self.current_location += 1
        self.location += 1
        self.direction = "U"
        return True
    
    def movediagonaldown(self):
        # self.current_location -= 1
        self.location -= 1
        self.direction = "D"
        return True
    
    def wait(self):
        time.sleep(self.wait_time)
        self.performance += 5
        self.moveahead()
        return True
    
    def search(self, problem, search_method):
        if search_method == "BFTS":
            print("    Running Breadth First Tree Search technique")
            search_result = breadth_first_tree_search(problem)
            print("    Breadth First Tree Search technique completed")
            return search_result
        elif search_method == "DFGS":
            print("    Running Depth First Graph Search technique")
            search_result = depth_first_graph_search(problem)
            print("    Depth First Graph Search technique completed")
            return search_result
        elif search_method == "UCS":
            print("    Running Uniform Cost Search technique")
            search_result = uniform_cost_search(problem)
            print("    Uniform Cost Search technique is complete")
            return search_result
        elif search_method == "BFGS":
            print("    Running Best First Graph Search technique")
            search_result = best_first_graph_search(problem, lambda node: node.state)
            print("    Best First Graph Search technique complete")
            return search_result
        elif search_method == "ASTS":
            print("    Running A* Search technique")
            search_result = astar_search(problem)
            print("    A* Search complete")
            return search_result
        elif search_method == "RBFS":
            print("    Running Recursive Best First Search technique")
            search_result = recursive_best_first_search(problem)
            print("    Recursive Best First Search technique complete")
            return search_result
        
    def get_current_location_name(self, location, city_map_locations):
        for k, v in city_map_locations.items():
            if tuple(self.location) == v:
                return k

#Add things to the environment things list
def Add_things(env, thing, location):
    env_size = np.array([env.width, env.height])
    if (location <= env_size).all():
        env.add_thing(thing, location)
    else:
        print("can not add {} at location: {} as environment size is: {}, choose a different location to add".format(thing, location, env_size))
        return
        
def RunEnvironment(agent, env, runs):
    # env_size = np.array([env.width, env.height])
    for i in range(runs):
        if not env.is_done():
            print("\nRun: {}: Agent current location: {}".format(i, agent.location))
            # print("    Performance: ", agent.performance)
            print("    Remaining fuel is: ", agent.fuel)
            env.step()
        else:
            if (agent.location == agent.goal_location).all():
                print("    agent reached its goal which is at {} and current location of agent is: {}".format(agent.location, agent.goal_location))
                return
            elif (agent.fuel <= 0):
                print("    {} has run out of fuel and has been turned off".format(agent))
                return

# Simple Reflex Agent (actions based on the percept)
def SimpleReflexAgent():
    def program(percepts):
        for p in percepts:
            # print("current percept: ", p)
            if isinstance(p, TrafficSignal): 
                return "wait"
            elif isinstance(p, TurnRightSignBoard):
                return "turnright"
            elif isinstance(p, TurnLeftSignBoard):
                return "turnleft"
        return "moveahead"
    
    # return AutonomousDriver(100, 100, program, [0,0])
    return AutonomousDriver(program)


# Actions based on percepts which have been percieved earlier
def ModelBasedAgent():
    model = {}
    state = {}
    def program(percepts):
        print("    model: ", model)
        print("    percepts:", percepts)
        for p in percepts:
            if str(p) in model.keys():
                print("    percept matched: ", p)
                action = model[str(p)]
                print("    returning action: ", action)
                return action
            if isinstance(p, TrafficSignal):
                action = "wait"
                model[str(p)] = action
                return action
            elif isinstance(p, TurnRightSignBoard):
                action = "turnright"
                model[str(p)] = action
                return action
            elif isinstance(p, TurnLeftSignBoard):
                action = "turnleft"
                model[str(p)] = action
                return action
        return "moveahead"
    return AutonomousDriver(program)


# Goal Based Agent (Actions based on the goal and also percepts while agent is on its way)
def GoalBasedAgent():
    model = {}
    def program(percepts):
        # print("    model: ", model)
        print("    percepts:", percepts)
        
        for p in percepts:
            if str(p) in model.keys():
                print("    percept matched: ", p)
                action = model[str(p)]
                print("    returning action: ", action)
                return action
            if isinstance(p, TrafficSignal):
                action = "wait"
                model[str(p)] = action
                return action
            elif isinstance(p, TurnRightSignBoard):
                action = "turnright"
                model[str(p)] = action
                return action
            elif isinstance(p, TurnLeftSignBoard):
                action = "turnleft"
                model[str(p)] = action

            goal_state = p.check_current_state()
            if goal_state:
                return "goal"
            else:
                if (p.location[0] < p.goal_location[0]):
                    p.direction = "U"
                    return "moveahead"
                elif (p.location[0] > p.goal_location[0]):
                    p.direction ="D"
                    return "moveahead"
                elif (p.location[1] < p.goal_location[1]):
                    p.direction = "U"
                    return "turnright"
                else:
                    p.direction = "U"
                    return "turnleft"
    # return AutonomousDriver(100, 100, program, [0,0], [3,3])
    return AutonomousDriver(program)

# Problem solving agent (which is also a goal based agent, implement differs from a simple goal based agent)
def GoalBasedProblemSolvingAgent(search_method, city_map, city_map_locations, problem):
    model = {}
    search_algo = search_method
    
    def program(percepts):
        print("    model: ", model)
        print("    percepts:", percepts)
        
        for p in percepts:
            if str(p) in model.keys():
                print("    percept matched: ", p)
                action = model[str(p)]
                print("    returning action: ", action)
                return action
            if isinstance(p, TrafficSignal):
                action = "wait"
                model[str(p)] == action
                return action
            elif isinstance(p, TurnRightSignBoard):
                action = "turnright"
                model[str(p)] = action
                return action
            elif isinstance(p, TurnLeftSignBoard):
                action = "turnleft"
                model[str(p)] = action
                return action

            goal_state = p.check_current_state()
            if goal_state:
                return "goal"
            else:
                current_location_name = p.get_current_location_name(p.location, city_map_locations)
                city_problem = GraphProblem(current_location_name, 'home', city_map)
                result = p.search(city_problem, search_algo)
                sequence = result.solution()
                print("    Solution obtained from {}technique is: {}".format(search_algo, sequence))
                print("    Path cost of {} is: {} ".format(search_algo, result.path_cost))
                # print("***********************************************")
                for location_name in sequence:
                    location_coordinate = np.array(city_map_locations[location_name])
                    while not (p.location == location_coordinate).all():
                        # print("------------->Inside while loop<----------------")
                        if (p.location[0] < location_coordinate[0]):
                            p.location[0] += 1
                            p.fuel -= 5
                            p.performance -= 5
                        elif (p.location[0] > location_coordinate[0]):
                            p.location[0] -= 1
                            p.performance += 5
                        elif (p.location[1] < location_coordinate[1]):
                            p.location[1] += 1
                            p.fuel -= 5
                            p.performance -= 5
                            p.direction = "R"
                        else:
                            p.location[1] -= 1
                            p.fuel -= 5
                            p.performance -= 5
                            p.direction = "L"        

    return AutonomousDriver(program)

def RunSimpleReflexAgent():
    # city1 = City(100,100, color={'AutonomousDriver': (200,0,0), 'TrafficSignal': (0, 200, 200), 'TurnRightSignBoard': (0,200,0), 'TurnLeftSignBoard': (0, 0, 200)})
    city1 = City(100,100)
    car = SimpleReflexAgent()
    signal = TrafficSignal()
    right = TurnRightSignBoard()
    left = TurnLeftSignBoard()
    Add_things(city1, car, [0,0])
    Add_things(city1, signal, [5,0])
    Add_things(city1, right, [8,0])
    Add_things(city1, left, [12,0])
    print("\nDEMONSTRATING A SIMPLE REFLEX AGENT")
    print("_____________________________________")
    RunEnvironment(car, city1, 22)
    print("\nSIMPLE REFLEX AGENT DEMONSTRATION COMPLETE")
    print("*******************************************\n")

def RunGoalBasedAgent():
    # city2 = City(100,100, color={'AutonomousDriver': (200,0,0), 'TrafficSignal': (0, 200, 200), 'TurnRightSignBoard': (0,200,0), 'TurnLeftSignBoard': (0, 0, 200)})
    city2 = City(100,100)
    car = GoalBasedAgent()
    signal = TrafficSignal()
    right = TurnRightSignBoard()
    right1 = TurnRightSignBoard()
    left = TurnLeftSignBoard()
    Add_things(city2, car, [0,0])
    Add_things(city2, right1, [6,1])
    Add_things(city2, signal, [5,0])
    Add_things(city2, right, [8,0])
    Add_things(city2, left, [12,0])
    print("\nDEMONSTRATING A GOAL BASED AGENT")
    print("__________________________________")
    RunEnvironment(car, city2, 20)
    print("\nGOAL BASED AGENT DEMONSTRATION COMPLETE")
    print("****************************************\n")

def RunModelBasedAgent():
    # city3 = City(100,100, color={'AutonomousDriver': (200,0,0), 'TrafficSignal': (0, 200, 200), 'TurnRightSignBoard': (0,200,0), 'TurnLeftSignBoard': (0, 0, 200)})
    city3 = City(100,100)
    car = ModelBasedAgent()
    signal = TrafficSignal()
    right = TurnRightSignBoard()
    right1 = TurnRightSignBoard()
    left = TurnLeftSignBoard()
    left1 = TurnLeftSignBoard()
    Add_things(city3, car, [0,0])
    Add_things(city3, signal, [5,0])
    Add_things(city3, right, [8,0])
    Add_things(city3, right1, [8,4])
    Add_things(city3, left, [6,4])
    Add_things(city3, left1, [6,6])
    print("\nDEMONSTRATING A MODEL BASED AGENT")
    print("___________________________________")
    RunEnvironment(car, city3, 20)
    print("\nMODLE BASED AGENT DEMONSTRATION COMPLETE")
    print("******************************************\n")
    
def executeQuestion1():
    RunSimpleReflexAgent()
    RunGoalBasedAgent()
    RunModelBasedAgent()

def executeQuestion2():
    # city1 = City(100,100, color={'AutonomousDriver': (200,0,0), 'TrafficSignal': (0, 200, 200), 'TurnRightSignBoard': (0,200,0), 'TurnLeftSignBoard': (0, 0, 200)})
    city4 = City(100,100)
    city_map, city_map_locations = city4.getCityMapAndLocations()
    city_problem = GraphProblem('college', 'home', city_map)
    
    search_methods = ["BFTS", "DFGS", "UCS", "BFGS", "ASTS", "RBFS"]
    for method in search_methods:
        agent = GoalBasedProblemSolvingAgent(method, city_map, city_map_locations, city_problem)
        Add_things(city4, agent, [0,0])
        RunEnvironment(agent, city4, 1)
        print("    Agent reached its goal which is at location {} after implementing the {} technique".format(agent.location, method))
        # print("    Agent performance after executing {} search technique is: {}".format(method, agent.performance))
        print("    Remaining fuel with Agent is: ", agent.fuel)
        city4.delete_thing(agent)
      
if __name__ == "__main__":
    print("___________________________________________________________________________________________")
    print("Demonstrating working of Different types of Agents (SimpleReflex, ModelBased and GoalBased):\n")
    executeQuestion1()
    print("*******************************************************************************************")
    print("\nDemonstrating different search techniques (Uninformed and Informed Searches):\n")
    executeQuestion2()
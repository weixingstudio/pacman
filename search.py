# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
  """
  This class outlines the structure of a search problem, but doesn't implement
  any of the methods (in object-oriented terminology: an abstract class).
  
  You do not need to change anything in this class, ever.
  """
  
  def getStartState(self):
     """
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           

def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first [p 74].
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm [Fig. 3.18].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  "*** YOUR CODE HERE ***"
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  print problem
  
  visited = dict()
  state = problem.getStartState()
  frontier = util.Stack()

  node = {}
  node["parent"] = None
  node["action"] = None
  node["state"] = state
  frontier.push(node)

  # DFS, non-recursive implementation
  # by non-recurisve, we need to use stack to record
  # which node  to visit when recall 
  while not frontier.isEmpty():
    node = frontier.pop()
    state = node["state"]
    if visited.has_key(hash(state)):
      continue
    visited[hash(state)] = state

    if problem.isGoalState(state) == True:
      break

    for child in problem.getSuccessors(state):
      if not visited.has_key(hash(child[0])):
        sub_node = {}
        sub_node["parent"] = node
        sub_node["action"] = child[1]
        sub_node["state"] = child[0]
        frontier.push(sub_node)

  actions = []
  while node["action"] != None:
    actions.insert(0, node["action"])
    node = node["parent"]

  return actions

def depthFirstSearch_recursive(problem):
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  print problem

  visited = dict()
  state = problem.getStartState()
  action = []
  DFS(problem, state, action, visited)
  return action

def DFS(problem, state, action, visited):
  if problem.isGoalState(state):
    return True

  for child in problem.getSuccessors(state):
    state = child[0]
    if not visited.has_key(hash(state)):
      visited[hash(state)] = state
      action.append(child[1])
      if DFS(problem, state, action, visited) == True:
        return True
      action.pop()

  return False

def breadthFirstSearch(problem):
  "Search the shallowest nodes in the search tree first. [p 74]"
  "*** YOUR CODE HERE ***"
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  print problem

  frontier = util.Queue()
  visited = dict()

  state = problem.getStartState()
  node = {}
  node["parent"] = None
  node["action"] = None
  node["state"] = state
  frontier.push(node)

  while not frontier.isEmpty():
    node = frontier.pop()
    state = node["state"]
    #print state
    if visited.has_key(state):
      continue

    visited[state] = True
    #print state
    if problem.isGoalState(state) == True:
      break

    for child in problem.getSuccessors(state):
      #if child[0] not in visited:
      sub_node = {}
      sub_node["parent"] = node
      sub_node["state"] = child[0]
      sub_node["action"] = child[1]
      frontier.push(sub_node)

  res = []
  while node["action"] != None:
    res.insert(0, node["action"])
    node = node["parent"]

  print res
  return res
      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  #util.raiseNotDefined()
  frontier = util.PriorityQueue()
  visited = dict()

  state = problem.getStartState()
  node = {}
  node["parent"] = None
  node["action"] = None
  node["state"] = state
  node["cost"] = 0
  frontier.push(node, node["cost"])

  while not frontier.isEmpty():
    node = frontier.pop()
    state = node["state"]
    cost = node["cost"]
    print cost

    if visited.has_key(state):
      continue

    visited[state] = state
    if problem.isGoalState(state) == True:
      break

    for child in problem.getSuccessors(state):
      if not visited.has_key(child[0]):
        sub_node = {}
        sub_node["parent"] = node
        sub_node["state"] = child[0]
        sub_node["action"] = child[1]
        sub_node["cost"] = child[2] + cost
        frontier.push(sub_node, sub_node["cost"])

  res = []
  while node["action"] != None:
    res.insert(0, node["action"])
    node = node["parent"]

  print res
  return res


def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  #util.raiseNotDefined()
  frontier = util.PriorityQueue()
  visited = dict()

  state = problem.getStartState()
  node = {}
  node["parent"] = None
  node["action"] = None
  node["state"] = state
  node["cost"] = 0
  node["eval"] = heuristic(state, problem)
  # A* use f(n) = g(n) + h(n)
  frontier.push(node, node["cost"] + node["eval"])

  while not frontier.isEmpty():
    node = frontier.pop()
    state = node["state"]
    cost = node["cost"]
    v = node["eval"]

    if visited.has_key(state):
      continue

    visited[state] = True
    if problem.isGoalState(state) == True:
      break

    for child in problem.getSuccessors(state):
      if not visited.has_key(child[0]):
        sub_node = {}
        sub_node["parent"] = node
        sub_node["state"] = child[0]
        sub_node["action"] = child[1]
        sub_node["cost"] = child[2] + cost
        sub_node["eval"] = heuristic(sub_node["state"], problem)
        frontier.push(sub_node, sub_node["cost"] + node["eval"])

  res = []
  while node["action"] != None:
    res.insert(0, node["action"])
    node = node["parent"]

  print res
  return res
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
dfs_r = depthFirstSearch_recursive
astar = aStarSearch
ucs = uniformCostSearch

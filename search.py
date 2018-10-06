# sample code used at Discussion Week 1 of CS-171 2018 Summer
# this code is adapted from https://github.com/aimacode/aima-python/blob/master/search.ipynb

maximum_expansion = 20  # some problems don't have solution. so stop search when expand this number of nodes

class Problem():
    def __init__(self, input_graph, heuristic = {}):
        self.graph = input_graph
        self.initial_state = 'S'
        # self.goal_state = goal
        self.heuristic = heuristic

    def actions(self, state):
        """ in a graph problems, action is simply the neighbor nodes from the current node"""
        if state in self.graph:
            return sorted(list(self.graph[state].keys()))       # children are sorted in lexicographic order
        else:
            return []

    def result(self, state, action):
        """ this is redundant function in graph problems because action is simply directing the next state"""
        return action

    def path_cost(self, cost_so_far, s1, action, s2):
        return cost_so_far + self.graph[s1][s2]

    def h(self, state):
        if state in self.heuristic:
            return self.heuristic[state]
        return None

    def goal_test(self, state):
        return 'G' in state         # test if character G is in state, allow multiple goals G1, G2


class SearchNode():
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def child_node(self, problem, action):
        next_state = problem.result(self.state, action)
        if next_state:
            return SearchNode(next_state, self, action, problem.path_cost(self.path_cost, self.state, action, next_state))
        else:
            return None

    def expand(self, problem):
        return [self.child_node(problem, action) for action in problem.actions(self.state)]

    def path(self):
        """ path from the root search node to the current node """
        node = self
        path = []
        while node:
            path.append(node)
            node = node.parent
        return list(reversed(path))

    def solution(self):
        """ sequence of actions from the root to the current node """
        return [node.action for node in self.path()[1:]]    # the initial state has no action

    def __repr__(self):
        return "<Node:state={} with path cost={} from {}>".format(self.state, self.path_cost, self.parent.state if self.parent else None)


def print_frontier(frontier):
    print("\t\tfrontier nodes (use list to simulate algorithm)")
    if frontier:
        for ind, n in enumerate(frontier):
            print("\t\t\t{}:{}".format(ind, n))
    else:
        print("\t\t\tempty")


def print_solution(node):
    if isinstance(node, SearchNode):
        print("solution path:{}".format(['S'] + node.solution()))
        print("solution cost:{}".format(node.path_cost))
    else:
        print("no solution")

# tree_search builds a tree of nodes while doing search
# so, it does not check whether you visited the same state before by alternative way from the input graph
# you can turn tree search to graph search by remembering what nodes you visited (explored set), and
# do a test if a ngenerated node is already in the explored set
def breadth_first_tree_search(problem):
    print("\n\nbreadth_first_tree_search")
    iterations = 1
    frontier = [SearchNode(problem.initial_state, parent=None, action=None, path_cost=0)]
    print_frontier(frontier)
    while frontier and iterations <= maximum_expansion:
        print("\niter:{}".format(iterations))
        node = frontier.pop(0)                      # first-in-first-out, i.e., Queue
        print("\tExpand node:{}".format(node))
        children_nodes = node.expand(problem)       # expand the current node and generate children nodes
        for each_child in children_nodes:
            goal_test_result = problem.goal_test(each_child.state)
            if goal_test_result:
                return each_child
            else:
                frontier.append(each_child)
        print("\tfrontier_after_expand:{}".format(frontier))
        iterations += 1
    return None


def depth_first_tree_search(problem):
    print("\n\ndepth_first_tree_search")
    iterations = 1
    frontier = [SearchNode(problem.initial_state, parent=None, action=None, path_cost=0)]
    print_frontier(frontier)
    while frontier and iterations <= maximum_expansion:
        print("\niter:{}".format(iterations))
        node = frontier.pop()                       # last-in-first-out, i.e., Stack
        print("\tExpand node:{}".format(node))
        children_nodes = node.expand(problem)       # expand the current node and generate children nodes
        temp = []
        for each_child in children_nodes:
            goal_test_result = problem.goal_test(node.state)
            if goal_test_result:
                return each_child
            else:
                temp.append(each_child)
        frontier = frontier + list(reversed(temp))  # to preserve alphabetical order
        print_frontier(frontier)
        iterations += 1
    return None


def informed_tree_search(problem, search_algorithm = 'uniform_cost'):
    """ search algorithm can be one of uniform_cost, greedy_best_first, astar"""
    print("\n\n{} informed_tree_search".format(search_algorithm))
    if search_algorithm == 'uniform_cost':
        eval_f = lambda n: n.path_cost                      # use path cost from the initial state to the current
    elif search_algorithm == 'astar':
        eval_f = lambda n: n.path_cost + problem.h(n.state) # path cost + estimated remaining cost by heuristic
    elif search_algorithm == 'greedy_best_first':           # use estimated remaining cost only
        eval_f = lambda n: problem.h(n.state)
    else:
        print("error unknown search algorithm")
        return None
    iterations = 1
    frontier = [SearchNode(problem.initial_state, parent=None, action=None, path_cost=0)]
    print("frontier:{}".format(frontier))
    while frontier and iterations <= maximum_expansion:
        print("iter:{}".format(iterations))
        node = frontier.pop(0)                              # expand the first node in the priority queue
        print("\tExpand node:{}".format(node))
        goal_test_result =problem.goal_test(node.state)     # do goal test after pop to find the shortest path
        if goal_test_result:
            return node
        children_nodes = node.expand(problem)               # expand the current node and generate children nodes
        frontier.extend(children_nodes)
        frontier.sort(key=eval_f)           # simulate priority queue by sorting a list everytime adding children
        print_frontier(frontier)
        iterations += 1
    print("no solution found")
    return None


# def depth_limited_search(problem, depth_limit):
#     def recursive_dls(node, problem, limit):
#         print("limit={}, node:{}".format(limit, node))
#         # if problem.goal_test(node.state):
#         #     return node
#         if limit == 0:
#             return "cutoff"
#         else:
#             cutoff_occured = False
#             for each_action in problem.actions(node.state):
#                 child = node.child_node(problem, each_action)
#                 if problem.goal_test(child.state):
#                     print("pass goal test {}".format(child))
#                     return child
#                 result = recursive_dls(child, problem, limit - 1)
#                 if result == "cutoff":
#                     cutoff_occured = True
#                 elif result != "failure":
#                     return result
#             if cutoff_occured:
#                 return "cutoff"
#             else:
#                 return "failure"
#     return recursive_dls(SearchNode(problem.initial_state), problem, depth_limit)


def iterative_dls(problem, limit):
    iterations=1
    frontier = [SearchNode(problem.initial_state)]
    print_frontier(frontier)
    while frontier:
        print("\titer={}".format(iterations))
        node = frontier.pop()
        # print("\t\tSelect node:{}".format(node))
        if node.depth < limit:
            print("\t\tExpand node:{}".format(node))    # generate children only if they are within depth limit
            children_nodes = node.expand(problem)
            temp = []       # to preserve alphabetical order, put to temp and add to frontier later
            for each_child in children_nodes:
                if problem.goal_test(each_child.state):
                    return each_child
                if each_child.depth < limit:            # if a child is not a goal and depth is at the limit don't add
                    temp.append(each_child)
            frontier = frontier + list(reversed(temp))
        print_frontier(frontier)
        iterations += 1
    return "cutoff"


def iterative_deepening_search(problem, max_depth_limit):
    print("\n\niterative_deepening_search")
    for depth_limit in range(max_depth_limit):
        print("ids depth_limit={}".format(depth_limit))
        result = iterative_dls(problem, depth_limit)
        if result != "cutoff":
            return result
        else:
            print("\tcutoff")


if __name__ == "__main__":
    """
    We define an input problem by a graph using dictionary
    for example,
    all neighbors from S can be retrived by Graph_Problem['S'].keys()
    an edge from S to A with path cost 3 can be retrieved by Graph_Problem['S']['A']
    you could replace the following graph with the one given in other problems.
    """

    """the following problem is taken from 2017-smrq-cs-171-quiz2"""
    p1 = {
        'S': {'A': 3, 'B': 2, 'C': 1},
        'A': {'D': 6},
        'B': {'E': 4},
        'C': {'G': 20},
        'D': {'F': 1},
        'E': {'G': 8},
        'F': {'G': 1}
    }
    problem1 = Problem(p1, {})
    print_solution(breadth_first_tree_search(problem1))
    print_solution(depth_first_tree_search(problem1))
    print_solution(informed_tree_search(problem1, 'unfirom_cost'))

    """the following problem is taken from 2017-fq-cs-171-quiz1"""
    p2 = {
        'S' : {'A': 4, 'B': 17, 'C':2},
        'A' : {'A': 15, 'B': 3},
        'B' : {'G': 4},
        'C' : {'B':4, 'C':15}
    }
    h2 = {
        'S' : 10,
        'A' : 3,
        'B' : 2,
        'C' : 1,
        'G' : 0
    }
    problem2 = Problem(p2, h2)
    print_solution(breadth_first_tree_search(problem2))
    print_solution(depth_first_tree_search(problem2))
    print_solution(informed_tree_search(problem2, 'uniform_cost'))
    print_solution(informed_tree_search(problem2, 'astar'))
    print_solution(informed_tree_search(problem2, 'greedy_best_first'))
    print_solution(iterative_deepening_search(problem2, max_depth_limit=5))

    """the following problem is taken from 2018-wq-cs-171-midterm; 
    this problem changed the node C and D
    this script generate children by alphabetical order; but the problem is following the left to right"""
    p3 = {
        'S' : {'A': 4, 'B': 10, 'D':50},
        'A' : {'B': 3},
        'B' : {'D': 10, 'C':6},
        'D' : {'G1':6, 'G2':5},
        'C' : {'C':15}
    }
    h3 = {
        'S' : 21,
        'A' : 4,
        'B' : 15,
        'D' : 5,
        'C' : 4,
        'G1' : 0,
        'G2' : 0
    }
    problem3 = Problem(p3, h3)
    print_solution(breadth_first_tree_search(problem3))
    print_solution(depth_first_tree_search(problem3))
    print_solution(iterative_deepening_search(problem3, max_depth_limit=5))
    # print_solution(informed_tree_search(problem3, 'uniform_cost'))
    # print_solution(informed_tree_search(problem3, 'astar'))
    # print_solution(informed_tree_search(problem3, 'greedy_best_first'))

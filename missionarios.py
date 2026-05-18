import collections
import time

# --- Estruturas de Dados ---
class Node:
    def __init__(self, state, parent=None, action=None, depth=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.depth = depth

    def __repr__(self):
        return str(self.state)

# --- Definição do Problema ---
class MissionariesProblem:
    def __init__(self):
        self.initial_state = (3, 3, 1)  # (M, C, Barco)
        self.goal_state = (0, 0, 0)

    def is_goal_state(self, state):
        return state == self.goal_state

    def is_valid(self, m, c):
        if m < 0 or c < 0 or m > 3 or c > 3:
            return False
        if m > 0 and m < c:  # Margem inicial
            return False
        if (3 - m) > 0 and (3 - m) < (3 - c):  # Margem destino
            return False
        return True

    def expand(self, state):
        m, c, b = state
        sucessors = []
        moves = [(1, 0), (2, 0), (0, 1), (0, 2), (1, 1)]
        
        for dm, dc in moves:
            if b == 1: # Saindo da margem A
                new_s = (m - dm, c - dc, 0)
            else:      # Voltando da margem B
                new_s = (m + dm, c + dc, 1)
            
            if self.is_valid(new_s[0], new_s[1]):
                sucessors.append(new_s)
        return sucessors

def get_path(node):
    path = []
    while node:
        path.append(node.state)
        node = node.parent
    return path[::-1]

# --- Algoritmos de Busca ---

def breadth_first_search(problem):

    node = Node(problem.initial_state)
    if problem.is_goal_state(node.state): return get_path(node), 1, 1
    
    frontier = collections.deque([node]) 
    explored = set()
    generated = 1
    
    while frontier:
        node = frontier.popleft()
        if node.state in explored: continue
        
        explored.add(node.state)
        
        for state in problem.expand(node.state):
            generated += 1
            child = Node(state, node, depth=node.depth + 1)
            if state not in explored and all(n.state != state for n in frontier):
                if problem.is_goal_state(state):
                    return get_path(child), generated, len(explored) + 1
                frontier.append(child)
    return None

def depth_first_search(problem):
    node = Node(problem.initial_state)
    frontier = [node] 
    closed = set()
    generated = 1
    
    while frontier:
        node = frontier.pop()
        if node.state in closed: continue
        
        closed.add(node.state)
        if problem.is_goal_state(node.state):
            return get_path(node), generated, len(closed)
        
        for state in problem.expand(node.state):
            generated += 1
            child = Node(state, node, depth=node.depth + 1)
            frontier.append(child)
    return None

def iterative_deepening_search(problem):
    depth = 0
    stats = {'gen': 1, 'vis': 0}
    while True:
        result = depth_limited_search(problem, depth, stats)
        if result != 'cutoff':
            return result, stats['gen'], stats['vis']
        depth += 1

def depth_limited_search(problem, limit, stats):
    node = Node(problem.initial_state)
    return recursive_dls(node, problem, limit, stats, set())

def recursive_dls(node, problem, limit, stats, visited_in_path):
    stats['vis'] += 1
    if problem.is_goal_state(node.state):
        return get_path(node)
    if limit == 0:
        return 'cutoff'
    
    cutoff_occurred = False
    visited_in_path.add(node.state)
    
    for state in problem.expand(node.state):
        if state not in visited_in_path:
            stats['gen'] += 1
            child = Node(state, node, depth=node.depth + 1)
            result = recursive_dls(child, problem, limit - 1, stats, visited_in_path.copy())
            if result == 'cutoff':
                cutoff_occurred = True
            elif result is not None:
                return result
    return 'cutoff' if cutoff_occurred else None

# --- Interface de Execução ---

def run():
    prob = MissionariesProblem()
    algos = [
        ("BFS", breadth_first_search),
        ("DFS", depth_first_search),
        ("IDS", iterative_deepening_search)
    ]
    
    print(f"{'Algoritmo':<10} | {'Passos':<7} | {'Gerados':<10} | {'Visitados':<10} | {'Tempo (ms)':<10}")
    print("-" * 65)
    
    for name, func in algos:
        start = time.perf_counter()
        path, gen, vis = func(prob)
        end = time.perf_counter()
        
        t_ms = (end - start) * 1000
        print(f"{name:<10} | {len(path)-1:<7} | {gen:<10} | {vis:<10} | {t_ms:<10.4f}")
        print(f"Caminho {name}: {path}\n")

if __name__ == "__main__":
    run()
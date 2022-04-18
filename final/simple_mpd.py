import numpy as np

# map = np.array([[-1, -1 ,100,], [-1, -100, -1], [-1, -1, -1]])
# gamma = 0.1
# movementprob = 0.9
# utilites = np.array([[0,0,0],[0,0,0],[0,0,0]])
# utilites_calc = np.array([[0,0,0],[0,0,0],[0,0,0]])

goal = 100
nf = -100

map = np.array([[-1, -1 ,-1, -1, -1], [-1, -1, nf, -1, nf], [-1, goal, -1, -1, -1], [goal, -1, goal, -1, -1], [-1, -1, nf, -1, -1]])
gamma = 0.1
movementprob = 0.9
utilites = np.array([[0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0]])
utilites_calc = np.array([[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]])

initial_state = [0,0]
init2 = [0,4]
init3 = [4,4]
path = [initial_state]
path2 = [init2]
path3 = [init3]
max_state = 4
min_state = 0



num_calls = 0

def utility(state):
    global utilites, num_calls
    num_calls += 1
    # print("state: ", state)

    if state[0] < min_state or state[0] > max_state or state[1] < min_state or state[1] > max_state:
        return -1000

    if map[state[0]][state[1]] == goal:
        utilites[state[0]][state[1]] = map[state[0]][state[1]]
        utilites_calc[state[0]][state[1]] = -1
        return utilites[state[0]][state[1]]  
    
    if not utilites_calc[state[0]][state[1]] == 0:
        return -1000  

    utilites_calc[state[0]][state[1]] = -1
    up = utility([state[0]-1, state[1]])
    right = utility([state[0], state[1]+1])
    down = utility([state[0]+1, state[1]])
    left = utility([state[0], state[1]-1])

    u = map[state[0]][state[1]] + gamma*max(up, down, left, right)
    utilites[state[0]][state[1]] = u 
    
    return u

def pathCalc(map, initial_state, path):
    state = initial_state
    # print('start utility', utilites[state[0]][state[1]])
    while not map[state[0]][state[1]] == goal:
        if state[0]-1 >= min_state:
            up = utilites[state[0]-1][state[1]]
        else:
            up = -1000
        if state[0]+1 <= max_state:
            down = utilites[state[0]+1][state[1]]
        else:
            down = -1000
        if state[1]-1 >= min_state:
            left = utilites[state[0]][state[1]-1]
        else:
            left = -1000
        if state[1]+1 <= max_state:
            right = utilites[state[0]][state[1]+1]
        else:
            right = -1000

        choice = max(up, down, right, left)
        # print("up, down, left, right")
        # print(up)
        # print(down)
        # print(left)
        # print(right)
        # print("state: ", state)

        if right == choice:
            state = [state[0], state[1]+1]
        if left == choice:
            state = [state[0], state[1]-1]
        if up == choice:
            state = [state[0]-1, state[1]]
        if down == choice:
            state = [state[0]+1, state[1]]
        # origonal_state = state
        # newlist = [up,down,right,left]
        # while state in path:
        #     print("went back to same spot")
        #     print("old choice: ", choice)
        #     newlist.remove(choice)
        #     choice = max(newlist)
        #     print("new choice: ", choice)
        #     if right == choice:
        #         state = [origonal_state[0], origonal_state[1]+1]
        #     if left == choice:
        #         state = [origonal_state[0], origonal_state[1]-1]
        #     if up == choice:
        #         state = [origonal_state[0]-1, origonal_state[1]]
        #     if down == choice:
        #         state = [origonal_state[0]+1, origonal_state[1]]

        path.append(state)
        # print("choice: ", choice)
        # print("state: ", state)
        print("path: ", path)
            

utility(initial_state)
pathCalc(map, initial_state, path)

print("map: \n", map)
# print(utilites)
print("path1: ", path)


rmgoal = path[len(path)-1]
map[rmgoal[0]][rmgoal[1]] = -1

# print("adjusted map: \n", map)
utilites = np.array([[0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0]])
utilites_calc = np.array([[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]])
utility(init2)
print("utilites2: ", utilites)
pathCalc(map, init2, path2)

print("path2: ", path2)
rmgoal = path2[len(path2)-1]
map[rmgoal[0]][rmgoal[1]] = -1

utilites = np.array([[0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0]])
utilites_calc = np.array([[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]])
utility(init3)
print("utilites3: \n", utilites)
print("map3: \n", map)
pathCalc(map, init3, path3)
print("path3: ", path3)


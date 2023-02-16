#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2023/1/14 9:51
# @Author  : Xavier Ma
# @Email   : xavier_mayiming@163.com
# @File    : TBS4MOCSP.py
# @Statement : The time-based search (TBS) for the multi-objective constrained shortest path problem
import heapq


def dijkstra(network, source):
    """
    The Dijkstra algorithm for the one-to-all shortest path problem
    :param network: {node1: {node2: length, node3: length, ...}, ...}
    :param source: the source node
    :return:
    """
    # Step 1. Initialization
    nn = len(network)  # node number
    neighbor = find_neighbor(network)
    dis = []
    inf = 1e10
    queue = []
    for i in range(nn):
        if i == source:
            dis.append(0)
        else:
            dis.append(inf)
    heapq.heappush(queue, (0, source))
    searched_node = []

    # Step 2. The main loop
    while queue:
        temp_dis, temp_node = heapq.heappop(queue)
        if temp_node not in searched_node:
            searched_node.append(temp_node)
            for node in neighbor[temp_node]:
                alt = temp_dis + network[temp_node][node]
                if alt < dis[node]:
                    dis[node] = alt
                    heapq.heappush(queue, (alt, node))

    # Step 3. Sort the results
    return dis


def find_speed(network, neighbor, nf, nn):
    """
    Find the ripple-spreading speed and ripple-spreading network
    :param network:
    :param neighbor:
    :param nf:
    :param nn:
    :return:
    """
    min_value = [1e6] * nf
    max_value = [0] * nf
    for i in range(nn):
        for j in neighbor[i]:
            for k in range(nf):
                min_value[k] = min(min_value[k], network[i][j][0][k])
                max_value[k] = max(max_value[k], network[i][j][0][k])
    temp_value = [max_value[i] / min_value[i] for i in range(nf)]
    ind = temp_value.index(min(temp_value))
    new_network = {}
    for i in range(nn):
        new_network[i] = {}
        for j in neighbor[i]:
            new_network[i][j] = network[i][j][0][ind]
    return min_value[ind], new_network


def find_neighbor(network):
    """
    Find the neighbor of each node
    :param network:
    :return: [[the neighbor nodes of node1], [the neighbor nodes of node2], ...]
    """
    neighbor = []
    for i in range(len(network)):
        neighbor.append(list(network[i].keys()))
    return neighbor


def init_constraint(network, destination, constraint, neighbor, nn, nc):
    """
    Initialize the constraint at each node
    :param network:
    :param destination:
    :param constraint:
    :param neighbor:
    :param nn:
    :param nc:
    :return:
    """
    constraint_dict = {}
    for i in range(nn):
        constraint_dict[i] = []
    for n in range(nc):
        temp_network = {}
        for i in range(nn):
            temp_network[i] = {}
        for i in range(nn):
            for j in neighbor[i]:
                temp_network[j][i] = network[i][j][1][n]
        temp_constraint = dijkstra(temp_network, destination)
        for i in range(nn):
            constraint_dict[i].append(constraint[n] - temp_constraint[i])
    return constraint_dict


def dominated(obj1, obj2):
    """
    Check whether obj1 is Pareto dominated by obj2
    :param obj1: the first objective
    :param obj2: the second objective
    :return:
    """
    flag = False
    for i in range(len(obj1)):
        if obj1[i] < obj2[i]:
            return False
        elif obj1[i] > obj2[i]:
            flag = True
    return flag


def fully_dominated(obj1, con1, obj2, con2):
    """
    Check whether obj1 and con1 are fully dominated by obj2 and con2
    :param obj1:
    :param con1:
    :param obj2:
    :param con2:
    :return:
    """
    temp_obj1 = obj1.copy()
    temp_obj2 = obj2.copy()
    temp_obj1.extend(con1)
    temp_obj2.extend(con2)
    return dominated(temp_obj1, temp_obj2)


def new_feasible(ripples, dest_obj, con_dict):
    """
    Find the feasible ripples at the node
    :param ripples: all feasible ripples at the node
    :param dest_obj:
    :param con_dict: the constraint at the node
    :return:
    """
    nw = len(con_dict)
    new_feasible_ripples = []
    remove_ripples = []
    for ripple in ripples:
        # Check the feasibility of constraints
        flag = False
        temp_con = ripple['constraint']
        for i in range(nw):
            if temp_con[i] > con_dict[i]:
                flag = True
                remove_ripples.append(ripple)
                break
        if flag:
            continue

        # Check whether the ripple is dominated by the ripples at the destination node
        temp_obj = ripple['objective']
        for obj in dest_obj:
            if dominated(temp_obj, obj):
                remove_ripples.append(ripple)
                break

    for ripple in remove_ripples:
        ripples.remove(ripple)

    # Delete the ripple that is dominated by other incoming ripples
    for ripple1 in ripples:
        flag = True
        obj1 = ripple1['objective']
        con1 = ripple1['constraint']
        for ripple2 in ripples:
            if ripple1 != ripple2 and fully_dominated(obj1, con1, ripple2['objective'], ripple2['constraint']):
                flag = False
                break
        if flag:
            new_feasible_ripples.append(ripple1)
    return new_feasible_ripples


def find_POR(ripples, omega, dest_obj, obj_set, con_set, con_dict):
    """
    Find the new PORs at a node
    :param ripples:
    :param omega:
    :param dest_obj:
    :param obj_set:
    :param con_set:
    :param con_dict:
    :return:
    """
    new_ripples = []
    new_feasible_ripples = new_feasible(ripples, dest_obj, con_dict)
    if not omega:
        return new_feasible_ripples
    else:
        for ripple1 in new_feasible_ripples:
            flag = True
            obj1 = ripple1['objective']
            con1 = ripple1['constraint']
            for ripple2 in omega:
                if fully_dominated(obj1, con1, obj_set[ripple2], con_set[ripple2]):
                    flag = False
                    break
            if flag:
                new_ripples.append(ripple1)
    return new_ripples


def main(network, source, destination, constraint):
    """
    The TBS for the multi-objective constrained shortest path problem
    :param network: {node1: {node2: [[1, 2, 5], [2, 3]], ...}}
    :param source: the source node
    :param destination: the destination node
    :param constraint: [constraint1, constraint2, ...]
    :return:
    """
    # Step 1. Initialize parameters
    nn = len(network)  # node number
    neighbor = find_neighbor(network)
    nf = len(network[source][neighbor[source][0]][0])  # the number of objective functions
    nc = len(network[source][neighbor[source][0]][1])  # the number of constraints
    s, s_network = find_speed(network, neighbor, nf, nn)
    t = 0
    nr = 0  # the number of ripples - 1
    epicenter_set = []  # epicenter set
    radius_set = []  # radius set
    path_set = []  # path set
    objective_set = []  # objective set
    constraint_set = []  # constraint set
    active_set = []  # active ripple set
    dest_obj = []  # the set of objective values of ripples at the destination node
    omega = {}  # the ripple at each node
    con_dict = init_constraint(network, destination, constraint, neighbor, nn, nc)
    for node in range(nn):
        omega[node] = []

    # Step 2. Initialize the first ripple
    epicenter_set.append(source)
    radius_set.append(0)
    path_set.append([source])
    objective_set.append([0] * nf)
    constraint_set.append([0] * nc)
    active_set.append(nr)
    omega[source].append(nr)
    nr += 1

    # Step 3. The main loop
    while active_set:

        # Step 3.1. Time updates
        t += 1
        incoming_ripples = {}
        remove_ripples = []

        # Step 3.2. Active ripples spread out
        for ripple in active_set:
            radius_set[ripple] += s
            flag_active = False

            # Step 3.3. New incoming ripples
            epicenter = epicenter_set[ripple]
            radius = radius_set[ripple]
            path = path_set[ripple]
            obj = objective_set[ripple]
            con = constraint_set[ripple]
            for node in neighbor[epicenter]:
                temp_length = s_network[epicenter][node]
                if node not in path and temp_length <= radius < temp_length + s:
                    temp_path = path.copy()
                    temp_path.append(node)
                    temp_obj = [obj[i] + network[epicenter][node][0][i] for i in range(nf)]
                    temp_con = [con[i] + network[epicenter][node][1][i] for i in range(nc)]
                    if node in incoming_ripples.keys():
                        incoming_ripples[node].append({
                            'path': temp_path,
                            'radius': radius - temp_length,
                            'objective': temp_obj,
                            'constraint': temp_con,
                        })
                    else:
                        incoming_ripples[node] = [{
                            'path': temp_path,
                            'radius': radius - temp_length,
                            'objective': temp_obj,
                            'constraint': temp_con,
                        }]

                # Step 3.4. Inactive -> active
                if radius < temp_length:
                    flag_active = True
            if not flag_active:
                remove_ripples.append(ripple)
        for ripple in remove_ripples:
            active_set.remove(ripple)

        # Step 3.5. New incoming ripples
        if destination in incoming_ripples.keys():
            new_ripples = find_POR(incoming_ripples[destination], omega[destination], dest_obj, objective_set, constraint_set, con_dict[destination])
            for ripple in new_ripples:
                epicenter_set.append(destination)
                radius_set.append(ripple['radius'])
                path_set.append(ripple['path'])
                objective_set.append(ripple['objective'])
                constraint_set.append(ripple['constraint'])
                dest_obj.append(ripple['objective'])
                omega[destination].append(nr)
                nr += 1

        for node in incoming_ripples.keys():
            if node != destination:
                new_ripples = find_POR(incoming_ripples[node], omega[node], dest_obj, objective_set, constraint_set, con_dict[node])
                for ripple in new_ripples:
                    epicenter_set.append(node)
                    radius_set.append(ripple['radius'])
                    path_set.append(ripple['path'])
                    objective_set.append(ripple['objective'])
                    constraint_set.append(ripple['constraint'])
                    active_set.append(nr)
                    omega[node].append(nr)
                    nr += 1

    # Step 4. Select Pareto-optimal paths from the ripples at the destination node
    result = []
    for ripple1 in omega[destination]:
        flag = True
        obj = objective_set[ripple1]
        for ripple2 in omega[destination]:
            if ripple1 != ripple2 and dominated(obj, objective_set[ripple2]):
                flag = False
                break
        if flag:
            result.append({
                'path': path_set[ripple1],
                'objective': objective_set[ripple1],
                'constraint': constraint_set[ripple1],
            })
    return result


if __name__ == '__main__':
    network1 = {
        0: {1: [[31, 1], [4]], 2: [[22, 7], [3]], 3: [[32, 4], [6]]},
        1: {0: [[31, 1], [4]], 2: [[16, 2], [3]], 4: [[29, 6], [1]]},
        2: {0: [[22, 7], [3]], 1: [[16, 2], [3]], 3: [[17, 4], [9]], 4: [[25, 1], [2]]},
        3: {0: [[32, 4], [6]], 2: [[17, 4], [9]], 4: [[21, 2], [5]]},
        4: {1: [[29, 6], [1]], 2: [[25, 1], [2]], 3: [[21, 2], [5]]},
    }
    print(main(network1, 0, 4, [10]))

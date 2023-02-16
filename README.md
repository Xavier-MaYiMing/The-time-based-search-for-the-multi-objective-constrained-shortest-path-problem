### The Time-Based Search for the Multi-Objective Constrained Shortest Path Problem

##### The Multi-Objective Constrained Shortest Path (MOCSP) problem aims to find all Pareto-optimal paths in a multi-objective shortest path problem under multiple constraints.

| Variables      | Meaning                                                      |
| -------------- | ------------------------------------------------------------ |
| network        | Dictionary, {node1: {node2: [objective, constraint], ...}, ...} |
| source         | The source node                                              |
| destination    | The destination node                                         |
| constraint     | The constraint                                               |
| nn             | The number of nodes                                          |
| neighbor       | Dictionary, {node1: [the neighbor nodes of node1], ...}      |
| s              | The ripple-spreading speed (i.e., the minimum length of arcs) |
| t              | The simulated time index                                     |
| nr             | The number of ripples - 1                                    |
| epicenter_set  | List, the epicenter node of the ith ripple is epicenter_set[i] |
| path_set       | List, the path of the ith ripple from the source node to node i is path_set[i] |
| radius_set     | List, the radius of the ith ripple is radius_set[i]          |
| objective_set  | List, the objective value of the traveling path of the ith ripple is objective_set[i] |
| constraint_set | List, the constraint value of the traveling path of the ith ripple is objective_set[i] |
| active_set     | List, active_set contains all active ripples                 |
| dest_obj       | List, the set of objective values of ripples at the destination node |
| con_dict       | List, con_dict[n] denotes the constraint at node n           |
| omega          | Dictionary, omega[n] = i denotes that ripple i is generated at node n |

----

#### Example 1

![](https://github.com/Xavier-MaYiMing/The-time-based-search-for-the-multi-objective-constrained-shortest-path-problem/blob/main/MOCSP%20example.png)

```python
if __name__ == '__main__':
    network1 = {
        0: {1: [[31, 1], [4]], 2: [[22, 7], [3]], 3: [[32, 4], [6]]},
        1: {0: [[31, 1], [4]], 2: [[16, 2], [3]], 4: [[29, 6], [1]]},
        2: {0: [[22, 7], [3]], 1: [[16, 2], [3]], 3: [[17, 4], [9]], 4: [[25, 1], [2]]},
        3: {0: [[32, 4], [6]], 2: [[17, 4], [9]], 4: [[21, 2], [5]]},
        4: {1: [[29, 6], [1]], 2: [[25, 1], [2]], 3: [[21, 2], [5]]},
    }
    print(main(network1, 0, 4, [10]))
```

##### Output

The TBS finds three Pareto-optimal paths satisfying the constraint.

```python
[
  {'path': [0, 2, 4], 'objective': [47, 8], 'constraint': [5]}, 
  {'path': [0, 1, 4], 'objective': [60, 7], 'constraint': [5]}, 
  {'path': [0, 1, 2, 4], 'objective': [72, 4], 'constraint': [9]}
]
```


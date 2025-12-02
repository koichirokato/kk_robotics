from __future__ import annotations

import dataclasses
import enum
import heapq
import math
import time

import numpy as np

AROUND_NODE_DISTINATION = [[-1, 0], [0, -1], [1, 0], [0, 1]]


class NodeStatus(enum.Enum):
    OPEN = 0
    CLOSE = 1


@dataclasses.dataclass
class Node:
    x: int
    y: int
    parent_node: Node | None = None
    status: NodeStatus = NodeStatus.OPEN
    id: int = -1
    cost_from_start: int = 0
    cost_to_goal: int = 0  # huristic
    cost: int = 0
    is_obstacle: bool = False


class NodeQueue:
    def __init__(self):
        self._heap: list[tuple[float, int, Node]] = []

    def push(self, node: Node):
        heapq.heappush(self._heap, (node.cost, node.id, node))

    def pop(self) -> Node | None:
        while self._heap:
            cost, id, node = heapq.heappop(self._heap)
            if node.status != NodeStatus.CLOSE and not node.is_obstacle:
                return node
        return None


class Planner:
    def __init__(self):
        self._world: np.ndarray = np.zeros((100, 100))
        self._size = 100

    def plan_goal(
        self, start: tuple[float, float], goal: tuple[float, float]
    ) -> list[tuple[float, float]]:
        start_time = time.time()
        start_node = Node(
            id=0,
            x=int(start[0]),
            y=int(start[1]),
            parent_node=None,
            status=NodeStatus.OPEN,
            is_obstacle=False,
            cost_to_goal=int(
                math.sqrt((goal[0] - start[0]) ** 2 + (goal[1] - goal[0]) ** 2)
            ),
        )

        id_to_node = {}
        id_to_node[start_node.id] = start_node
        target_nodes = []
        node_x_y = set()
        current_id = start_node.id

        node_queue = NodeQueue()
        node_queue.push(start_node)

        count = 0
        while True:
            # No.2 1.64 [s] -> 1.34 [s]
            # stop id_to_node.values() 2 times
            # current_node_list = list(id_to_node.values())
            # No.4 0.69 -> 0.08 [s]
            current_node = node_queue.pop()
            count += 1
            if current_node is None:
                # No path to goal
                return []
            current_node.status = NodeStatus.CLOSE

            if current_node.x == goal[0] and current_node.y == goal[1]:
                # Goal
                path = self._get_path_to_goal(id_to_node, current_node)
                print("Elaspled time:", time.time() - start_time)
                print("Loop:", count)
                print("Length of node list:", len(id_to_node))
                return path

            minimum_cost = 100000000
            for i, distination in enumerate(AROUND_NODE_DISTINATION):
                x = current_node.x + distination[0]
                y = current_node.y + distination[1]
                # No.3 1.34 [s] -> 0.69 [s]
                # use node_x_y set()
                if (x, y) in node_x_y:
                    continue

                is_obstacle = bool(self._world[x, y] == 1.0)
                if is_obstacle:
                    continue

                current_id += 1
                node = Node(
                    x=x,
                    y=y,
                    id=current_id,
                    parent_node=current_node,
                    status=NodeStatus.OPEN,
                    is_obstacle=is_obstacle,
                    cost_from_start=current_node.cost_from_start + 1,
                    cost_to_goal=int(
                        math.sqrt((goal[0] - x) ** 2 + (goal[1] - y) ** 2)
                    ),
                )

                node.cost = node.cost_from_start + node.cost_to_goal

                id_to_node[node.id] = node
                target_nodes.append(node)
                node_x_y.add((node.x, node.y))
                node_queue.push(node)

    def set_world(self, grid: np.ndarray) -> None:
        self._world = grid

    def _get_lowest_node(self, node_list: list[Node]) -> Node | None:
        minimum_cost = 100000000
        lowest_node: Node | None = None

        for node in node_list:
            if node.status == NodeStatus.CLOSE or node.is_obstacle:
                continue
            if node.cost < minimum_cost:
                minimum_cost = node.cost
                lowest_node = node

        return lowest_node

    def _get_node_status(
        self, node_list: list[Node], x: int, y: int
    ) -> NodeStatus | None:
        for node in node_list:
            if node.x == x and node.y == y:
                return node.status
        return None

    def _is_node_exist(self, node_list: list[Node], x: int, y: int) -> bool:
        for node in node_list:
            if node.x == x and node.y == y:
                return True
        return False

    def _get_path_to_goal(
        self, node_list: list[Node], goal_node: Node
    ) -> list[tuple[float, float]]:
        path: list[tuple[int, int]] = []

        node = goal_node
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent_node

        path.reverse()
        return path

    def _generate_node(self) -> list[Node]:
        nodes: list[Node] = []

        for x in range(self._size):
            for y in range(self._size):
                nodes.append(
                    Node(
                        x=x,
                        y=y,
                        status=NodeStatus.OPEN,
                        is_obstacle=True if self._world[y, x] == 1.0 else False,
                    )
                )

        return nodes


if __name__ == "__main__":
    planner = Planner()
    planner.plan_goal([0, 0], [10, 10])

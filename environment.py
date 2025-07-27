from shapely.geometry import Polygon, Point, LineString
from typing import List, Dict, Tuple
import random

def create_u_shape_scene() -> Tuple[Dict[str, Polygon], Tuple[Tuple[float, float], Tuple[float, float]]]:
    """U-förmiges Hindernis in der Mitte des Raums."""
    wall = LineString([(6, 18), (6, 8), (16, 8), (16, 18)]).buffer(1.0)
    scene = {"wall": wall}
    limits = ((0, 22), (0, 22))
    start = (2.0, 20.0)
    goal = (20.0, 2.0)
    return scene, limits, (start, goal)

def create_l_shape_scene() -> Tuple[Dict[str, Polygon], Tuple[Tuple[float, float], Tuple[float, float]]]:
    """L-förmiges Hindernis in der Mitte des Raums."""
    wall = LineString([(6, 18), (6, 8), (16, 8), (16, 12), (10, 12), (10, 18)]).buffer(1.0)
    scene = {"wall": wall}
    limits = ((0, 22), (0, 22))
    start = (2.0, 2.0)
    goal = (15.0, 15.0)
    return scene, limits, (start, goal)

def create_corridor_scene() -> Tuple[Dict[str, Polygon], Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Langer Tunnel mit schmalem Durchgang."""
    left = Polygon([(3, 2), (5, 2), (5, 18), (3, 18.0)])
    right = Polygon([(15, 0), (17, 0), (17, 18), (15, 18)])
    block = Polygon([(5, 9), (15, 9), (15, 11), (5, 11)])
    scene = {"left": left, "right": right, "block": block}
    limits = ((0, 20), (0, 20))
    start = (10.0, 5.0)
    goal = (10.0, 15.0)
    return scene, limits, (start, goal)

def create_complex_shape_scene() -> Tuple[Dict[str, Polygon], Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Komplexe Form mit mehreren Hindernissen."""
    wall1 = LineString([(2, 18), (2, 2), (8, 2), (8, 18)]).buffer(1.0)
    wall2 = LineString([(14, 18), (14, 2), (20, 2), (20, 18)]).buffer(1.0)
    wall3 = LineString([(6, 10), (16, 10)]).buffer(1.0)
    scene = {"wall1": wall1, "wall2": wall2, "wall3": wall3}
    limits = ((0, 22), (0, 22))
    start = (5.0, 5.0)
    goal = (17.5, 5.0)
    return scene, limits, (start, goal)

def create_multiple_obstacles_scene() -> Tuple[Dict[str, Polygon], Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Mehrere kleine Hindernisse im Raum."""
    obstacles = {
        f"obstacle_{i}": Polygon([(i * 2, 2), (i * 2 + 1, 2), (i * 2 + 1, 20), (i * 2, 20)])
        for i in range(10)
    }
    scene = obstacles
    limits = ((0, 22), (0, 22))
    start = (1.0, 1.0)
    goal = (5.0, 21.0)
    return scene, limits, (start, goal)

def create_crossed_obstacles_scene() -> Tuple[Dict[str, Polygon], Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Ein X mit einem schmalen Durchgang in der Mitte."""
    wall1 = LineString([(6, 18), (16, 8)]).buffer(1.0)
    wall2 = LineString([(6, 8), (16, 18)]).buffer(1.0)
    scene = {"wall1": wall1, "wall2": wall2}
    limits = ((0, 22), (0, 22))
    start = (1.0, 1.0)
    goal = (20.5, 21.0)
    return scene, limits, (start, goal)


def create_random_obstacles_scene(num_obstacles: int = 10) -> Tuple[Dict[str, Polygon], Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Zufällige Hindernisse im Raum."""
    obstacles = {}
    for i in range(num_obstacles):
        x1 = random.uniform(0, 20)
        y1 = random.uniform(0, 20)
        x2 = x1 + random.uniform(1, 3)
        y2 = y1 + random.uniform(1, 3)
        obstacles[f"obstacle_{i}"] = Polygon([(x1, y1), (x2, y1), (x2, y2), (x1, y2)])
    # Start- und Zielpositionen
    scene = obstacles
    limits = ((0, 22), (0, 22))
    start = (1.0, 1.0)
    goal = (19.0, 19.0)
    return scene, limits, (start, goal)

def get_all_scenes() -> List[Tuple[str, Tuple[Dict[str, Polygon], Tuple[Tuple[float, float], Tuple[float, float]]]]]:
    """Liste aller verfügbaren Szenen."""
    return [
        ("U-Shape", create_u_shape_scene()),
        ("L-Shape", create_l_shape_scene()),
        ("Corridor", create_corridor_scene()),
        ("Complex Shape", create_complex_shape_scene()),
        ("Multiple Obstacles", create_multiple_obstacles_scene()),
        ("Crossed Obstacles", create_crossed_obstacles_scene()),
        ("Random Obstacles", create_random_obstacles_scene(10)),
    ]
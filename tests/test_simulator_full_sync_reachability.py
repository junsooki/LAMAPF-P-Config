from simulator_full_sync import reachable_cells


def test_reachable_cells_respects_walls():
    # 2x3 grid with a vertical wall at x=1
    # shelves at (0,0) only reach left column
    cells = [
        [0, 1, 0],
        [0, 1, 0],
    ]
    reachable = reachable_cells(cells, sources=[(0, 0)])
    assert (0, 0) in reachable
    assert (0, 1) in reachable
    assert (2, 0) not in reachable
    assert (2, 1) not in reachable

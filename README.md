![linux](https://github.com/BlondeBurrito/pathfinding_astar/actions/workflows/build_linux.yml/badge.svg)
![windows](https://github.com/BlondeBurrito/pathfinding_astar/actions/workflows/build_windows.yml/badge.svg)
[![crates.io](https://img.shields.io/crates/v/pathfinding_astar.svg)](https://crates.io/crates/pathfinding_astar)
[![docs](https://img.shields.io/badge/docs-docs.rs-orange.svg)](https://docs.rs/pathfinding_astar)

# pathfinding_astar

This is an enhancement and generalisation of my [previous work](https://github.com/BlondeBurrito/hexagonal_pathfinding_astar) with the A-Star pathfinding algorithm.

The key difference is that with my hexagonal work the library was responsible for determining valid neighbouring nodes for path traversal whereby the user has to target a particular coordinate system and specify the bounds of their hexagonal space. For instance working with offset geometries you could use the following to return the best path:

```rust
let start_node: (i32, i32) = (0, 0);
let mut nodes: HashMap<(i32, i32), f32> = HashMap::new();
nodes.insert((0, 0), 1.0);
// snip inserting all the nodes
let end_node: (i32, i32) = (3, 3);
let min_column = -1;
let max_column = 4;
let min_row = -1;
let max_row = 4;
let orientation = HexOrientation::FlatTopOddUp;
let path = astar_path(
	start_node,
	nodes,
	end_node,
	min_column,
	max_column,
	min_row,
	max_row,
	orientation,
);
```

As you can see there are `min` and `max` bounds which must be supplied as well as an orientation (are your hexagons flat topped or pointy topped? Which column/row has been shifted to allow for flush tesselation?).

This new approach is intended to work with any arrangment of 'travel points' or nodes. To achieve this involves using a data structure whereby a node has knowledge of what its neighbours are and how far away the are, rather than the librar calculating these on the fly.

It fits in these three different use cases which may be useful in gamedev or any application for calculating some kind of path:

## What Is A-Star? <a name="astar"></a>

For a number of interconnected nodes A-Star involves using what's called a heuristic (referred to as weight `W`) to guide the calculations so that an optimal path can be found quickly.

If we take a starting point `S` and wish to move to end point `E` we have two paths we could choose to traverse, to `O1` or to `O2`.

```txt
                        Length:22             W:4
                 S ----------------------> O1
                 |                         |
                 |                         |
        Length:5 |                         | Length:4
                 |                         |
                 ▼                         ▼
                 O2 ---------------------> E
            W:1          Length:20            W:2
```

Each point has an associated weight `W` which is a general measure designed to guide the algorithm.

To find the opitmal path we discover the available routes from our starting point `S`, the distance from `S` to a given point and create an A-Star score for moving along a path. For instance:

* The distance between `S` and `O1` is `22`
* The A-Star score of moving from `S` to `O1` is the sum of the distance moved with the weight of the point, i.e `22 + 4 = 26`

We then consider the alternate route of moving from `S` to `O2`:

* The distance between `S` and `O2` is `5`
* The A-Star score is therefore `5 + 1 = 6`

We can see that currently the most optimal route is to move from `S` to `O2` (it has a lower A-Star score) - as moving to `O2` has a better A-Star score we interrogate this point first.

From `O2` we discover that we can traverse to `E`:

* The overall disatnce covered is now `20 + 5 = 25`
* The A-Star score is the sum of the overall distance and the weight of `E`, `25 + 2 = 27`

So far we have explored:

* `S` to `O1` with an A-Star score of `26`
* `S` to `O2` to `E` with an A-Star score of `27`

As we still have a route avaiable with a better A-Star score we expand it, `O1` to `E`:

* Overall distance `22 + 4 = 26`
* The A-Star score is `26 + 2 = 28`

Now we know which path is better, moving via `O2` has a better final A-Star score (it is smaller).

The idea is that for a large number of points and paths certain routes will not be explored as they'd have much higher A-Star scores, this cuts down on search time.

## Using This Library

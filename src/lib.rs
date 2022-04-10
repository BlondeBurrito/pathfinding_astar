//! An implementation of the A-Star pathfinding algorithm
//!
//! Given a collection of nodes you can find the most optimal route from one node to another (if it exists) by providing:
//!
//! * Your starting node
//! * Your end or target node
//! * A map of nodes containing their weight heuristic and what neighbours they have with the respective distances to each one.
//!   * Note that weight is setup such that large weight values indicate a difficult node to traverse
//!
//! If a route does not exist the library will return `None`, otherwise you'll have `Some(Vec<T>)` containing the node labels of the best path, where the type `T` corresponds to what you've used to uniquely label your nodes. Note `T` must implement the `Eq`, `Hash`, `Debug`, `Copy` and `Clone` traits, typically I use `i32` or `(i32, i32)` as labels which satisfy this.
//!
//! Note that if your node weightings are very similar then the algorithm may give you the second or third highly optimal path rather than the best, tuning your weightings is how to ensure the best result but in most cases the second/third route is good enough - this arises from cases where multiple nodes end up having the same A-Star score and the first one of them which gets processed in turn generates a good A-Star score for your end node and that is returned.
//!
//! So in general choose a type `T` to label each of your nodes, specify your starting node and ending node, and along with a map of all your nodes you can find a path with the following function:
//!
//! ```txt
//! pub fn astar_path<T>(
//!     start_node: T,
//!     nodes: HashMap<T, (Vec<(T, f32)>, f32)>,
//!     end_node: T,
//! ) -> Option<Vec<T>>
//! ```
//!
//!Where `nodes` must also contain your `start_node` and `end_node`. The `HashMap` keys are also your chosen label to uniquely identify nodes and the value tuple has two parts:
//!
//! * A vector of neighbours with the same type label and the distance between that neighbour and the current key as an `f32`
//! * An `f32` weighting for the node which will guide the algorithm
//!

use std::{collections::HashMap, fmt::Debug, hash::Hash};

/// Will find the most optimal path from `start_node` to `end_node` if it exists.
/// The `nodes` data set uses the keys as labels to uniquely identify a node/travel point.
/// The values take the form of a tuple containing:
/// * Vector of tuples: `(neighbour_label, distance_to_neighbour)` - used to explore possible paths to traverse
/// * Weight - the heuristic which helps judge whether a given route is good or bad
///
/// For instance:
///
/// ```rust
/// use std::collections::HashMap;
/// use pathfinding_astar::astar_path;
///
/// let start: i32 = 0;
/// let end: i32 = 2;
/// let mut nodes: HashMap<i32, (Vec<(i32, f32)>, f32)> = HashMap::new();
/// nodes.insert(0, (vec![(1, 5.0)], 3.0));
/// nodes.insert(1, (vec![(0, 5.0), (2, 4.0)], 2.0));
/// nodes.insert(2, (vec![(1, 4.0)], 6.0));
/// let path = astar_path(start, nodes, end).unwrap();
/// assert_eq!(vec![0, 1, 2], path);
/// ```
pub fn astar_path<T>(
	start_node: T,
	nodes: HashMap<T, (Vec<(T, f32)>, f32)>,
	end_node: T,
) -> Option<Vec<T>>
where
	T: Eq + Hash + Debug + Clone + Copy,
{
	// ensure nodes data contains start and end points
	if !nodes.contains_key(&start_node) {
		panic!("Node data does not contain start node {:?}", start_node);
	}
	if !nodes.contains_key(&end_node) {
		panic!("Node data does not contain end node {:?}", end_node);
	}
	// retreive the weight of the start point
	let start_weight: f32 = match nodes.get(&start_node) {
		Some(x) => x.1,
		None => panic!("Unable to find starting node weight"),
	};

	// Every time we process a new node we add it to a map.
	// If a node has already been recorded then we replace it if it has a better a-star score (smaller number)
	// otherwise we discard it.
	// This is used to optimise the searching whereby if we find a new path to a previously
	// processed node we can quickly decide to discard or explore the new route
	let mut node_astar_scores: HashMap<T, f32> = HashMap::new();

	// add starting node a-star score to data set (starting node score is just its weight)
	node_astar_scores.insert(start_node, start_weight);

	// create a queue of nodes to be processed based on discovery
	// of form (current_node, a_star_score, vec_previous_nodes_traversed, total_distance_traversed)
	// start by add starting node to queue
	let mut queue = vec![(
		start_node,
		start_weight, // we haven't moved so starting node score is just its weight
		Vec::<T>::new(),
		0.0,
	)];

	// If a path exists then the end node will shift to the beginning of the queue and we can return it.
	// If a path does not exist the `queue` will shrink to length 0 and we return `None` through a check
	//  at the end of each loop iteration.
	while queue[0].0 != end_node {
		// Remove the first element ready for processing
		let current_path = queue.swap_remove(0);
		// Grab the neighbours with their distances from the current path so we can explore each
		let neightbours = match nodes.get(&current_path.0) {
			Some(x) => &x.0,
			None => panic!(
				"Node {:?} is not a key in the `nodes` data set",
				current_path.0
			),
		};
		// Process each new path
		for n in neightbours.iter() {
			let distance_traveled_so_far: f32 = current_path.3;
			let distance_to_this_neighbour: f32 = n.1;
			// Calculate the total distance from the start to this neighbour node
			let distance_traveled = distance_traveled_so_far + distance_to_this_neighbour;
			let node_weight: f32 = match nodes.get(&n.0) {
				Some(x) => x.1,
				None => panic!("Unable to find node weight for neighbour {:?}, key probably doesn't exist in `nodes` data set", &n),
			};
			// Now we know the overall distance traveled and the weight of where we're going to we can score it
			let astar_score = a_star_score(distance_traveled, node_weight);
			// Create a vector of the nodes traversed to get to this `n`
			let mut previous_nodes_traversed = current_path.2.clone();
			previous_nodes_traversed.push(current_path.0);
			// Update the a-star data set.
			// If it already has a record of this node we choose to either update it or ignore this new path as it is worse than what we have calculated in a previous iteration
			if node_astar_scores.contains_key(&n.0) {
				if node_astar_scores.get(&n.0) >= Some(&astar_score) {
					// `node_astar_scores` contains a worse score so update the map with the better score
					node_astar_scores.insert(n.0, astar_score);
					// Search the queue to see if we already have a route to this node.
					// If we do but this new path is better then replace it, otherwise discard
					let mut new_queue_item_required_for_node = true;
					for mut q in queue.iter_mut() {
						if q.0 == n.0 {
							// If existing score is worse (higher) then replace the queue item and
							// don't allow a fresh queue item to be added
							if q.1 >= astar_score {
								new_queue_item_required_for_node = false;
								q.1 = astar_score;
								q.2 = previous_nodes_traversed.clone();
								q.3 = distance_traveled;
							}
						}
					}
					// Queue doesn't contain a route to this node, as we have now found a better route
					// update the queue with it so it can be explored
					if new_queue_item_required_for_node {
						queue.push((
							n.0,
							astar_score,
							previous_nodes_traversed,
							distance_traveled,
						));
					}
				}
			} else {
				// No record of node therefore this is the first time it has been visted
				// Update the a-star score data
				node_astar_scores.insert(n.0, astar_score);
				// Update the queue with this new route to process later
				queue.push((
					n.0,
					astar_score,
					previous_nodes_traversed,
					distance_traveled,
				));
			}
		}

		// Sort the queue by a-star sores so each loop processes the current best path
		queue.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

		// As the `queue` is processed elements are removed, neighbours discovered and scores calculated.
		//If the `queue` length becomes zero then it means there are no routes to the `end_node` and we return `None`
		if queue.len() == 0 {
			return None;
		}
	}
	let mut best_path = queue[0].2.clone();
	// add end node to data
	best_path.push(end_node);
	Some(best_path)
}

/// Determines a score to rank a chosen path, lower scores are better
fn a_star_score(distance: f32, weighting: f32) -> f32 {
	distance + weighting
}

#[cfg(test)]
mod tests {
	use super::*;
	use std::collections::HashMap;

	#[test]
	/// Calcualtes the best path based on the a-star explaination in the README.md
	/// ```txt
	///                    Length:22             W:4
	///         W:1  S ----------------------> O1
	///             |                         |
	///             |                         |
	///    Length:5 |                         | Length:4
	///             |                         |
	///             ▼                         ▼
	///             O2 ---------------------> E
	///        W:1          Length:20            W:2
	///  ```
	fn readme_what_is_a_star_example() {
		let start: (i32, i32) = (0, 0);
		let mut nodes: HashMap<(i32, i32), (Vec<((i32, i32), f32)>, f32)> = HashMap::new();
		nodes.insert((0, 0), (vec![((0, 1), 22.0), ((0, 2), 5.0)], 1.0)); // S
		nodes.insert((0, 1), (vec![((0, 3), 4.0)], 4.0)); // O1
		nodes.insert((0, 2), (vec![((0, 3), 20.0)], 1.0)); // O2
		nodes.insert((0, 3), (vec![], 2.0)); // E
		let end: (i32, i32) = (0, 3);
		let path = astar_path(start, nodes, end).unwrap();
		let actual_path = vec![(0, 0), (0, 2), (0, 3)];
		assert_eq!(actual_path, path);
	}
	#[test]
	#[should_panic]
	/// Expect a panic if the `nodes` data set doesn't contain the starting node
	fn missing_start_node() {
		let start = (0, 0);
		let end = (0, 1);
		let mut nodes: HashMap<(i32, i32), (Vec<((i32, i32), f32)>, f32)> = HashMap::new();
		nodes.insert((0, 1), (vec![((0, 3), 4.0)], 4.0));
		let _path = astar_path(start, nodes, end);
	}
	#[test]
	#[should_panic]
	/// Expect a panic if the `nodes` data set doesn't contain the end node
	fn missing_end_node() {
		let start = (0, 0);
		let end = (0, 1);
		let mut nodes: HashMap<(i32, i32), (Vec<((i32, i32), f32)>, f32)> = HashMap::new();
		nodes.insert((0, 0), (vec![((0, 3), 4.0)], 4.0));
		let _path = astar_path(start, nodes, end);
	}
	#[test]
	/// Test for `None` indicating that no path exists to the end node
	fn no_path_exists() {
		let start = 0;
		let end = 5;
		let mut nodes: HashMap<i32, (Vec<(i32, f32)>, f32)> = HashMap::new();
		nodes.insert(0, (vec![(1, 5.0)], 3.0));
		nodes.insert(1, (vec![(0, 5.0), (2, 3.0)], 2.0));
		nodes.insert(2, (vec![(1, 3.0), (3, 2.0), (4, 1.0)], 1.0));
		nodes.insert(3, (vec![(2, 2.0)], 5.0));
		nodes.insert(4, (vec![(2, 1.0)], 2.0));
		// while end node `5` contains a path backwards to node `4`, `4` itself doesn't have a path to it in the first place
		nodes.insert(5, (vec![(4, 3.0)], 6.0));
		let path = astar_path(start, nodes, end);
		assert_eq!(None, path);
	}
	#[test]
	/// Find the best path through the below grid, distance between each square is a unit of 1.0:
	/// ```txt
	/// ________________________
	/// | L:12| L:13| L:14| L:15|
	/// | W:5 | W:8 | W:9 | W:4 |
	/// |_____|_____|_____|_____|
	/// | L:8 | L:9 | L:10| L:11|
	/// | W:1 | W:1 | W:4 | W:3 |
	/// |_____|_____|_____|_____|
	/// | L:4 | L:5 | L:6 | L:7 |
	/// | W:1 | W:9 | W:14| W:6 |
	/// |_____|_____|_____|_____|
	/// | L:0 | L:1 | L:2 | L:3 |
	/// | W:1 | W:7 | W:3 | W:7 |
	/// |_____|_____|_____|_____|
	/// ```
	fn grid_like_path() {
		let start = 0;
		let end = 15;
		let mut nodes: HashMap<i32, (Vec<(i32, f32)>, f32)> = HashMap::new();
		nodes.insert(0, (vec![(4, 1.0), (1, 1.0)], 1.0));
		nodes.insert(1, (vec![(5, 1.0), (2, 1.0), (0, 1.0)], 7.0));
		nodes.insert(2, (vec![(6, 1.0), (3, 1.0), (1, 1.0)], 3.0));
		nodes.insert(3, (vec![(7, 1.0), (2, 1.0)], 7.0));
		nodes.insert(4, (vec![(8, 1.0), (5, 1.0), (0, 1.0)], 1.0));
		nodes.insert(5, (vec![(9, 1.0), (6, 1.0), (1, 1.0), (4, 1.0)], 9.0));
		nodes.insert(6, (vec![(10, 1.0), (7, 1.0), (2, 1.0), (5, 1.0)], 14.0));
		nodes.insert(7, (vec![(11, 1.0), (3, 1.0), (6, 1.0)], 6.0));
		nodes.insert(8, (vec![(12, 1.0), (9, 1.0), (4, 1.0)], 1.0));
		nodes.insert(9, (vec![(13, 1.0), (10, 1.0), (5, 1.0), (8, 1.0)], 1.0));
		nodes.insert(10, (vec![(14, 1.0), (11, 1.0), (6, 1.0), (9, 1.0)], 4.0));
		nodes.insert(11, (vec![(15, 1.0), (7, 1.0), (10, 1.0)], 3.0));
		nodes.insert(12, (vec![(13, 1.0), (8, 1.0)], 5.0));
		nodes.insert(13, (vec![(14, 1.0), (9, 1.0), (12, 1.0)], 8.0));
		nodes.insert(14, (vec![(15, 1.0), (10, 1.0), (13, 1.0)], 9.0));
		nodes.insert(15, (vec![(11, 1.0), (14, 1.0)], 4.0));

		let path = astar_path(start, nodes, end).unwrap();
		let actual = vec![0, 4, 8, 9, 10, 11, 15];
		assert_eq!(actual, path);
	}
	#[test]
	/// Calcualtes the best path from S to E simulating a hexagonal grid (distance from one hexagon to another is the same assuming a path orthognal to an edge, we use unit size of 1.0 for distance)
	///```txt
	///                 _________               _________
	///                /         \             /         \
	///               /           \           /     E     \
	///     _________/    (1,3)    \_________/    (3,3)    \
	///    /         \             /         \             /
	///   /           \    W:2    /           \    W:2    /
	///  /    (0,3)    \_________/    (2,3)    \_________/
	///  \             /         \             /         \
	///   \    W:3    /           \    W:9    /           \
	///    \_________/    (1,2)    \_________/    (3,2)    \
	///    /         \             /         \             /
	///   /           \    W:4    /           \    W:5    /
	///  /    (0,2)    \_________/    (2,2)    \_________/
	///  \             /         \             /         \
	///   \    W:1    /           \    W:8    /           \
	///    \_________/    (1,1)    \_________/    (3,1)    \
	///    /         \             /         \             /
	///   /           \    W:9    /           \    W:4    /
	///  /    (0,1)    \_________/    (2,1)    \_________/
	///  \             /         \             /         \
	///   \    W:1    /           \    W:6    /           \
	///    \_________/    (1,0)    \_________/    (3,0)    \
	///    /         \             /         \             /
	///   /     S     \    W:2    /           \    W:3    /
	///  /    (0,0)    \_________/    (2,0)    \_________/
	///  \             /         \             /
	///   \    W:1    /           \    W:2    /
	///    \_________/             \_________/
	///  ```
	fn astar_hexagon_up_right() {
		let start_node: (i32, i32) = (0, 0);
		let mut nodes: HashMap<(i32, i32), (Vec<((i32, i32), f32)>, f32)> = HashMap::new();
		nodes.insert((0, 0), (vec![((0, 1), 1.0), ((1, 0), 1.0)], 1.0));
		nodes.insert(
			(0, 1),
			(
				vec![((0, 2), 1.0), ((1, 1), 1.0), ((1, 0), 1.0), ((0, 0), 1.0)],
				1.0,
			),
		);
		nodes.insert(
			(0, 2),
			(
				vec![((0, 3), 1.0), ((1, 2), 1.0), ((1, 1), 1.0), ((0, 1), 1.0)],
				1.0,
			),
		);
		nodes.insert(
			(0, 3),
			(vec![((1, 3), 1.0), ((1, 2), 1.0), ((0, 2), 1.0)], 3.0),
		);
		nodes.insert(
			(1, 0),
			(
				vec![
					((1, 1), 1.0),
					((2, 1), 1.0),
					((2, 0), 1.0),
					((0, 0), 1.0),
					((0, 1), 1.0),
				],
				2.0,
			),
		);
		nodes.insert(
			(1, 1),
			(
				vec![
					((1, 2), 1.0),
					((2, 2), 1.0),
					((2, 1), 1.0),
					((1, 0), 1.0),
					((0, 1), 1.0),
					((0, 2), 1.0),
				],
				9.0,
			),
		);
		nodes.insert(
			(1, 2),
			(
				vec![
					((1, 3), 1.0),
					((2, 3), 1.0),
					((2, 2), 1.0),
					((1, 1), 1.0),
					((0, 2), 1.0),
					((0, 3), 1.0),
				],
				4.0,
			),
		);
		nodes.insert(
			(1, 3),
			(vec![((2, 3), 1.0), ((1, 2), 1.0), ((0, 3), 1.0)], 2.0),
		);
		nodes.insert(
			(2, 0),
			(vec![((2, 1), 1.0), ((3, 0), 1.0), ((1, 0), 1.0)], 2.0),
		);
		nodes.insert(
			(2, 1),
			(
				vec![
					((2, 2), 1.0),
					((3, 1), 1.0),
					((3, 0), 1.0),
					((2, 0), 1.0),
					((1, 0), 1.0),
					((1, 1), 1.0),
				],
				6.0,
			),
		);
		nodes.insert(
			(2, 2),
			(
				vec![
					((2, 3), 1.0),
					((3, 2), 1.0),
					((3, 1), 1.0),
					((2, 1), 1.0),
					((1, 1), 1.0),
					((1, 2), 1.0),
				],
				8.0,
			),
		);
		nodes.insert(
			(2, 3),
			(
				vec![
					((3, 3), 1.0),
					((3, 2), 1.0),
					((2, 2), 1.0),
					((1, 2), 1.0),
					((1, 3), 1.0),
				],
				9.0,
			),
		);
		nodes.insert(
			(3, 0),
			(vec![((3, 1), 1.0), ((2, 0), 1.0), ((2, 1), 1.0)], 3.0),
		);
		nodes.insert(
			(3, 1),
			(
				vec![((3, 2), 1.0), ((3, 0), 1.0), ((2, 1), 1.0), ((2, 2), 1.0)],
				4.0,
			),
		);
		nodes.insert(
			(3, 2),
			(
				vec![((3, 3), 1.0), ((3, 1), 1.0), ((2, 2), 1.0), ((2, 3), 1.0)],
				5.0,
			),
		);
		nodes.insert((3, 3), (vec![((3, 2), 1.0), ((2, 3), 1.0)], 2.0));
		let end_node: (i32, i32) = (3, 3);
		let path = astar_path(start_node, nodes, end_node).unwrap();
		let actual = vec![(0, 0), (1, 0), (2, 1), (3, 1), (3, 2), (3, 3)];
		assert_eq!(actual, path);
	}
	#[test]
	/// Calcualtes the best path from S (3, 3) to E (0, 0)
	///```txt
	///                 _________               _________
	///                /         \             /         \
	///               /           \           /     S     \
	///     _________/    (1,3)    \_________/    (3,3)    \
	///    /         \             /         \             /
	///   /           \    W:2    /           \    W:2    /
	///  /    (0,3)    \_________/    (2,3)    \_________/
	///  \             /         \             /         \
	///   \    W:3    /           \    W:9    /           \
	///    \_________/    (1,2)    \_________/    (3,2)    \
	///    /         \             /         \             /
	///   /           \    W:4    /           \    W:5    /
	///  /    (0,2)    \_________/    (2,2)    \_________/
	///  \             /         \             /         \
	///   \    W:1    /           \    W:8    /           \
	///    \_________/    (1,1)    \_________/    (3,1)    \
	///    /         \             /         \             /
	///   /           \    W:9    /           \    W:4    /
	///  /    (0,1)    \_________/    (2,1)    \_________/
	///  \             /         \             /         \
	///   \    W:1    /           \    W:6    /           \
	///    \_________/    (1,0)    \_________/    (3,0)    \
	///    /         \             /         \             /
	///   /     E     \    W:2    /           \    W:7    /
	///  /    (0,0)    \_________/    (2,0)    \_________/
	///  \             /         \            /
	///   \    W:1    /           \    W:2    /
	///    \_________/             \_________/
	///  ```
	fn astar_hexagon_down_left() {
		let start_node: (i32, i32) = (3, 3);
		let mut nodes: HashMap<(i32, i32), (Vec<((i32, i32), f32)>, f32)> = HashMap::new();
		nodes.insert((0, 0), (vec![((0, 1), 1.0), ((1, 0), 1.0)], 1.0));
		nodes.insert(
			(0, 1),
			(
				vec![((0, 2), 1.0), ((1, 1), 1.0), ((1, 0), 1.0), ((0, 0), 1.0)],
				1.0,
			),
		);
		nodes.insert(
			(0, 2),
			(
				vec![((0, 3), 1.0), ((1, 2), 1.0), ((1, 1), 1.0), ((0, 1), 1.0)],
				1.0,
			),
		);
		nodes.insert(
			(0, 3),
			(vec![((1, 3), 1.0), ((1, 2), 1.0), ((0, 2), 1.0)], 3.0),
		);
		nodes.insert(
			(1, 0),
			(
				vec![
					((1, 1), 1.0),
					((2, 1), 1.0),
					((2, 0), 1.0),
					((0, 0), 1.0),
					((0, 1), 1.0),
				],
				2.0,
			),
		);
		nodes.insert(
			(1, 1),
			(
				vec![
					((1, 2), 1.0),
					((2, 2), 1.0),
					((2, 1), 1.0),
					((1, 0), 1.0),
					((0, 1), 1.0),
					((0, 2), 1.0),
				],
				9.0,
			),
		);
		nodes.insert(
			(1, 2),
			(
				vec![
					((1, 3), 1.0),
					((2, 3), 1.0),
					((2, 2), 1.0),
					((1, 1), 1.0),
					((0, 2), 1.0),
					((0, 3), 1.0),
				],
				4.0,
			),
		);
		nodes.insert(
			(1, 3),
			(vec![((2, 3), 1.0), ((1, 2), 1.0), ((0, 3), 1.0)], 2.0),
		);
		nodes.insert(
			(2, 0),
			(vec![((2, 1), 1.0), ((3, 0), 1.0), ((1, 0), 1.0)], 2.0),
		);
		nodes.insert(
			(2, 1),
			(
				vec![
					((2, 2), 1.0),
					((3, 1), 1.0),
					((3, 0), 1.0),
					((2, 0), 1.0),
					((1, 0), 1.0),
					((1, 1), 1.0),
				],
				6.0,
			),
		);
		nodes.insert(
			(2, 2),
			(
				vec![
					((2, 3), 1.0),
					((3, 2), 1.0),
					((3, 1), 1.0),
					((2, 1), 1.0),
					((1, 1), 1.0),
					((1, 2), 1.0),
				],
				8.0,
			),
		);
		nodes.insert(
			(2, 3),
			(
				vec![
					((3, 3), 1.0),
					((3, 2), 1.0),
					((2, 2), 1.0),
					((1, 2), 1.0),
					((1, 3), 1.0),
				],
				9.0,
			),
		);
		nodes.insert(
			(3, 0),
			(vec![((3, 1), 1.0), ((2, 0), 1.0), ((2, 1), 1.0)], 7.0),
		);
		nodes.insert(
			(3, 1),
			(
				vec![((3, 2), 1.0), ((3, 0), 1.0), ((2, 1), 1.0), ((2, 2), 1.0)],
				4.0,
			),
		);
		nodes.insert(
			(3, 2),
			(
				vec![((3, 3), 1.0), ((3, 1), 1.0), ((2, 2), 1.0), ((2, 3), 1.0)],
				5.0,
			),
		);
		nodes.insert((3, 3), (vec![((3, 2), 1.0), ((2, 3), 1.0)], 2.0));
		let end_node: (i32, i32) = (0, 0);
		let path = astar_path(start_node, nodes, end_node).unwrap();
		let actual = vec![(3, 3), (3, 2), (3, 1), (2, 1), (1, 0), (0, 0)];
		assert_eq!(actual, path);
	}
}

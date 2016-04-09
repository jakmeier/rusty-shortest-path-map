//! This module can maintain all the shortest paths on a two dimensional map
//! Only rectangular objects are allowed and the paths only go vertically or horizontally

#[cfg(test)]
pub mod unit_tests;

use std::cmp::Ordering;
use std::collections::BinaryHeap;


/// Stores a graph with the shortest path from each node to the destination
/// To recompute this, it also keeps in memory what obstucles there are, therefore, if the actual map changes this struct has to be notified
pub struct JkmShortestPathMap {
	graph: Vec<Box<GraphNode>>,
	obstacles: Vec<(f64,f64,f64,f64)>,
	start_point_index: usize,
	end_point_index: usize,
	map: (f64,f64,f64,f64),
}

	// line: (x, y, x2)
	// obstacle: (x, y, w, h)
	fn h_line_crosses_obstacle (line:(f64, f64, f64), obstacle: (f64, f64, f64, f64) ) -> bool {
		line.0 < obstacle.0 + obstacle.2 && line.2 > obstacle.0  
		&& line.1 > obstacle.1 && line.1 < obstacle.1 + obstacle.3 
	}
	
	// line: (x, y, y2)
	// obstacle: (x, y, w, h)
	fn v_line_crosses_obstacle (line:(f64, f64, f64), obstacle: (f64, f64, f64, f64) ) -> bool {
		line.1 < obstacle.1 + obstacle.3 && line.2 > obstacle.1  
		&& line.0 > obstacle.0 && line.0 < obstacle.0 + obstacle.2 
	}

impl JkmShortestPathMap {

	/// Create graph with a starting point, a destination and a rectangular map.
	/// The graph will contain 2 or 4 vertices after the creation, the starting and the end point
	/// as well as two connection points in case they cannot be connnected with one straight line. 
	
	/// #Panics 
	/// Panics when the start point is identical with the end point or if they are not both within the map.
	
	pub fn new (start: (f64, f64), end: (f64, f64), map: (f64,f64,f64,f64)) -> JkmShortestPathMap {
		let mut g = Vec::new();
		let mut s = Box::new(GraphNode::new(start.0, start.1));
		let mut start_index = 1;
		let mut e = Box::new(GraphNode::new(end.0, end.1));
		e.cost = 0.0;
		g.push(e);
		
		if start.0 == end.0 {
			//vertically connected
			if start.1 < end.1 {
				s.cost = end.1 - start.1;
				s.neighbours[SOUTH] = Some(0);
				g[0].neighbours[NORTH] = Some(start_index);
				s.shortest_path = Some(SOUTH);
			}
			else if end.1 < start.1 {
				s.cost = start.1 - end.1;
				s.neighbours[NORTH] = Some(0);
				g[0].neighbours[SOUTH] = Some(start_index);
				s.shortest_path = Some(NORTH);
			}
			else { panic!("Start and end point identical coordinates."); }
			
		}
		else if start.1 == end.1 {
			//horizontally connected
			if start.0 < end.0 {
				s.cost = end.0 - start.0;
				s.neighbours[EAST] = Some(0);
				g[0].neighbours[WEST] = Some(start_index);
				s.shortest_path = Some(EAST);
			}
			else if end.0 < start.0 {
				s.cost = start.0 - end.0;
				s.neighbours[WEST] = Some(0);
				g[0].neighbours[EAST] = Some(start_index);
				s.shortest_path = Some(WEST);				
			}
			else { panic!("Start and end point identical coordinates."); }

		}
		else {
			//not directly connected => create helper nodes
			s.cost = (start.0 - end.0).abs() + (start.1 - end.1).abs();
			let mut h = Box::new(GraphNode::new(end.0, start.1)); //on same x coordinate as end
			let mut h1 = Box::new(GraphNode::new(start.0, end.1)); // on same y coordinate as end
			h.cost = (start.0 - end.0).abs(); 
			h1.cost = (start.1 - end.1).abs(); 
			start_index += 2;
			
			//connect helper nodes horizontally
			if start.0 < end.0 {
				h.neighbours[EAST] = Some(0);
				g[0].neighbours[WEST] = Some(1);
				h.shortest_path = Some(EAST);
				
				h1.neighbours[WEST] = Some(start_index);
				s.neighbours[EAST] = Some(2);
				s.shortest_path = Some(EAST);
			}
			else {
				h.neighbours[WEST] = Some(0);
				g[0].neighbours[EAST] = Some(1);
				h.shortest_path = Some(WEST);
				
				h1.neighbours[EAST] = Some(start_index);
				s.neighbours[WEST] = Some(2);
				s.shortest_path = Some(WEST);
			}
			
			//connect helper nodes vertically
			if start.1 < end.1 {
				h.neighbours[NORTH] = Some(start_index);
				s.neighbours[SOUTH] = Some(1);
				s.shortest_path = Some(SOUTH);
				
				h1.neighbours[SOUTH] = Some(0);
				g[0].neighbours[NORTH] = Some(2);
				h1.shortest_path = Some(SOUTH);
			}
			else {
				h.neighbours[SOUTH] = Some(start_index);
				s.neighbours[NORTH] = Some(1);
				s.shortest_path = Some(NORTH);
				
				h1.neighbours[NORTH] = Some(0);
				g[0].neighbours[SOUTH] = Some(2);
				h1.shortest_path = Some(NORTH);
			}
			g.push(h);
			g.push(h1);
		} 
		
		g.push(s);
		
		
		
		JkmShortestPathMap {
			graph: g,
			obstacles: Vec::new(),
			end_point_index: 0,
			start_point_index: start_index, 
			map: map, 
		}
	}
	
	///
	pub fn insert_obstacle (&mut self, x: f64, y: f64, w: f64, h: f64) {
		//add obstacle to list
		self.obstacles.push((x,y,w,h));
		
		// Find all edges going thorugh the new obstacle
		let mut h_blocked :BinaryHeap<MinSortableEdge> = BinaryHeap::new();
		let mut v_blocked :BinaryHeap<MinSortableEdge> = BinaryHeap::new();
		
		for v0 in 0..self.graph.len() {
			// only look at one direction of the edges, therefore only look at up and right
			if let Some(up) = self.graph[v0].neighbours[NORTH] {
				if v_line_crosses_obstacle( (self.graph[v0].x, self.graph[v0].y, self.graph[up].y), (x,y,w,h) )
				{
					v_blocked.push(MinSortableEdge((v0,up), self.graph[v0].x));
				}	
			}
			if let Some(right) = self.graph[v0].neighbours[EAST] {
				if h_line_crosses_obstacle( (self.graph[v0].x, self.graph[v0].y, self.graph[right].x), (x,y,w,h) )
				{
					h_blocked.push(MinSortableEdge((v0,right), self.graph[v0].y));
				}
			}
		}
		// Remove all blocked edges, invalidate paths going through these edges
		for blocked_edge in h_blocked.iter() {
			let v0 = (blocked_edge.0).0;
			let v1 = (blocked_edge.0).1;
			self.graph[v0].delete_neighbour(v1);
			self.graph[v1].delete_neighbour(v0);
			if let Some(sp) = self.graph[v0].shortest_path {
				if sp == v1 {
					self.invalidate_paths_through_node(v0);
				}
			}
			if let Some(sp) = self.graph[v1].shortest_path {
				if sp == v0 {
					self.invalidate_paths_through_node(v1);
				}
			}
		}
		
		// Create new vertices and connect them in a circle
		// While doing so, also make a connection to the rest of the graph:
			// From each edge-node  
			//For the corners search in both open directions the closest edge that could be crossed,
			// create a new node there and connect to this.
		let mut lu = Box::new(GraphNode::new(x,y));
		let mut ru = Box::new(GraphNode::new(x+w,y));
		let mut rb = Box::new(GraphNode::new(x+w,y+h));
		let mut lb = Box::new(GraphNode::new(x,y+h));
		
		let i = self.graph.len();
		let v = v_blocked.len();
		let ho = h_blocked.len();
		
		lu.neighbours[EAST] = Some(i+1);
		lu.neighbours[SOUTH] = Some(i+3+2*v+2*ho);
		
		ru.neighbours[WEST] = Some(i+v);
		ru.neighbours[SOUTH] = Some(i+2+v);

		rb.neighbours[NORTH] = Some(i+v+ho+1);
		rb.neighbours[WEST] = Some(i+3+v+ho);

		lb.neighbours[EAST] = Some(i+2+2*v+ho);
		if v == 0 && ho == 0 {lb.neighbours[NORTH] = Some(i);}
		else {lb.neighbours[NORTH] = Some(i+4+2*v+ho);}

		
		self.graph.push(lu); //i
		// add upper line and connect it as described above, no path computation done here
		let mut v_blocked_buf = Vec::new(); // use to buffer reversed order
		for j in 0..v {
			if let Some(edge) = v_blocked.pop() {
				let (top, _) = edge.0;
				let cross_x:f64 = edge.1;
				let mut new_node = Box::new(GraphNode::new(cross_x, y));
				new_node.neighbours[NORTH] = Some(top);
				self.graph[top].neighbours[SOUTH] = Some(i + j + 1);
				new_node.neighbours[EAST] = Some(i+j);
				new_node.neighbours[WEST] = Some(i+j+2);
				self.graph.push(new_node);
				v_blocked_buf.push(edge);
			}
			else {panic!("Something in the graph went wrong. Number of vertices expected: {}, number found: {}\n", v, j);}
		}
		
		self.graph.push(ru); //i + v + 1
		// right line
		let mut h_blocked_buf = Vec::new(); // use to buffer reversed order
		for j in 0..ho {
			if let Some(edge) = h_blocked.pop(){
				let (_, right) = edge.0;
				let cross_y:f64 = edge.1;
				let mut new_node = Box::new(GraphNode::new(x+w, cross_y));
				new_node.neighbours[EAST] = Some(right);
				self.graph[right].neighbours[WEST] = Some(i + v + j + 2);
				new_node.neighbours[NORTH] = Some(i+v+1+j);
				new_node.neighbours[SOUTH] = Some(i+v+3+j);
				self.graph.push(new_node);
				h_blocked_buf.push(edge);
			}
			else {panic!("Something in the graph went wrong. Number of vertices expected: {}, number found: {}\n", ho, j);}
	
		}
		
		self.graph.push(rb); //i+ v + ho +2
		// bottom line
		for j in 0..v {
			if let Some(edge) = v_blocked.pop(){
				let (_, bot) = edge.0;
				let cross_x:f64 = edge.1;
				let mut new_node = Box::new(GraphNode::new(cross_x, y+h));
				new_node.neighbours[SOUTH] = Some(bot);
				self.graph[bot].neighbours[NORTH] = Some(i + v + ho + j + 3);
				new_node.neighbours[EAST] = Some(i + v + ho + j + 4);
				new_node.neighbours[WEST] = Some(i + v + ho + j + 2);
				self.graph.push(new_node);
			}
			else {panic!("Something in the graph went wrong. Number of vertices expected: {}, number found: {}\n", v, j);}

		}
		
		self.graph.push(lb); //i + 2v + ho + 3
		
		if ho > 0 {
		// left line
			for j in 0..(ho-1) {
				if let Some(edge) = h_blocked.pop(){
					let (left, _) = edge.0;
					let cross_y:f64 = edge.1;
					let mut new_node = Box::new(GraphNode::new(x, cross_y));
					new_node.neighbours[WEST] = Some(left);
					self.graph[left].neighbours[EAST] = Some(i + 2 * v + ho + j + 4);
					new_node.neighbours[NORTH] = Some(i + 2 * v + ho + j + 5);
					new_node.neighbours[SOUTH] = Some(i + 2 * v + ho + j + 3);
					self.graph.push(new_node);
				}
				else {panic!("Something in the graph went wrong. Number of vertices expected: {},  found: {}\n", ho, j);}

			}
			if let Some(edge) = h_blocked.pop(){
				let (left, _) = edge.0;
				let cross_y = edge.1;
				let mut new_node = Box::new(GraphNode::new(x, cross_y));
				new_node.neighbours[WEST] = Some(left);
				self.graph[left].neighbours[EAST] = Some(i + 2 * v + 2 * ho + 3);
				new_node.neighbours[NORTH] = Some(i);
				new_node.neighbours[SOUTH] = Some(i + 2 * v + 2 * ho + 2);
				self.graph.push(new_node);
			}
			else {panic!("Something in the graph went wrong. Number of vertices expected: {},  found: {}\n", ho, ho-1);}

		}
	
		// Connect corner vertices with nearest edge in the corresponsing direction, by creating helper vertices where needed
		self.link_to_north(i);
		self.link_to_west(i);
		self.link_to_north(i+v+1);
		self.link_to_east(i+v+1);
		self.link_to_east(i+v+ho+2);
		self.link_to_south(i+v+ho+2);
		self.link_to_south(i + 2 * v + ho + 3); 
		self.link_to_west(i + 2 * v + ho + 3); 
		
		// At this point, the graph should be consistent again
	}
	
	fn invalidate_paths_through_node (&mut self, n: usize) {
		for j in 0..4 {
			if let Some(i) = self.graph[n].neighbours[j] {
				if let Some(s) = self.graph[i].shortest_path {
					if s == n {
						self.graph[i].shortest_path = None;
						self.graph[i].cost = std::f64::INFINITY;
						self.invalidate_paths_through_node (i);
					}
				}
			}
		}
	}
	
	// line: (x, y, x2)
	// obstacle: (x, y, w, h)
	
	fn h_line_crosses_no_obstacle (&self, x0: f64, y: f64, x1: f64) -> bool {
		for &o in self.obstacles.iter() {
			if h_line_crosses_obstacle ( (x0,y,x1), o ) { return false; }
		}
		false
	}
	
	// line: (x, y, y2)
	// obstacle: (x, y, w, h)

	fn v_line_crosses_no_obstacle (&self, x: f64, y0: f64, y1: f64) -> bool {
		for &o in self.obstacles.iter() {
			if h_line_crosses_obstacle ( (x,y0,y1), o ) { return false; }
		}
		false
	}
	
	/// Connects two nodes horizontally if possibly and updates shortest paths that are changed by this new edge
	fn connect_h(&mut self, left: usize, right: usize) {
		debug_assert!(self.graph[left].x < self.graph[right].x && self.graph[left].y == self.graph[right].y, "Left is actually not left from right or they are not alligned.");
		
		if self.h_line_crosses_no_obstacle(self.graph[left].x, self.graph[left].y, self.graph[right].x) {
			let cost = self.graph[right].x - self.graph[left].x;
			self.graph[left].neighbours[EAST] = Some(right);
			self.graph[right].neighbours[WEST] = Some(left);
			
			if self.graph[left].cost + cost < self.graph[right].cost {
				self.graph[right].shortest_path = Some(WEST);
				self.graph[right].cost = self.graph[left].cost + cost;
				self.update_neighbours(right);
			}
			else if self.graph[right].cost + cost < self.graph[left].cost {
				self.graph[left].shortest_path = Some(EAST);
				self.graph[left].cost = self.graph[right].cost + cost;
				self.update_neighbours(left);
			}
		}
	}
	
	/// Connects two nodes vertically if possible and updates shortest paths that are changed by this new edge
	fn connect_v(&mut self, top: usize, bot: usize) {
		debug_assert!(self.graph[top].y < self.graph[bot].y && self.graph[top].x == self.graph[bot].x, "The top node is actually not higher than the bottom one or they are not alligned.");
		if self.v_line_crosses_no_obstacle(self.graph[top].x, self.graph[top].y, self.graph[bot].y) {
			let cost = self.graph[bot].x - self.graph[top].x;
			self.graph[top].neighbours[SOUTH] = Some(bot);
			self.graph[bot].neighbours[NORTH] = Some(top);
			
			if self.graph[top].cost + cost < self.graph[bot].cost {
				self.graph[bot].shortest_path = Some(NORTH);
				self.graph[bot].cost = self.graph[top].cost + cost;
				self.update_neighbours(bot);
			}
			else if self.graph[bot].cost + cost < self.graph[top].cost {
				self.graph[top].shortest_path = Some(SOUTH);
				self.graph[top].cost = self.graph[bot].cost + cost;
				self.update_neighbours(top);
			}
		}
	}
	
	// Call this after adding a node / edge
	// Checks all neighbours if they could have a shorter path when using this path
	fn update_neighbours (&mut self, n: usize) {
		for j in 0..4 {
			if let Some(i) = self.graph[n].neighbours[j] {
				let cost = (self.graph[n].x - self.graph[i].x).abs() + (self.graph[n].y - self.graph[i].y).abs();
				if self.graph[n].cost + cost < self.graph[i].cost {
					self.graph[i].shortest_path = Some((j+2)%4);
					self.graph[i].cost = self.graph[n].cost + cost;
					self.update_neighbours(i);
				}
			}
		}
	}
	
	/// Update a specific node's shortest path by looking at all neighbours
	fn update_node (&mut self, n: usize) {
		let mut result = (self.graph[n].shortest_path, self.graph[n].cost);
		for i in 0..4 {
			if let Some(neighbour) = self.graph[n].neighbours[i] {
				let cost = self.graph[neighbour].cost + self.distance_on_map(n, neighbour);
				if cost < result.1 { result = (Some(i), cost); }
			}
			
		}
		self.graph[n].shortest_path = result.0;
		self.graph[n].cost = result.1;
	}
	
	fn distance_on_map(&self, n0: usize, n1: usize) -> f64 {
		(self.graph[n0].x - self.graph[n1].x).abs() + (self.graph[n0].y - self.graph[n1].y).abs() 
	}
	
	// Take an edge and insert a new node on it.
	fn split_edge(&mut self, n: usize, direction: usize, coordinate: f64) -> usize {
		debug_assert!(self.graph[n].neighbours[direction].is_some());
		debug_assert!(
			(direction == NORTH && self.graph[n].y > coordinate && self.graph[self.graph[n].neighbours[direction].unwrap()].y < coordinate)
			|| (direction == SOUTH && self.graph[n].y < coordinate && self.graph[self.graph[n].neighbours[direction].unwrap()].y > coordinate)
			|| (direction == EAST && self.graph[n].x < coordinate  && self.graph[self.graph[n].neighbours[direction].unwrap()].x > coordinate)
			|| (direction == WEST && self.graph[n].x > coordinate  && self.graph[self.graph[n].neighbours[direction].unwrap()].x < coordinate)
			, "Invalid cut point.\n"
		);
		let other_direction = (direction + 2)%4;
		let new_index = self.graph.len();
		let x;
		let y;
		
		if direction == NORTH || direction == SOUTH { x = self.graph[n].x; y = coordinate; }
		else { x = coordinate; y = self.graph[n].y; }
		
		// because each node only stores the directoin and not the next node on tha shortest path, we don't have to adjust this here for the two consisting nodes
		let other_node = self.graph[n].neighbours[direction].unwrap();
		self.graph.push(Box::new(GraphNode::new(x, y)));
		self.graph[new_index].neighbours[other_direction] = Some(n);
		self.graph[n].neighbours[direction] = Some(new_index);
		self.graph[new_index].neighbours[direction] = Some(other_node);
		self.graph[other_node].neighbours[other_direction] = Some(new_index);
		self.update_node(new_index);
		new_index
	}
	
	// These 4 functions take a node and search the next edge in one direction
	// Then they connect to this edge which usually involves creating a new node
	fn link_to_north(&mut self, n: usize ) {
		debug_assert!(self.graph[n].neighbours[NORTH] == None);
		// We only have to see for right neighbours, otherwise we would look at each edge twice
		// Also we are not interested in edges that go vertically here
		let x = self.graph[n].x;
		let y = self.graph[n].y;
		let mut new_node : Option<(usize,usize,f64)> = None;
		for (i, node) in self.graph.iter().enumerate() {
			if let Some(neighbour) = node.neighbours[EAST] {
				if node.x <=  x && self.graph[neighbour].x >= x && node.y < y {
					if let Some((_, _, height)) = new_node {
						if node.y > height { new_node = Some((i,neighbour, node.y)); }
					}
					else { new_node = Some((i,neighbour, node.y)); }
				}
			}
		}
		if let Some((left, right, _)) = new_node {
			if self.graph[left].x == x {
				self.connect_v(left, n);
			}
			else if self.graph[right].x == x {
				self.connect_v(right, n);
			}
			else {
				let new_index = self.split_edge(left, EAST, x);
				self.connect_v(new_index, n);
			}
			
		}
	}
	fn link_to_east(&mut self, n: usize) {
		debug_assert!(self.graph[n].neighbours[EAST] == None);
		let x = self.graph[n].x;
		let y = self.graph[n].y;
		let mut new_node : Option<(usize,usize,f64)> = None;
		for (i, node) in self.graph.iter().enumerate() {
			if let Some(neighbour) = node.neighbours[SOUTH] {
				if node.y <=  y && self.graph[neighbour].y >= y && node.x > x {
					if let Some((_, _, height)) = new_node {
						if node.x < height { new_node = Some((i,neighbour, node.x)); }
					}
					else { new_node = Some((i,neighbour, node.x)); }
				}
			}
		}
		if let Some((top, bot, _)) = new_node {
			if self.graph[top].y == y {
				self.connect_h(n, top);
			}
			else if self.graph[bot].y == y {
				self.connect_h(n, bot);
			}
			else {
				let new_index = self.split_edge(top, SOUTH, y);
				self.connect_h(n, new_index);
			}
			
		}
	}
	fn link_to_south(&mut self,n: usize) {
		debug_assert!(self.graph[n].neighbours[SOUTH] == None);
		let x = self.graph[n].x;
		let y = self.graph[n].y;
		let mut new_node : Option<(usize,usize,f64)> = None;
		for (i, node) in self.graph.iter().enumerate() {
			if let Some(neighbour) = node.neighbours[EAST] {
				if node.x <=  x && self.graph[neighbour].x >= x && node.y > y {
					if let Some((_, _, height)) = new_node {
						if node.y < height { new_node = Some((i,neighbour, node.y)); }
					}
					else { new_node = Some((i,neighbour, node.y)); }
				}
			}
		}
		if let Some((left, right, _)) = new_node {
			if self.graph[left].x == x {
				self.connect_v(n, left);
			}
			else if self.graph[right].x == x {
				self.connect_v(n, right);
			}
			else {
				let new_index = self.split_edge(left, EAST, x);
				self.connect_v(n, new_index);
			}
			
		}
	}
	fn link_to_west(&mut self, n: usize) {
		debug_assert!(self.graph[n].neighbours[WEST] == None);
		let x = self.graph[n].x;
		let y = self.graph[n].y;
		let mut new_node : Option<(usize,usize,f64)> = None;
		for (i, node) in self.graph.iter().enumerate() {
			if let Some(neighbour) = node.neighbours[SOUTH] {
				if node.y <=  y && self.graph[neighbour].y >= y && node.x < x {
					if let Some((_, _, height)) = new_node {
						if node.x > height { new_node = Some((i,neighbour, node.x)); }
					}
					else { new_node = Some((i,neighbour, node.x)); }
				}
			}
		}
		if let Some((top, bot, _)) = new_node {
			if self.graph[top].y == y {
				self.connect_h(top, n);
			}
			else if self.graph[bot].y == y {
				self.connect_h(bot, n);
			}
			else {
				let new_index = self.split_edge(top, SOUTH, y);
				self.connect_h(new_index, n);
			}
			
		}
	}
	
}



/// Stores: 
/// - An indices of each neighbour
/// - The coordinate of the node
/// - The shortest path's cost and the direction of the next node on this path
/// The shortest path's cost is set to infinity if and only if it is invalid or unkown
struct GraphNode {
	neighbours: [Option<usize>;4],
	x: f64, y: f64, 
	shortest_path: Option<usize>, cost: f64,
}

const NORTH: usize = 0;
const EAST: usize = 1;
const SOUTH: usize = 2;
const WEST: usize = 3;

impl GraphNode {
	pub fn new(x: f64, y: f64,) -> GraphNode {
		GraphNode{
			neighbours: [None, None, None, None],
			x: x, y: y, 
			shortest_path: None, cost: std::f64::INFINITY,
		}
	}
	
	pub fn delete_neighbour(&mut self, node: usize) {
		for i in 0..4 {
			if let Some(n) = self.neighbours[i] {
				if n == node {
					self.neighbours[i] = None;
				}
			}
		}
	}
	
}

/// used for insert_obstacle
#[derive(Copy, Clone, PartialEq)]
struct MinSortableEdge((usize, usize), f64);

impl Ord for MinSortableEdge {
	/// Panics if one or both of the floats is NaN
	fn cmp (&self, other: &MinSortableEdge) -> Ordering {
		if (self.0).0 == (other.0).0 && (self.0).1 == (other.0).1 //Equality defined over edge points
			{return Ordering::Equal;}		
		if self.1.is_nan() || other.1.is_nan(){
			panic!("Can't compare NaN here.");
		}
		else { //flipped order to get a min_heap instead
			if other.1 < self.1 { Ordering::Greater }
			else if other.1 > self.1 { Ordering::Less }
			else { Ordering::Equal }
		}
	}
}
impl PartialOrd for MinSortableEdge {
    fn partial_cmp(&self, other: &MinSortableEdge) -> Option<Ordering> {
        Some(self.cmp(other)) //use comparison implemented for Ord
    }
}
impl Eq for MinSortableEdge {}


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
		debug_assert!(line.0 < line.2, "Parameters must be in order.");
		line.0 < obstacle.0 + obstacle.2 && line.2 > obstacle.0  
		&& line.1 > obstacle.1 && line.1 < obstacle.1 + obstacle.3 
	}
	
	// line: (x, y, y2) where y < y2
	// obstacle: (x, y, w, h)
	fn v_line_crosses_obstacle (line:(f64, f64, f64), obstacle: (f64, f64, f64, f64) ) -> bool {
		debug_assert!(line.1 < line.2, "Parameters must be in order.");
		line.1 < obstacle.1 + obstacle.3 && line.2 > obstacle.1  
		&& line.0 > obstacle.0 && line.0 < obstacle.0 + obstacle.2 
	}

impl JkmShortestPathMap {

	/// Create graph with a starting point, a destination and a rectangular map.
	/// The graph will contain between  6 and 6 vertices after the creation, allconnected in a grid
	
	/// At the moment this module does not support the statrting and ending point to be on border of the map, unless they are vertically or horizontally connected which is handeld seperatly
	
	/// #Panics 
	/// Panics when the start point is identical with the end point or if they are not both within the map.
	
	pub fn new (start: (f64, f64), end: (f64, f64), map: (f64,f64,f64,f64)) -> JkmShortestPathMap {
		// TODO: Create map, then pseudo insert obstacle
		
		let mut g = Vec::new();
		g.push(Box::new(GraphNode::new(map.0, map.1)));
		g.push(Box::new(GraphNode::new(map.0 + map.2, map.1)));
		g.push(Box::new(GraphNode::new(map.0 + map.2, map.1 + map.3)));
		g.push(Box::new(GraphNode::new(map.0, map.1 + map.3)));
		
		g[0].neighbours[EAST] = Some(1);
		g[1].neighbours[WEST] = Some(0);
		g[1].neighbours[SOUTH] = Some(2);
		g[2].neighbours[NORTH] = Some(1);
		g[2].neighbours[WEST] = Some(3);
		g[3].neighbours[EAST] = Some(2);
		g[3].neighbours[NORTH] = Some(0);
		g[0].neighbours[SOUTH] = Some(3);
		
		let mut obj = JkmShortestPathMap {
			graph: g,
			obstacles: Vec::new(),
			end_point_index: 0,
			start_point_index: 0, 
			map: map, 
		};
	
		if start.1 == map.1 && end.1 == map.1+map.3 && end.0 == start.0{
				let si = obj.split_edge(0, EAST, start.0);
				let ei = obj.split_edge(3, EAST, start.0);
				obj.graph[si].cost = end.1 - start.1;
				obj.graph[si].neighbours[SOUTH] = Some(ei);
				obj.graph[ei].neighbours[NORTH] = Some(si);
				obj.start_point_index = si;
				obj.end_point_index = ei;
				obj.invalidate_paths_through_node(0);
				obj.graph[si].shortest_path = Some(SOUTH);
				obj.graph[ei].cost = 0.0;
				obj.graph[ei].shortest_path = None;
				obj.update_neighbours(ei);
		}
		else if start.0 == map.0 && end.0 == map.0+map.2 && end.1 == start.1{
				let si = obj.split_edge(0, SOUTH, start.1);
				let ei = obj.split_edge(1, SOUTH, start.1);
				obj.graph[si].cost = end.0 - start.0;
				obj.graph[si].neighbours[EAST] = Some(ei);
				obj.graph[ei].neighbours[WEST] = Some(si);
				obj.start_point_index = si;
				obj.end_point_index = ei;
				obj.invalidate_paths_through_node(0);
				obj.graph[si].shortest_path = Some(EAST);
				obj.graph[ei].cost = 0.0;
				obj.graph[ei].shortest_path = None;
				obj.update_neighbours(ei);
		}
		else if start.0 == end.0 {
			//vertically connected
			let si = obj.graph.len();
			obj.start_point_index = si;
			obj.graph.push(Box::new(GraphNode::new(start.0, start.1)));
			let ei = obj.graph.len();
			obj.end_point_index = ei;
			obj.graph.push(Box::new(GraphNode::new(end.0, end.1)));
			
			if start.1 < end.1 {
				obj.graph[si].cost = end.1 - start.1;
				obj.graph[si].neighbours[SOUTH] = Some(ei);
				obj.graph[ei].neighbours[NORTH] = Some(si);
				obj.graph[si].shortest_path = Some(SOUTH);
				obj.link_to_north(si);		
				obj.link_to_south(ei);
			}
			else if end.1 < start.1 {
				obj.graph[si].cost = start.1 - end.1;
				obj.graph[si].neighbours[NORTH] = Some(ei);
				obj.graph[ei].neighbours[SOUTH] = Some(si);
				obj.graph[si].shortest_path = Some(NORTH);
				obj.link_to_south(si);
				obj.link_to_north(ei);
				
			}
			else { panic!("Start and end point identical coordinates."); }
			
			obj.link_to_east(si);
			obj.link_to_west(si);
			obj.link_to_west(ei);
			obj.link_to_east(ei);
		}
		else if start.1 == end.1 {
			//horizontally connected
			let si = obj.graph.len();
			obj.start_point_index = si;
			obj.graph.push(Box::new(GraphNode::new(start.0, start.1)));
			let ei = obj.graph.len();
			obj.end_point_index = ei;
			obj.graph.push(Box::new(GraphNode::new(end.0, end.1)));
			
			if start.0 < end.0 {
				obj.graph[si].cost = end.0 - start.0;
				obj.graph[si].neighbours[EAST] = Some(obj.end_point_index);
				obj.graph[ei].neighbours[WEST] = Some(obj.start_point_index);
				obj.graph[si].shortest_path = Some(EAST);
				obj.link_to_west(si);
				obj.link_to_east(ei);
			}
			else if end.0 < start.0 {
				obj.graph[si].cost = start.0 - end.0;
				obj.graph[si].neighbours[WEST] = Some(ei);
				obj.graph[ei].neighbours[EAST] = Some(si);
				obj.graph[si].shortest_path = Some(WEST);	
				obj.link_to_east(si);
				obj.link_to_west(ei);				
			}
			else { panic!("Start and end point identical coordinates."); }
			obj.link_to_south(si);
			obj.link_to_south(ei);
			obj.link_to_north(si);
			obj.link_to_north(ei);
		}
		else {
			//not directly connected => create obstacle that connects itself with the border of the map
			let x; let y; let w; let h;
			if start.0 < end.0 {x = start.0; w = end.0-start.0;} else {x=end.0; w = start.0-end.0;}
			if start.1 < end.1 {y = start.1; h = end.1-start.1;} else {y=end.1; h = start.1-end.1;}
			obj.insert_obstacle(x,y,w,h);
			obj.obstacles = Vec::new(); // overwrite pseudo obstacle
			let mut end_index = 0; let mut start_index = 0;
			for (i, node) in obj.graph.iter().enumerate() {
				if node.x == start.0 && node.y == start.1 { start_index = i; }
				if node.x == end.0 && node.y == end.1 { end_index = i;}
			}
			obj.start_point_index = start_index;
			obj.end_point_index = end_index;
			obj.invalidate_paths_through_node(0);
			obj.graph[end_index].cost = 0.0;
			obj.graph[end_index].shortest_path = None;
			obj.update_neighbours(end_index);
			
		}
		obj
	}
	
	///
	pub fn insert_obstacle (&mut self, x: f64, y: f64, w: f64, h: f64) {
		//add obstacle to list
		self.obstacles.push((x,y,w,h));
		
		// Find all edges going thorugh the new obstacle
		let mut h_blocked: BinaryHeap<MinSortableEdge> = BinaryHeap::new();
		let mut v_blocked: BinaryHeap<MinSortableEdge> = BinaryHeap::new();
		
		for v0 in 0..self.graph.len() {
			// only look at one direction of the edges, therefore only look at up and right
			if let Some(up) = self.graph[v0].neighbours[NORTH] {
				if v_line_crosses_obstacle( (self.graph[v0].x, self.graph[up].y, self.graph[v0].y), (x,y,w,h) )
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
		// Remove all blocked edges 
		//invalidate paths going through these edges
		for blocked_edge in h_blocked.iter() {
			let (left, right) = blocked_edge.0;
			self.graph[left].delete_neighbour(right);
			self.graph[right].delete_neighbour(left);
			if self.graph[left].shortest_path == Some(EAST) {
				self.invalidate_paths_through_node(left);
				self.graph[left].shortest_path = None;
				self.graph[left].cost = std::f64::INFINITY;
			}
			if self.graph[right].shortest_path == Some(WEST) {
					self.invalidate_paths_through_node(right);
					self.graph[right].shortest_path = None;
					self.graph[right].cost = std::f64::INFINITY;
			}
		}
		for blocked_edge in v_blocked.iter() {
			let (bot, top) = blocked_edge.0;
			self.graph[bot].delete_neighbour(top);
			self.graph[top].delete_neighbour(bot);
			if self.graph[bot].shortest_path == Some(NORTH) {
				self.invalidate_paths_through_node(bot);
				self.graph[bot].shortest_path = None;
				self.graph[bot].cost = std::f64::INFINITY;
			}
			if self.graph[top].shortest_path == Some(SOUTH) {
				self.invalidate_paths_through_node(top);
				self.graph[top].shortest_path = None;
				self.graph[top].cost = std::f64::INFINITY;
			}
		}
		
		
		// Create new vertices and connect them in a circle
		// While doing so, also make a connection to the rest of the graph:
			// From each edge-node to the node from which an edge was deleted
			// For the corners, search in both open directions the closest edge that could be crossed,
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
		if ho == 0 {lb.neighbours[NORTH] = Some(i);}
		else {lb.neighbours[NORTH] = Some(i+4+2*v+ho);}
		
		debug_assert!(self.graph.len() == i);
		self.graph.push(lu); //i
		
		
		// Add upper line and connect it as described above, 
		// then refresh shortest path on both nodes to a temporary value.
		// The value need to be refreshed again afterwards, but to where the updateing should be initiatet
		// we need these values
		let mut v_blocked_buf = Vec::new(); // use to buffer reversed order
		for j in 0..v {
			if let Some(edge) = v_blocked.pop() {
				let (_, top) = edge.0;
				let cross_x:f64 = edge.1;
				let mut new_node = Box::new(GraphNode::new(cross_x, y));
				new_node.neighbours[NORTH] = Some(top);
				self.graph[top].neighbours[SOUTH] = Some(i + j + 1);
				new_node.neighbours[EAST] = Some(i+j+2);
				new_node.neighbours[WEST] = Some(i+j);
				self.graph.push(new_node);
				v_blocked_buf.push(edge);
				self.update_node(top);
				self.update_node(i + j + 1);
			}
			else {panic!("Something in the graph went wrong. Number of vertices expected: {}, number found: {}\n", v, j);}
		}
		
		debug_assert!(self.graph.len() == i + v + 1);
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
				self.update_node(right);
				self.update_node(i + v + j + 2);
			}
			else {panic!("Something in the graph went wrong. Number of vertices expected: {}, number found: {}\n", ho, j);}
	
		}
		
		debug_assert!(self.graph.len() == i+ v + ho +2);
		self.graph.push(rb); //i+ v + ho +2
		
		// bottom line
		for j in 0..v {
			if let Some(edge) = v_blocked_buf.pop(){
				let (bot, _) = edge.0;
				let cross_x:f64 = edge.1;
				let mut new_node = Box::new(GraphNode::new(cross_x, y+h));
				new_node.neighbours[SOUTH] = Some(bot);
				self.graph[bot].neighbours[NORTH] = Some(i + v + ho + j + 3);
				new_node.neighbours[EAST] = Some(i + v + ho + j + 2);
				new_node.neighbours[WEST] = Some(i + v + ho + j + 4);
				self.graph.push(new_node);
				self.update_node(bot);
				self.update_node(i + v + ho + j + 3);
			}
			else {panic!("Something in the graph went wrong. Number of vertices expected: {}, number found: {}\n", v, j);}

		}
		
		debug_assert!(self.graph.len() == i + 2*v + ho + 3);
		self.graph.push(lb); //i + 2v + ho + 3
		
		if ho > 0 {
		// left line
			for j in 0..(ho-1) {
				if let Some(edge) = h_blocked_buf.pop(){
					let (left, _) = edge.0;
					let cross_y:f64 = edge.1;
					let mut new_node = Box::new(GraphNode::new(x, cross_y));
					new_node.neighbours[WEST] = Some(left);
					self.graph[left].neighbours[EAST] = Some(i + 2 * v + ho + j + 4);
					new_node.neighbours[SOUTH] = Some(i + 2 * v + ho + j + 3);
					new_node.neighbours[NORTH] = Some(i + 2 * v + ho + j + 5);
					self.graph.push(new_node);
					self.update_node(left);
					self.update_node(i + 2 * v + ho + j + 4);
				}
				else {panic!("Something in the graph went wrong. Number of vertices expected: {},  found: {}\n", ho, j);}

			}
			if let Some(edge) = h_blocked_buf.pop(){
				let (left, _) = edge.0;
				let cross_y = edge.1;
				let mut new_node = Box::new(GraphNode::new(x, cross_y));
				new_node.neighbours[WEST] = Some(left);
				self.graph[left].neighbours[EAST] = Some(i + 2 * v + 2 * ho + 3);
				new_node.neighbours[NORTH] = Some(i);
				new_node.neighbours[SOUTH] = Some(i + 2 * v + 2 * ho + 2);
				self.graph.push(new_node);
				self.update_node(left);
				self.update_node(i + 2 * v + 2 * ho + 3);
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
		
		// Search the node closest to the end point, then inititate recomputation starting from this node
		let mut closest_node = (None, std::f64::INFINITY);
		for j in i..self.graph.len() {
			if self.graph[j].cost < closest_node.1 {
				closest_node = (Some(j), self.graph[j].cost);
			}
		}
		if let (Some(closest),_) = closest_node {
			//self.update_node(closest);
			//self.update_neighbours(closest); //This alone does not work, since it will not always update all nodes
			for k in 0..(2*v + 2*ho + 4) {
				let mut to_update = if k%2 == 0 {closest + ((k+1)/2)} else {closest - ((k+1)/2)};
				loop {
					to_update = if to_update >= (i + 2 * v + 2 * ho + 4) { to_update-(2 * v + 2 * ho + 4) }
									else if to_update < i { to_update + (2 * v + 2 * ho + 4) }
									else {break};
				}
				debug_assert!(to_update <(i+ 2 * v + 2 * ho + 4) && to_update >= i, "Boundry violated: to_update: {}, i:{}, v:{}, ho:{}", to_update, i, v, ho);
				self.update_node(to_update);
				self.update_neighbours(to_update);
			}
			// Update new connection nodes
			for k in  (2*v + 2*ho + 4)..self.graph.len() {
				self.update_node(k);
				self.update_neighbours(k);
			}
			
		}//else: not connected at all => no update possible
		
		// At this point, the graph should be consistent again
	}
	
	fn invalidate_paths_through_node (&mut self, n: usize) {	
		for direction in 0..4 {
			if let Some(i) = self.graph[n].neighbours[direction] {
				if let Some(sp_of_neighbour) = self.graph[i].shortest_path {
					if self.graph[i].neighbours[sp_of_neighbour] == Some(n) {
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
		true
	}
	
	// line: (x, y, y2)
	// obstacle: (x, y, w, h)

	fn v_line_crosses_no_obstacle (&self, x: f64, y0: f64, y1: f64) -> bool {
		for &o in self.obstacles.iter() {
			if v_line_crosses_obstacle ( (x,y0,y1), o ) { return false; }
		}
		true
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
			let cost = self.graph[bot].y - self.graph[top].y;
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
	
	/// Update a specific node's shortest path by looking at all neighbours that exist
	/// This function can be (and is) called while the node has connection to neighbours that have not been added to the graph yet
	fn update_node (&mut self, n: usize) {
		if n == self.end_point_index {return;}
		let mut result = (None, std::f64::INFINITY); // Old value does not need to be considered, if this way is still available we will find it anyway
		let graph_size = self.graph.len();
		for i in 0..4 {
			if let Some(neighbour) = self.graph[n].neighbours[i] {
				if neighbour < graph_size // check whether the node exists
				{
					let cost = self.graph[neighbour].cost + self.distance_on_map(n, neighbour);
					if cost < result.1 { result = (Some(i), cost); }
				}
			}
			
		}
		self.graph[n].shortest_path = result.0;
		self.graph[n].cost = result.1;
	}
	
	fn distance_on_map(&self, n0: usize, n1: usize) -> f64 {
		(self.graph[n0].x - self.graph[n1].x).abs() + (self.graph[n0].y - self.graph[n1].y).abs() 
	}
	
	// Take an edge and insert a new node on it. 
	// The coordinate has to be given absolute, not relative to the node n 
	// and can be an x- or y-coordinate, depending on the direction.
	// No new edges are inserted in this function except fot the two who replace the one to split up.
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
		
		// because each node only stores the direction and not the next node on tha shortest path, we don't have to adjust this here for the two consisting nodes
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
	// If the linking was successful and a new node was created, 
	// it will link the new node again in the same direction and so on, 
	// until it cannot link again or it can do so without creating a new node
	// Note: This could be done more efficiently without recursion and a binary heap
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
				self.link_to_north(new_index);
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
				self.link_to_east(new_index);
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
				self.link_to_south(new_index);
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
				self.link_to_west(new_index);
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
			if other.1 > self.1 { Ordering::Greater }
			else if other.1 < self.1 { Ordering::Less }
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


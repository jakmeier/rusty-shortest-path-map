//! This module can maintain all the shortest paths on a two dimensional map
//! Only rectangular objects are allowed and the paths only go vertically or horizontally

#[cfg(test)]
pub mod unit_tests;

use std::cmp::Ordering;
use std::collections::BinaryHeap;

const EPS: f64 = 1.0/1048576.0;

/// Stores a graph with the shortest path from each node to the destination.
/// To recompute this, it also keeps in memory what obstacles there are, therefore, if the actual map changes this struct has to be notified.
/// The map is initially only the border for where obstacles can be placed, to make it blocking call add_map_border() on the shortest path map.
pub struct JkmShortestPathMap {
	graph: Vec<Box<GraphNode>>,
	obstacles: Vec<(f64,f64,f64,f64)>,
	start_point_index: usize,
	end_point_index: usize,
	map: (f64,f64,f64,f64),
	dead_nodes: BinaryHeap<usize>,
	update_root: Vec<usize>,
}

	// line: (x, y, x2)
	// obstacle: (x, y, w, h)
	fn h_line_touches_obstacle (line:(f64, f64, f64), obstacle: (f64, f64, f64, f64) ) -> bool {
		debug_assert!(line.0 <= line.2, "Parameters must be in order. Got {} and {} in this order.", line.0, line.1);
		line.0 < obstacle.0 + obstacle.2 && line.2 > obstacle.0  
		&& line.1 >= obstacle.1 && line.1 <= obstacle.1 + obstacle.3 
	}
	
	// line: (x, y, y2) where y < y2
	// obstacle: (x, y, w, h)
	fn v_line_touches_obstacle (line:(f64, f64, f64), obstacle: (f64, f64, f64, f64) ) -> bool {
		debug_assert!(line.1 <= line.2, "Parameters must be in order. Got {} and {} in this order.", line.1, line.2);
		line.1 < obstacle.1 + obstacle.3 && line.2 > obstacle.1  
		&& line.0 >= obstacle.0 && line.0 <= obstacle.0 + obstacle.2 
	}
	
	// line: (x, y, x2)
	// obstacle: (x, y, w, h)
	fn h_line_overlaps_obstacle (line:(f64, f64, f64), obstacle: (f64, f64, f64, f64) ) -> bool {
		debug_assert!(line.0 <= line.2, "Parameters must be in order. Got {} and {} in this order.", line.0, line.1);
		line.0 < obstacle.0 + obstacle.2 && line.2 > obstacle.0  
		&& line.1 > obstacle.1 && line.1 < obstacle.1 + obstacle.3 
	}
	
	// line: (x, y, y2) where y < y2
	// obstacle: (x, y, w, h)
	fn v_line_overlaps_obstacle (line:(f64, f64, f64), obstacle: (f64, f64, f64, f64) ) -> bool {
		debug_assert!(line.1 <= line.2, "Parameters must be in order. Got {} and {} in this order.", line.1, line.2);
		line.1 < obstacle.1 + obstacle.3 && line.2 > obstacle.1  
		&& line.0 > obstacle.0 && line.0 < obstacle.0 + obstacle.2 
	}

impl JkmShortestPathMap {

	/// Create graph with a starting point, a destination and a rectangular map.
	/// The graph will contain between  6 and 16 vertices after the creation, all connected in a grid
	
	/// At the moment this module does not support the statrting and ending point to be on border of the map, unless they are vertically or horizontally connected which is handeld seperatly
	
	/// #Panics 
	/// Panics when the start point is identical with the end point or if they are not both within the map.
	
	pub fn new (start: (f64, f64), end: (f64, f64), map: (f64,f64,f64,f64)) -> JkmShortestPathMap {
		
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
			dead_nodes: BinaryHeap::new(),
			update_root: Vec::new(), 
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
	
	/// Reads out the coordinates of the end_point_index
	pub fn get_destination_coordinates(&self) -> (f64, f64) {
		(self.graph[self.end_point_index].x, self.graph[self.end_point_index].y)
	}
	
	/// Adds a rectangular obstacle to the map and changes the graph's nodes and shortest paths accordingly. 
	/// Note that two obstacles that are exactly aligned will not block the way between them. There must be 
	/// an overlapping to disable paths between obstacles.
	pub fn insert_obstacle (&mut self, x: f64, y: f64, w: f64, h: f64) {
		//add obstacle to list
		self.obstacles.push((x,y,w,h));
		
		// check map boundaries:
		if x > self.map.0 + self.map.2 || y > self.map.1 + self.map.3 
			{ return; }
		let w = if x + w > self.map.0 + self.map.2 { self.map.0 + self.map.2 - x + 4.0 * EPS} else { w };
		let h = if y + h > self.map.1 + self.map.3 { self.map.1 + self.map.3  - y + 4.0 * EPS} else { h };
		
		// Find all edges going thorugh the new obstacle
		let mut h_blocked: BinaryHeap<MinSortableEdge> = BinaryHeap::new();
		let mut v_blocked: BinaryHeap<MinSortableEdge> = BinaryHeap::new();
		let mut nodes_to_erase = Vec::new();
		
		for v0 in 0..self.graph.len() {
			// only look at one direction of the edges, therefore only look up and right
			if let Some(up) = self.graph[v0].neighbours[NORTH] {
				if v_line_touches_obstacle( (self.graph[v0].x, self.graph[up].y, self.graph[v0].y), (x,y,w,h) )
				{
					if v_line_overlaps_obstacle( (self.graph[v0].x, self.graph[up].y, self.graph[v0].y), (x,y,w,h) )
					{
						v_blocked.push(MinSortableEdge((v0,up), self.graph[v0].x));
					}
					else { // line goes parallel to an obstacle and touches it
							//remove the edge and the nodes which become unnecessary because of the new obstacle
							// all edges around the boredrs are inserted later on
						self.graph[v0].delete_neighbour(up);
						self.graph[up].delete_neighbour(v0);
						if self.graph[v0].shortest_path == Some(NORTH) {
							self.invalidate_paths_through_node(v0);
						}
						if self.graph[up].shortest_path == Some(SOUTH) {
							self.invalidate_paths_through_node(up);
						}
						if self.graph[v0].y <= y + h { nodes_to_erase.push(v0); }
						if self.graph[up].y >= y { nodes_to_erase.push(up); }
					}
				}	
			}
			if let Some(right) = self.graph[v0].neighbours[EAST] {
				if h_line_touches_obstacle( (self.graph[v0].x, self.graph[v0].y, self.graph[right].x), (x,y,w,h) )
				{
					if h_line_overlaps_obstacle( (self.graph[v0].x, self.graph[v0].y, self.graph[right].x), (x,y,w,h) ) {
						h_blocked.push(MinSortableEdge((v0,right), self.graph[v0].y));
					}
					else {
					//remove the edge and the nodes which become unnecessary because of the new obstacle
						self.graph[v0].delete_neighbour(right);
						self.graph[right].delete_neighbour(v0);
						if self.graph[v0].shortest_path == Some(EAST) {
							self.invalidate_paths_through_node(v0);
						}
						if self.graph[right].shortest_path == Some(WEST) {
							self.invalidate_paths_through_node(right);
						}
						if self.graph[v0].x >= x { nodes_to_erase.push(v0); }
						if self.graph[right].x <= x + w { nodes_to_erase.push(right); }
					}
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
			}
			if self.graph[right].shortest_path == Some(WEST) {
					self.invalidate_paths_through_node(right);
			}
		}
		for blocked_edge in v_blocked.iter() {
			let (bot, top) = blocked_edge.0;
			self.graph[bot].delete_neighbour(top);
			self.graph[top].delete_neighbour(bot);
			if self.graph[bot].shortest_path == Some(NORTH) {
				self.invalidate_paths_through_node(bot);
			}
			if self.graph[top].shortest_path == Some(SOUTH) {
				self.invalidate_paths_through_node(top);
			}
		}
		
		// Erase scheduled nodes
		for &node in nodes_to_erase.iter() {
			self.erase_node(node);
		}
		
		// Create new vertices and connect them in a circle
		// While doing so, also make a connection to the rest of the graph:
			// From each edge-node to the node from which an edge was deleted
			// For the corners, search in both open directions the closest edge that could be crossed,
			// create a new node there and connect to this.
		let lu = Box::new(GraphNode::new(x,y));
		let mut ru = Box::new(GraphNode::new(x+w,y));
		let mut rb = Box::new(GraphNode::new(x+w,y+h));
		let mut lb = Box::new(GraphNode::new(x,y+h));
		
		let i = self.graph.len();
		let v = v_blocked.len();
		let ho = h_blocked.len();
		
		// Add upper line and connect it as described above, 
		// then refresh shortest path on both nodes to a temporary value.
		// The value need to be refreshed again afterwards, but to determine where the updating should be initiatet
		// we need these values
		
		let mut v_blocked_buf = Vec::new(); // use to buffer reversed ord
		let mut h_blocked_buf = Vec::new(); 
		
		let mut predecessor : Option<usize>; 
		
		//left upper corner
		let lu_exists;
		if !self.coordinate_is_blocked(lu.x, lu.y) {
			self.graph.push(lu);
			self.link_to_north(i);
			self.link_to_west(i);
			predecessor = Some(i);
			lu_exists = true;
		} else {predecessor = None; lu_exists=false;}
		
		// upper line
		for j in 0..v {
			let index = self.graph.len();
			if let Some(edge) = v_blocked.pop() {
				let (_, top) = edge.0;
				let cross_x:f64 = edge.1;
				let mut new_node = Box::new(GraphNode::new(cross_x, y));
				if !self.coordinate_is_blocked(new_node.x, new_node.y) {
					if self.graph[top].y < y { 		
						new_node.neighbours[NORTH] = Some(top); 
						self.graph[top].neighbours[SOUTH] = Some(index);
						
						if let Some(prev_index) = predecessor{ 
							if 	self.h_line_overlaps_no_obstacle ( self.graph[prev_index].x, self.graph[prev_index].y, new_node.x ) {
								self.graph[prev_index].neighbours[EAST] = Some(index);
								new_node.neighbours[WEST] = Some(prev_index);
							}
						}
						self.graph.push(new_node);
						self.update_neighbours(index);
						predecessor = Some(index);
					}
					//else keep predecessor info
				} else {predecessor = None;}
				v_blocked_buf.push(edge);
			}
			else {panic!("Something in the graph went wrong. Number of vertices expected: {}, number found: {}\n", v, j);}
		}
		
		// right upper corner
		let index = self.graph.len();
		if !self.coordinate_is_blocked(ru.x, ru.y) {
			if let Some(prev_index) = predecessor { 
				if 	self.h_line_overlaps_no_obstacle ( self.graph[prev_index].x, self.graph[prev_index].y, ru.x ) {
					ru.neighbours[WEST] = Some(prev_index); 
					self.graph[prev_index].neighbours[EAST] = Some(index);
				}
			}
			self.graph.push(ru);
			self.link_to_north(index);
			self.link_to_east(index);
			predecessor = Some(index);
		} else {predecessor = None;}
		
		// right line	
		for j in 0..ho {
		    let index = self.graph.len();
			if let Some(edge) = h_blocked.pop(){
				let (_, right) = edge.0;
				let cross_y:f64 = edge.1;
				let mut new_node = Box::new(GraphNode::new(x+w, cross_y));			
				if !self.coordinate_is_blocked(new_node.x, new_node.y) {
					if self.graph[right].x > x + w {
						new_node.neighbours[EAST] = Some(right);
						self.graph[right].neighbours[WEST] = Some(index );
					
						if let Some(prev_index) = predecessor { 
							if 	self.v_line_overlaps_no_obstacle ( self.graph[prev_index].x, self.graph[prev_index].y, new_node.y ) {
								new_node.neighbours[NORTH] = Some(prev_index); 
								self.graph[prev_index].neighbours[SOUTH] = Some(index);
							}
						}	
						self.graph.push(new_node);
						self.update_neighbours(index);
						predecessor = Some(index);
					}
				} else {predecessor = None;}
				h_blocked_buf.push(edge);
			}
			else {panic!("Something in the graph went wrong. Number of vertices expected: {}, number found: {}\n", ho, j);}
	
		}
		
		let index = self.graph.len();
		
		if !self.coordinate_is_blocked(rb.x, rb.y) {
			if let Some(prev_index) = predecessor { 
				if 	self.v_line_overlaps_no_obstacle ( self.graph[prev_index].x, self.graph[prev_index].y, rb.y ) {
					rb.neighbours[NORTH] = Some(prev_index); 
					self.graph[prev_index].neighbours[SOUTH] = Some(index);
				}
			}
			self.graph.push(rb);
			self.link_to_east(index);
			self.link_to_south(index);
			predecessor = Some(index);
		} else {predecessor = None;}
		
		// bottom line
		
		for j in 0..v {
			let index = self.graph.len();
			if let Some(edge) = v_blocked_buf.pop(){
				let (bot, _) = edge.0;
				//at this point, bot could be erased, therefore cleanup() is needed
				let cross_x:f64 = edge.1;
				let mut new_node = Box::new(GraphNode::new(cross_x, y+h));		
				if !self.coordinate_is_blocked(new_node.x, new_node.y) {
					if self.graph[bot].y > y + h {
						new_node.neighbours[SOUTH] = Some(bot);
						self.graph[bot].neighbours[NORTH] = Some(index);
					
						if let Some(prev_index) = predecessor { 
							if 	self.h_line_overlaps_no_obstacle ( new_node.x, self.graph[prev_index].y, self.graph[prev_index].x ) {
								new_node.neighbours[EAST] = Some(prev_index);
								self.graph[prev_index].neighbours[WEST] = Some(index);
							}
						}
						self.graph.push(new_node);
						self.update_neighbours(index);
						predecessor = Some(index);
					}
				} else {predecessor = None;}
			}
			else {panic!("Something in the graph went wrong. Number of vertices expected: {}, number found: {}\n", v, j);}

		}
		
		let index = self.graph.len();
		
		if !self.coordinate_is_blocked(lb.x, lb.y) {
			if let Some(prev_index) = predecessor { 
				if 	self.h_line_overlaps_no_obstacle ( lb.x, self.graph[prev_index].y, self.graph[prev_index].x ) {
					lb.neighbours[EAST] = Some(prev_index); 
					self.graph[prev_index].neighbours[WEST] = Some(index);
				}
			}
			self.graph.push(lb);
			self.link_to_south(index);
			self.link_to_west(index);
			predecessor = Some(index);
		} else {predecessor = None;}
		
		// left line	
		for j in 0..ho {
			let index = self.graph.len();
			if let Some(edge) = h_blocked_buf.pop(){
				let (left, _) = edge.0;
				let cross_y:f64 = edge.1;
				let mut new_node = Box::new(GraphNode::new(x, cross_y));	
				if !self.coordinate_is_blocked(new_node.x, new_node.y) {
					if self.graph[left].x < x {
						new_node.neighbours[WEST] = Some(left);
						self.graph[left].neighbours[EAST] = Some(index);
					
						if let Some(prev_index) = predecessor { 
							debug_assert!(new_node.x == self.graph[prev_index].x, "Nodes that should be connected vertically are not alligned. Nodes: a new node at x={} and Node#{} at x={}", new_node.x, prev_index, self.graph[prev_index].x) ;
							if 	self.v_line_overlaps_no_obstacle ( self.graph[prev_index].x, new_node.y, self.graph[prev_index].y ) {
								new_node.neighbours[SOUTH] = Some(prev_index);
								self.graph[prev_index].neighbours[NORTH] = Some(index);
							}
						}
						self.graph.push(new_node);
						self.update_neighbours(index);
						predecessor = Some(index);
					}
				} else {predecessor = None;}
			}
			else {panic!("Something in the graph went wrong. Number of vertices expected: {},  found: {}\n", ho, j);}
		}
		
		let index = self.graph.len();
		if index > i && lu_exists {
			if let Some(prev_index) = predecessor {
				debug_assert!(self.graph[i].x == self.graph[prev_index].x, "Nodes that should be connected vertically are not alligned. Nodes: #{} at {} and #{} at {}", i, self.graph[i].x , prev_index, self.graph[prev_index].x);
				if self.v_line_overlaps_no_obstacle ( self.graph[i].x, self.graph[i].y, self.graph[prev_index].y ) {		
					self.graph[prev_index].neighbours[NORTH] = Some(i);
					self.graph[i].neighbours[SOUTH] = Some(prev_index);			
				}
			}
		}
		
		
		self.cleanup();
		self.update();
		self.erase_lonely_nodes();
		
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
			let added_nodes = index - i;
			for k in 0..(added_nodes) {
				let mut to_update = if k%2 == 0 {closest + ((k+1)/2)} else {closest - ((k+1)/2)};
				loop {
					to_update = if to_update >= (i + added_nodes) { to_update-added_nodes }
									else if to_update < i { to_update + added_nodes }
									else {break};
				}
				debug_assert!(to_update <self.graph.len() && to_update >= i, "Boundry violated: to_update: {}, i:{}, v:{}, ho:{}", to_update, i, v, ho);
				self.update_node(to_update);
				self.update_neighbours(to_update);
			}	
		}//else: not connected at all => no update possible	
		
		// At this point, the graph should be consistent again
	
		// This call is only here to shrink the graph size and get rid of unecessary nodes
		// It is best called in the end because the recomputtation takes advantage of the fact that all 
		//  new nodes have been appended to the end of the vector. Calling this function may destroy that order.
		self.swap_out_dead_nodes();
	}
	
	/// Removes an obstacle that was instered earlier. 
	/// This function will do noting if there was no such obstacle inserted or if it was alread removed.
	pub fn remove_obstacle (&mut self, x: f64, y: f64, w: f64, h: f64) {
		let mut obstacle_index = None;
		for (i, &(ox, oy, ow, oh)) in self.obstacles.iter().enumerate() {
			if ox == x && oy == y && oh == h && ow == w {
				obstacle_index = Some(i);
				break;
			}
		}
		if let Some(i) = obstacle_index {
			self.obstacles.swap_remove(i);
			for i in 0..self.graph.len() {
				if self.graph[i].x > x && self.graph[i].x < x+w {
					// upper line
					if self.graph[i].y == y && !self.graph[i].neighbours[SOUTH].is_some() {
						self.link_to_south(i);
						self.update_node(i);
						self.update_neighbours(i);
					}
					// lower line
					if self.graph[i].y == y + h && !self.graph[i].neighbours[NORTH].is_some() {
						self.link_to_north(i);
						self.update_node(i);
						self.update_neighbours(i);
					}
				}
				if self.graph[i].y > y && self.graph[i].y < y+h {
					// left line
					if self.graph[i].x == x && !self.graph[i].neighbours[EAST].is_some() {
						self.link_to_east(i);
						self.update_node(i);
						self.update_neighbours(i);
					}
					// right line
					if self.graph[i].x == x + w && !self.graph[i].neighbours[WEST].is_some() {
						self.link_to_west(i);
						self.update_node(i);
						self.update_neighbours(i);
					}
				}
			}
		}
	}
	
	/// Makes the border of the map blocking, i.e. no paths can go through it
	pub fn add_map_border(&mut self) {
		let d = 0.0625;
		let (x,y,w,h) = self.map;
		self.obstacles.push( (x-d, y, d, h) );
		self.obstacles.push( (x+w, y, d, h) );
		self.obstacles.push( (x, y-d, w, d) );
		self.obstacles.push( (x, y+h, w, d) );
	}
	
	/// Returns the nearest checkpoint on the shortest path from the given coordinate to the destination. 
	/// This function will not check whether the given coordinate is on a node of the graph, therefore it 
	/// has to search through all edges in the graph.
	/// If the coordinate is most likely already on a node, call next_checkpoint() instead.
	/// Returns None if there is no path to the destination. 
	///	If the destination is already reached, its coordinates are returned.
	pub fn nearest_checkpoint(&self, x: f64, y: f64) -> Option<(f64,f64)> {
		let destination = self.end_point_index;
		if self.graph[destination].x == x && self.graph[destination].y == y {
			return Some((x,y));
		}
		
		let mut nearest = (None, std::f64::INFINITY, std::f64::INFINITY);
		
		for node in self.graph.iter() {
			if let Some(right_index) = node.neighbours[EAST] {
				let right = &self.graph[right_index];
				if node.x <= x && right.x >= x {
					let new_y = node.y;
					let total_cost;
					let cost_to_edge = (y-new_y).abs();
					let cost_on_edge = if (node.cost + x - node.x ) < (right.cost + right.x - x)
										    {total_cost = node.cost + x - node.x + cost_to_edge; x - node.x}
									   else {total_cost = right.cost + right.x - x + cost_to_edge; right.x - x};
					
					if total_cost <= nearest.1 && cost_on_edge + cost_to_edge > EPS 
						&& (total_cost < nearest.1 
							|| (cost_on_edge + cost_to_edge < nearest.2 && total_cost < std::f64::INFINITY )
						) {
						if (y < new_y && self.v_line_overlaps_no_obstacle(x, y, new_y ))
							||( y > new_y && self.v_line_overlaps_no_obstacle(x, new_y, y )){
							nearest = (Some((x,new_y)), total_cost, cost_on_edge + cost_to_edge);
						}		
					} 	
				}				
			}
			if let Some(bot_index) = node.neighbours[SOUTH] {
				let bot = &self.graph[bot_index];
				if node.y <= y && bot.y >= y {
					let new_x = node.x;
					let total_cost;
					let cost_to_edge = (x-new_x).abs();
					let cost_on_edge = if (node.cost + y - node.y ) < (bot.cost + bot.y - y)
										    {total_cost = node.cost + y - node.y + cost_to_edge; y - node.y}
									   else {total_cost = bot.cost + bot.y - y + cost_to_edge; bot.y - y};
					if total_cost <= nearest.1  && cost_on_edge + cost_to_edge > EPS 
						&& (total_cost < nearest.1 
							 || (cost_on_edge + cost_to_edge < nearest.2 && total_cost < std::f64::INFINITY  )
							) {
						if (x < new_x && self.h_line_overlaps_no_obstacle(x, y, new_x ))
							|| (x > new_x && self.h_line_overlaps_no_obstacle(new_x, y, x )){
							nearest = (Some((new_x,y)), total_cost, cost_on_edge + cost_to_edge);
						}		
					} 	
				}				
			}
		}
		nearest.0
	}
	
	/// Returns the next checkpoint on the shortest path from the given coordinate to the destination, 
	///  assuming that the given coordinate is a checkpoint given earlier. In case it is not, it will still
	///  find the correct result, however it is inefficient to use this function then.
	/// If the coordinate is most likely between nodes, call nearest_checkpoint() instead.
	/// Returns None if there is no path to the destination. 
	///	If the destination is already reached, its coordinates are returned.
	pub fn next_checkpoint(&self, x: f64, y: f64) -> Option<(f64,f64)> {

		let destination = self.end_point_index;
		if self.graph[destination].x == x && self.graph[destination].y == y {
			return Some((x,y));
		}
		
		for node in self.graph.iter() {
			if (node.x - x).abs() < EPS && (node.y - y).abs() < EPS {
				if let Some(sp) = node.shortest_path {
					if let Some(neighbour) = node.neighbours[sp] {
						return Some( (self.graph[neighbour].x, self.graph[neighbour].y) );
					}
				}
				break;
			}
		}
		//println!("No current node found.");
		self.nearest_checkpoint(x,y)
	}
	
	// Checks recursivly on neighbours wether their shortest path goes through the given node.
	// All these paths are invalidated, however no edges are deleted.
	fn invalidate_paths_through_node (&mut self, n: usize) {	
		for direction in 0..4 {
			if let Some(i) = self.graph[n].neighbours[direction] {
				if let Some(sp_of_neighbour) = self.graph[i].shortest_path {
					if self.graph[i].neighbours[sp_of_neighbour] == Some(n) {
						self.invalidate_paths_through_node (i);
					}
					else {
						self.consider_node_as_update_root(i);
					}
				}
			}
		}
		self.graph[n].shortest_path = None;
		self.graph[n].cost = std::f64::INFINITY;
	}
	
	fn consider_node_as_update_root(&mut self, n: usize) {
		self.update_root.push(n);
	}
	
	fn update(&mut self) {
		for i in 0..self.update_root.len() {
			let entry_point = self.update_root[i];
			self.update_neighbours(entry_point);
		}
		self.update_root = Vec::new();
		
		
	}
	
	// line: (x, y, x2)
	// obstacle: (x, y, w, h)
	fn h_line_overlaps_no_obstacle (&self, x0: f64, y: f64, x1: f64) -> bool {
		for &o in self.obstacles.iter() {
			if h_line_overlaps_obstacle ( (x0,y,x1), o ) { return false; }
		}
		true
	}
	
	// line: (x, y, y2)
	// obstacle: (x, y, w, h)
	fn v_line_overlaps_no_obstacle (&self, x: f64, y0: f64, y1: f64) -> bool {
		for &o in self.obstacles.iter() {
			if v_line_overlaps_obstacle ( (x,y0,y1), o ) { return false; }
		}
		true
	}
	
	fn coordinate_is_blocked(&mut self, x: f64, y: f64) -> bool {
		for obs in self.obstacles.iter() {
			if obs.0 < x && obs.0 + obs.2 > x
				&& obs.1 < y && obs.1 + obs.3 > y
				{ return true; }
		}
		false
	}
	
	/// Connects two nodes horizontally if possible and updates shortest paths that are changed by this new edge
	fn connect_h(&mut self, left: usize, right: usize) -> bool{
		debug_assert!(self.graph[left].x < self.graph[right].x && self.graph[left].y == self.graph[right].y, "Left is actually not left from right or they are not alligned.");
		
		if self.h_line_overlaps_no_obstacle(self.graph[left].x, self.graph[left].y, self.graph[right].x) {
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
			true
		}
		else {false}
	}
	
	/// Connects two nodes vertically if possible and updates shortest paths that are changed by this new edge
	fn connect_v(&mut self, top: usize, bot: usize) -> bool {
		debug_assert!(self.graph[top].y < self.graph[bot].y && self.graph[top].x == self.graph[bot].x, "The top node is actually not higher than the bottom one or they are not alligned.");
		if self.v_line_overlaps_no_obstacle(self.graph[top].x, self.graph[top].y, self.graph[bot].y) {
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
			true
		}
		else {false}
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
		
		// because each node only stores the direction and not the next node on the shortest path, we don't have to adjust this here for the two consisting nodes
		let other_node = self.graph[n].neighbours[direction].unwrap();
		self.graph.push(Box::new(GraphNode::new(x, y)));
		self.graph[new_index].neighbours[other_direction] = Some(n);
		self.graph[n].neighbours[direction] = Some(new_index);
		self.graph[new_index].neighbours[direction] = Some(other_node);
		self.graph[other_node].neighbours[other_direction] = Some(new_index);
		self.update_node(new_index);
		new_index
	}
	
	// panics if the node cannot be merged
	// a node can be merged if it has exactly two neighbouts which are in the opposite direction
	// Unless n is the last node in the graph, this funciton will produce unused nodes within the graph (no neighbours, coordinate std::f64::NEG_INFINITY|std::f64::NEG_INFINITY)
	fn merge_node(&mut self, n: usize) {
		if let Some(top) = self.graph[n].neighbours[NORTH] {
			if let Some(bot) = self.graph[n].neighbours[SOUTH] {
				debug_assert!(! (self.graph[n].neighbours[EAST].is_some() || self.graph[n].neighbours[WEST].is_some() ));
				self.graph[bot].neighbours[NORTH] = Some(top);
				self.graph[top].neighbours[SOUTH] = Some(bot);
				// no need to change shortest path directions or costs
			}
			else { panic!("Node #{} cannot be merged!", n); }
		}
		else {
			if let Some(left) = self.graph[n].neighbours[WEST] {
				if let Some(right) = self.graph[n].neighbours[EAST] {
					self.graph[right].neighbours[WEST] = Some(left);
					self.graph[left].neighbours[EAST] = Some(right);
				}
				else { panic!("Node #{} cannot be merged!", n); }
			}
			else { panic!("Node #{} cannot be merged!", n);}
		}
		// remove node
		self.graph[n].neighbours = [None, None, None, None];
		
		self.erase_node(n);
		self.cleanup();
	}
	
	// Detatches the node from the graph and moves it to std::f64::NEG_INFINITY | std::f64::NEG_INFINITY
	// The node can't be deleted since that would change the index of other nodes
	// TODO: Make the node slots available for new nodes
	fn erase_node (&mut self, n: usize) {
		self.graph[n].x = std::f64::NEG_INFINITY;
		self.graph[n].y = std::f64::NEG_INFINITY;
		self.invalidate_paths_through_node(n);
		for direction in 0..4 {
			if let Some(neighbour) = self.graph[n].neighbours[direction] {
				let other_direction = (direction + 2)%4;
				//debug_assert!(self.graph[neighbour].neighbours[other_direction] == Some(n));
				self.graph[neighbour].neighbours[other_direction] = None;
			}
		}
		self.graph[n] = Box::new(GraphNode::new(std::f64::NEG_INFINITY, std::f64::NEG_INFINITY));
		self.dead_nodes.push(n);
	}
	
	// This procedure will mark nodes without neighbours as dead
	fn erase_lonely_nodes (&mut self) {
		for i in 0..self.graph.len() {
			if !self.graph[i].neighbours[NORTH].is_some()
				&& !self.graph[i].neighbours[EAST].is_some()
				&& !self.graph[i].neighbours[SOUTH].is_some()
				&& !self.graph[i].neighbours[WEST].is_some() 
			{
				self.graph[i].x = std::f64::NEG_INFINITY;
				self.graph[i].y = std::f64::NEG_INFINITY;
				self.dead_nodes.push(i);
			}
		}
	}
	
	// Takes the last nodes in the graph and swaps them with the dead nodes so those can be erased
	fn swap_out_dead_nodes (&mut self) {
		let mut last = None;
		while let Some(dead_slot) = self.dead_nodes.pop() {
			// avoid deleting nodes more than once
			if let Some(last_node) = last {
				if last_node == dead_slot {continue;}
			}
			last = Some(dead_slot);
			debug_assert!(self.graph[dead_slot].x == std::f64::NEG_INFINITY && self.graph[dead_slot].y == std::f64::NEG_INFINITY , "Node #{} that was listed as dead was alive! It had the coordinates [{}|{}] ! ", dead_slot, self.graph[dead_slot].x, self.graph[dead_slot].y );
			debug_assert!(self.graph[dead_slot].neighbours == [None, None, None, None], "Node #{} that was listed as dead was alive! It had some neighbours! ", dead_slot );
			if dead_slot == self.graph.len() - 1 { self.graph.pop(); }  // Note: since dead_nodes is a Max-Heap, this if condition is either true or the last node is alive
			else {
				if let Some(node) = self.graph.pop() {
					for direction in 0..4 {
						if let Some(neighbour) = node.neighbours[direction] {
							debug_assert!(self.graph[neighbour].neighbours[ (direction + 2) %4 ] == Some(self.graph.len()));
							self.graph[neighbour].neighbours[ (direction + 2) %4 ] = Some(dead_slot);
						}
					}
					self.graph[dead_slot] = node;
				}
				else {
					println!("Unexpected branch in the shortest path map module: No node in the graph, but there is something on the list of dead nodes."); 
					self.dead_nodes.push(dead_slot);
					break;
				}
			}
		}
	}
	
	/*fn store_node (&mut self, node: GraphNode) -> usize {
		if let Some(dead_index) = self.dead_nodes.pop() {
			self.graph[dead_index] = node;
			dead_index
		}
		else {
			self.graph.push(node);
			self.graph.len() - 1
		}
	}*/
	
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
				if self.connect_v(new_index, n) {
					self.link_to_north(new_index);
				}
				else { self.merge_node(new_index); }
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
				if self.connect_h(n, new_index) {
					self.link_to_east(new_index);
				}
				else { self.merge_node(new_index); }
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
				if self.connect_v(n, new_index) {
					self.link_to_south(new_index);
				}
				else { self.merge_node(new_index); }
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
				if self.connect_h(new_index, n) {
					self.link_to_west(new_index);
				}
				else { self.merge_node(new_index); }
			}
			
		}
	}
	
	// Looks through dead nodes and repairs wrongly set neighbourhoods
	fn cleanup(&mut self) {
		let to_consider = self.dead_nodes.clone();
		for &n in to_consider.iter() {
			debug_assert!(self.graph[n].x == std::f64::NEG_INFINITY && self.graph[n].y == std::f64::NEG_INFINITY, "Dead node had a coordinate");
			for direction in 0..4 {
				if let Some(neighbour) = self.graph[n].neighbours[direction] {
					self.graph[n].neighbours[direction] = None;
					self.graph[neighbour].neighbours[(direction+2)%4] = None;
					match direction {
						NORTH => self.reconnect_to_south(neighbour),
						EAST => self.reconnect_to_west(neighbour),
						SOUTH => self.reconnect_to_north(neighbour),
						WEST => self.reconnect_to_east(neighbour),
						_ => panic!()
					}
					self.graph[n].shortest_path = None;
				}
			}
		}
	}
	
	
	// These function search for a perfectly aligned neighbour node to connect, 
	// they will ignore obstacles that are only touched on the border
	fn reconnect_to_north(&mut self, n: usize) {
		let mut closest = (None, std::f64::NEG_INFINITY);
		let x = self.graph[n].x;
		let y = self.graph[n].y;
		for (i, node) in self.graph.iter().enumerate() {
			if node.x == x && node.y < y && node.y > closest.1 {
				closest = (Some(i), node.y);
			}
		}
		if let Some(new_neighbour) = closest.0 {
			if self.v_line_overlaps_no_obstacle(x, self.graph[new_neighbour].y, y) {
				self.graph[new_neighbour].neighbours[SOUTH] = Some(n);
				self.graph[n].neighbours[NORTH] = Some(new_neighbour);
			}
		}
	}
	
	fn reconnect_to_east(&mut self, n: usize) {
		let mut closest = (None, std::f64::INFINITY);
		let x = self.graph[n].x;
		let y = self.graph[n].y;
		for (i, node) in self.graph.iter().enumerate() {
			if node.y == y && node.x > x && node.x < closest.1 {
				closest = (Some(i), node.x);
			}
		}
		if let Some(new_neighbour) = closest.0 {
			if self.h_line_overlaps_no_obstacle(x, y, self.graph[new_neighbour].x) {
				self.graph[new_neighbour].neighbours[WEST] = Some(n);
				self.graph[n].neighbours[EAST] = Some(new_neighbour);
			}
		}
	}
	
	fn reconnect_to_south(&mut self, n: usize) {
	let mut closest = (None, std::f64::INFINITY);
		let x = self.graph[n].x;
		let y = self.graph[n].y;
		for (i, node) in self.graph.iter().enumerate() {
			if node.x == x && node.y > y && node.y < closest.1 {
				closest = (Some(i), node.y);
			}
		}
		if let Some(new_neighbour) = closest.0 {
			if self.v_line_overlaps_no_obstacle(x, y, self.graph[new_neighbour].y) {
				self.graph[new_neighbour].neighbours[NORTH] = Some(n);
				self.graph[n].neighbours[SOUTH] = Some(new_neighbour);
			}
		}
	}
	
	fn reconnect_to_west(&mut self, n: usize) {
		let mut closest = (None, std::f64::NEG_INFINITY);
		let x = self.graph[n].x;
		let y = self.graph[n].y;
		for (i, node) in self.graph.iter().enumerate() {
			if node.y == y && node.x < x && node.x > closest.1 {
				closest = (Some(i), node.x);
			}
		}
		if let Some(new_neighbour) = closest.0 {
			if self.h_line_overlaps_no_obstacle(self.graph[new_neighbour].x, y, x ) {
				self.graph[new_neighbour].neighbours[EAST] = Some(n);
				self.graph[n].neighbours[WEST] = Some(new_neighbour);
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


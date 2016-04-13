//! Holds unit-style tests as well as module invariants that can be called from the integration test module
use std::fs::File;
use std::io::prelude::*;

use super::*;

const NORTH: usize = 0;
const EAST: usize = 1;
const SOUTH: usize = 2;
const WEST: usize = 3;

mod tests;

pub fn check_module_invariants (testee: &JkmShortestPathMap) {
	inv_neighbours_are_symmetric(testee);
	inv_all_shortest_paths_lead_to_destination(testee);	
}
	
	// Call in assertion with ||
pub fn print_graph (testee: &JkmShortestPathMap) -> bool {
	
	if let Ok(mut f) = File::create("log.jkmmap") {
	
		//nodes
		for i in 0..testee.graph.len() {
			let mut node_string = format!("{}|{}|{}|{}|{}|{}|{}|{}", 
			testee.graph[i].x, testee.graph[i].y,
			if let Some(n) = testee.graph[i].neighbours[0] {n.to_string()} else {"-".to_string()},
			if let Some(n) = testee.graph[i].neighbours[1] {n.to_string()} else {"-".to_string()},
			if let Some(n) = testee.graph[i].neighbours[2] {n.to_string()} else {"-".to_string()},
			if let Some(n) = testee.graph[i].neighbours[3] {n.to_string()} else {"-".to_string()},
			if let Some(sp) = testee.graph[i].shortest_path  {sp.to_string()} else {"-".to_string()},
			testee.graph[i].cost
			);
			f.write_all((node_string + "\n").as_bytes());
		}
		//obstacles
		f.write_all("#\n".as_bytes());
		for &(x,y,w,h) in testee.obstacles.iter() {
			f.write_all( (format!("{}|{}|{}|{}", x,y,w,h) + "\n").as_bytes() );
		}
		
	}
	println!("\nJkmShortestPathMap's graph looks like this: \n");
	
	for i in 0..testee.graph.len() {
		println!("Node #{}[{}|{}]: [N:{} E:{}, S: {}, W: {}] | shortest_path: {} cost: {} ", 
		i, testee.graph[i].x, testee.graph[i].y,
		if let Some(n) = testee.graph[i].neighbours[0] {n.to_string()} else {"-".to_string()},
		if let Some(n) = testee.graph[i].neighbours[1] {n.to_string()} else {"-".to_string()},
		if let Some(n) = testee.graph[i].neighbours[2] {n.to_string()} else {"-".to_string()},
		if let Some(n) = testee.graph[i].neighbours[3] {n.to_string()} else {"-".to_string()},
		if let Some(sp) = testee.graph[i].shortest_path  {sp.to_string()} else {"-".to_string()},
		testee.graph[i].cost
		);
	}
	println!(" ");
	false
}

	
fn inv_neighbours_are_symmetric(testee: &JkmShortestPathMap) {
	for (i, node) in testee.graph.iter().enumerate() {
		if let Some(neighbour) = node.neighbours[NORTH] {
			assert!(testee.graph[neighbour].neighbours[SOUTH] == Some(i)|| print_graph(testee), "\nNode #{} has a neighbour in the north, Node #{}, but the neighbourhood is not returned.\n", i, neighbour );
		}
		if let Some(neighbour) = node.neighbours[EAST] {
			assert!(testee.graph[neighbour].neighbours[WEST] == Some(i)|| print_graph(testee), "\nNode #{} has a neighbour in the east, Node #{}, but the neighbourhood is not returned.\n", i, neighbour  );
		}
		if let Some(neighbour) = node.neighbours[SOUTH] {
			assert!(testee.graph[neighbour].neighbours[NORTH] == Some(i)|| print_graph(testee), "\nNode #{} has a neighbour in the south, Node #{}, but the neighbourhood is not returned.\n", i, neighbour  );
		}
		if let Some(neighbour) =node.neighbours[WEST] {
			assert!(testee.graph[neighbour].neighbours[EAST] == Some(i) || print_graph(testee), "\nNode #{} has a neighbour in the west, Node #{}, but the neighbourhood is not returned.\n", i, neighbour  );
		}
	}
}

fn inv_all_shortest_paths_lead_to_destination(testee: &JkmShortestPathMap) {
	for i in 0..testee.graph.len() {
		assert!(shortest_path_leads_to_index(testee, i, testee.end_point_index, testee.graph.len())|| print_graph(testee) );
	}
}

fn shortest_path_leads_to_index(testee: &JkmShortestPathMap, start: usize, end: usize, allowed_calls: usize) -> bool {
	if start == end { true }
	else if allowed_calls == 0 {  println!("No way to get from Node #{} to Node #{}.", start, end); false }
	else { 
		if let Some(sp) = testee.graph[start].shortest_path {
			if let Some(next) =  testee.graph[start].neighbours[sp] {
				shortest_path_leads_to_index(testee, next, end, allowed_calls-1)
			}
			else { panic!("There was a shortest path marked from node {} in direction {} but there was no neighbour in this direction!\n", start, sp); }
		}
		else { true }
	}		
}

#[test]
fn simple_creation_test() {
	let start = (100.0, 0.0);
	let end = (100.0, 100.0);
	let map = (0.0,0.0,200.0,100.0);
	let spm = JkmShortestPathMap::new(start, end, map);
	assert!(spm.graph.len() == 6 || print_graph(&spm));
	let should_be_end = spm.graph[spm.start_point_index].neighbours[SOUTH].unwrap();
	assert!(should_be_end == spm.end_point_index || print_graph(&spm));
	assert!(spm.graph[spm.end_point_index].cost == 0.0 || print_graph(&spm));
	assert!(spm.graph[spm.start_point_index].cost == 100.0 || print_graph(&spm));
}

#[test]
fn more_general_creation_test() {
	general_test_creation(50.0, 50.0, 20.0, 90.0, (0.0,0.0,100.0,200.0) );
	general_test_creation(70.0, 60.0, 60.0, 70.0, (0.0,0.0,100.0,200.0) );
}

// not so general actually, can't change arrangement of start and endpoint
fn general_test_creation(start_x: f64, start_y: f64, end_x: f64, end_y: f64, map: (f64, f64, f64, f64)) {
	let start = (start_x, start_y);
	let end = (end_x, end_y);
	let total_cost = (start_y - end_y).abs() + (start_x - end_x).abs();
	let spm = JkmShortestPathMap::new(start, end, map);
	assert!(spm.graph.len() == 16 || print_graph(&spm));
	assert!(spm.graph[spm.end_point_index].cost == 0.0 || print_graph(&spm));
	assert!(spm.graph[spm.start_point_index].cost == total_cost || print_graph(&spm) , "Invalid cost: {}", spm.graph[spm.start_point_index].cost);
	
	
	if let Some(helper) = spm.graph[spm.start_point_index].neighbours[SOUTH]{
		assert!(spm.graph[helper].cost == (start_x - end_x).abs() || print_graph(&spm));
		if let Some( should_be_end) = spm.graph[helper].neighbours[WEST]
			{assert!(should_be_end == spm.end_point_index || print_graph(&spm));}
			else { assert!(print_graph(&spm)); }
	}
	else { assert!(print_graph(&spm)); }
	
	
	if let Some(other_helper) = spm.graph[spm.start_point_index].neighbours[WEST]{
		assert!(spm.graph[other_helper].cost == (start_y - end_y).abs() || print_graph(&spm));
		if let Some(should_be_end_too) = spm.graph[other_helper].neighbours[SOUTH] 
			{assert!(should_be_end_too == spm.end_point_index || print_graph(&spm));}
			else { assert!(print_graph(&spm)); }
	}
	else { assert!(print_graph(&spm)); }
	
}

#[test]
fn split_edge_test_v() {
	let start = (100.0, 0.0);
	let end = (100.0, 100.0);
	let map = (0.0,0.0,200.0,100.0);
	let mut spm0 = JkmShortestPathMap::new(start, end, map);
	let mut spm1 = JkmShortestPathMap::new(start, end, map);
	
	let var = spm0.start_point_index;
	assert_eq! (var, spm1.start_point_index);
	
	let var2 = spm1.end_point_index;
	assert_eq! (var2, spm0.end_point_index);
	
	let i0 = spm0.split_edge(var, SOUTH, 60.0);
	let i1 = spm1.split_edge(var2, NORTH, 60.0);
	
	check_module_invariants(&spm0);
	check_module_invariants(&spm1);
	
	assert!(i0 == i1, "Splitting the same edge from the north or from the south should give the same resulting index for the new node");
	assert!(spm0.graph[i0].x == 100.0 && spm1.graph[i1].x == 100.0);
	assert!(spm0.graph[i0].y == 60.0 && spm1.graph[i1].y == 60.0);
	
}

#[test]
fn split_edge_test_h() {
	let start = (0.0, 100.0);
	let end = (100.0, 100.0);
	let map = (0.0,0.0,100.0,200.0);
	let mut spm0 = JkmShortestPathMap::new(start, end, map);
	let mut spm1 = JkmShortestPathMap::new(start, end, map);
	let var = spm0.start_point_index;
	assert_eq! (var, spm1.start_point_index);
	
	let var2 = spm0.end_point_index;
	assert_eq! (var2, spm1.end_point_index);
	
	let i0 = spm0.split_edge(var, EAST, 60.0);
	let i1 = spm1.split_edge(var2, WEST, 60.0);
	
	check_module_invariants(&spm0);
	check_module_invariants(&spm1);
	
	assert!(i0 == i1, "Splitting the same edge from the north or from the south should give the same resulting index for the new node");
	assert!(spm0.graph[i0].y == 100.0 && spm1.graph[i1].y == 100.0);
	assert!(spm0.graph[i0].x == 60.0 && spm1.graph[i1].x == 60.0);
	
}




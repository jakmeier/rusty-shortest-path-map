use super::super::*;
use super::*;

// Tester module for integrations tests of the module JkmShortestPathMap
// This is a submodule of unit_tests rather than a client because we want to call the invariants defined in the unit_tests module

#[test]
fn usual_use_case_one() {
	let start = (100.0, 0.0);
	let end = (100.0, 300.0);
	let map = (0.0,0.0,200.0,300.0);
	let  mut spm = JkmShortestPathMap::new(start, end, map);

	check_module_invariants(&spm);
	assert!(spm.graph.len() == 6 || print_graph(&spm));
	
	let array_of_obstucles = [
		(80.0,20.0,40.0, 20.0),
		(140.0, 40.0, 50.0, 110.0),
		(50.0, 170.0, 20.0, 80.0),
		(150.0, 170.0, 10.0, 10.0),
		(150.0, 200.0, 10.0, 40.0),
		
	];
	
	for (i, &(x,y,w,h)) in array_of_obstucles.iter().enumerate() {
		spm.insert_obstacle(x,y,w,h);
		let filename = "usual_use_case_one_log".to_string() + &i.to_string();
		log_map(&spm, filename);
		check_module_invariants(&spm);
	}
	
	assert!(spm.graph[spm.start_point_index].cost == 340.00 || print_graph(&spm));
	
	
	spm.insert_obstacle(20.0,265.0,230.0,20.0);
	log_map(&spm, "usual_use_case_one_log_end".to_string());
	check_module_invariants(&spm);
	assert!(spm.graph[spm.start_point_index].cost == 460.00 || print_graph(&spm));
}

#[test]
fn usual_use_case_two() {
	let start = (0.0, 200.0);
	let end = (300.0, 200.0);
	let map = (0.0, 0.0, 300.0, 350.0);
	let  mut spm = JkmShortestPathMap::new(start, end, map);

	let expected_cost = 300.0 + 100.0 + 250.0 + 200.0 + 50.0;
	
	check_module_invariants(&spm);
	
	spm.add_map_border();
	
	check_module_invariants(&spm);
	
	let array_of_obstucles = [
		(30.0, -1.0, 40.0, 301.0),
		(100.0, 50.0, 50.0, 301.0),
		(200.0, -1.0, 50.0, 51.0),
		(200.0, 49.0, 50.0, 51.0),
		(200.0, 99.0, 50.0, 51.0),
		(200.0, 149.0, 50.0, 51.0),
		(200.0, 199.0, 50.0, 51.0),
	];
	
	for (i, &(x,y,w,h)) in array_of_obstucles.iter().enumerate() {
		spm.insert_obstacle(x,y,w,h);
		//if i%2 == 0 {
			let filename = "usual_use_case_two_log".to_string() + &i.to_string();
			log_map(&spm, filename);
		//}
		check_module_invariants(&spm);
	}
	
	log_map(&spm, "usual_use_case_two_log_end".to_string());
	assert!(spm.graph[spm.start_point_index].cost == expected_cost || print_graph(&spm));
}

#[test]
fn partial_overlapping_test() {
	let start = (100.0, 0.0);
	let end = (100.0, 300.0);
	let map = (0.0,0.0,200.0,300.0);
	let  mut spm = JkmShortestPathMap::new(start, end, map);

	check_module_invariants(&spm);	
	spm.insert_obstacle(50.0, 50.0, 100.0, 30.0);
	log_map(&spm, "partial_overlapping_log".to_string());	
	check_module_invariants(&spm);
	
	spm.insert_obstacle(60.0, 30.0, 10.0, 30.0);	
	log_map(&spm, "partial_overlapping_log".to_string());
	check_module_invariants(&spm);
	
	spm.insert_obstacle(100.0, 20.0, 30.0, 100.0);	
	log_map(&spm, "partial_overlapping_log".to_string());
	check_module_invariants(&spm);
	
}


// TODO List
	//	Invariant-styled:
		// There is no shorter path in the graph
		// There is no shorter path OUTSIDE of the nodes of the graph
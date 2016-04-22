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
fn usual_use_case_three() {
	let start = (0.0, 0.0);
	let end = (0.0, 300.0);
	let map = (-100.0, 0.0, 200.0, 300.0);
	let  mut spm = JkmShortestPathMap::new(start, end, map);
	
	check_module_invariants(&spm);
	
	spm.add_map_border();
	
	check_module_invariants(&spm);
	
	let array_of_obstucles = [
		(30.0, 20.0, 70.0, 70.0),
		(-60.0, 100.0, 70.0, 70.0),
		(-40.0, 150.0, 70.0, 70.0),
		(10.0, 130.0, 70.0, 70.0),
		(20.0, 220.0, 70.0, 70.0),
	];
	
	for &(x,y,w,h) in array_of_obstucles.iter() {
		spm.insert_obstacle(x,y,w,h);
		log_map(&spm, "usual_use_case_three_log".to_string());
		check_module_invariants(&spm);
	}
	
	let array_of_coordinate_checkpoint_pairs = [
		(end.0, end.1, None),
		(0.0, 220.0, Some((0.0, 290.0))),
		(10.0, 130.0, Some((30.0, 130.0))),
		(15.0, 130.0, Some((30.0, 130.0))),
		(45.0, 130.0, Some((80.0, 130.0))),
	];
	
	for (i, &(x, y, checkpoint)) in array_of_coordinate_checkpoint_pairs.iter().enumerate() {
		let result = spm.next_checkpoint(x,y);
		let result2 = spm.nearest_checkpoint(x,y);
		if let Some((new_x, new_y)) = checkpoint {
			assert!(result.is_some(), "The call to next_checkpoint() should return some checkpoint. (i={})", i);
			assert!(result2.is_some(), "The call to nearest_checkpoint() should return some checkpoint. (i={})", i);
			let result = result.unwrap();
			let result2 = result2.unwrap();
			assert!(result.0 == result2.0 && result.1 == result2.1, "The results from nearest and next checkpoint should always be the same. Next: [{}|{}], Nearest:[{}|{}] (i={})",result.0, result.1, result2.0, result2.1, i );
			assert!(result.0 == new_x && result.1 == new_y, "The resulting next checkpoint is not the expected. Got [{}|{}] but expected [{}|{}]. (i={})", result.0, result.1, new_x, new_y, i);
		}
	}
	
}

#[test]
fn no_path_available () {
	let start = (0.0, 0.0);
	let end = (0.0, 300.0);
	let map = (-100.0, 0.0, 200.0, 300.0);
	let  mut spm = JkmShortestPathMap::new(start, end, map);
	
	check_module_invariants(&spm);
	
	spm.add_map_border();
	
	check_module_invariants(&spm);
	
	let array_of_obstucles = [
		(-110.0, 150.0, 70.0, 50.0),
		(-60.0, 145.0, 70.0, 50.0),
		(0.0, 150.0, 70.0, 50.0),
		(50.0, 130.0, 70.0, 50.0),
	];
	
	for &(x,y,w,h) in array_of_obstucles.iter() {
		spm.insert_obstacle(x,y,w,h);
		let filename = "no_path_log".to_string();
		log_map(&spm, filename);
		check_module_invariants(&spm);
	}
	
	log_map(&spm, "no_path_log".to_string());
	
	assert!( !spm.graph[spm.start_point_index].shortest_path.is_some() || print_graph(&spm) );
	assert!( spm.graph[spm.end_point_index].cost == 0.0 || print_graph(&spm) );
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
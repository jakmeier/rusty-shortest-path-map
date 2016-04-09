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
	
	let array_of_obstucles = [
		(80.0,20.0,40.0, 20.0),
		(140.0, 40.0, 60.0, 110.0),
		(50.0, 170.0, 20.0, 80.0),
		(150.0, 170.0, 10.0, 10.0),
		(150.0, 200.0, 10.0, 40.0),
		
		(20.0,250.0,230.0,20.0),
	];
	
	for &(x,y,w,h) in array_of_obstucles.iter() {
		spm.insert_obstacle(x,y,w,h);
		check_module_invariants(&spm);
	}
	
}



// TODO List
	// There is no shorter path OUTSIDE of the nodes of the graph

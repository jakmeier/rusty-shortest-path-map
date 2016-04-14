# rusty-shortest-path-map
A rust library to maintain all the shortest paths on a two dimensional map with rectangular obstacles that are inserted dynamically.



The module seems to work in principle, all shortest paths are generated to the destination in the test cases which exist so far.

The following crucial features are still missing:
 - Check map boundary
 - Remove edges between two alligned obstacles
 - Allow obstacles to overlap eachother

Wish list:
 - Better and more test cases
 - Handle invalid inputs
 - Introduce sized characters which need to find the shortest path. Make it so that there are different sized characters and therefore different shortest paths on each node.

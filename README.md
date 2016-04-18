# Shortest Path Map - in pure rust
A rust library to maintain all the shortest paths on a two dimensional map with rectangular obstacles that are inserted dynamically.

To get an overview of the functionalities and a short description of all public functions, use *cargo doc*.

To create a map, ther is a constructor *new()*, afterwards all obstacles can be inserted after each other using *insert_obstacle()*. Two obstacles need to overlap each other by at least a bit to block the path between them, if the borders are on the exact same coordinate it will still be open. 

The map module is not aware of the size of the characters that will move through the map. The idea is to add the character size to the obstacle size, which should end up in the correct result. Unfortunately, this requires to have an instance of *JkmShortestPathMap* for each size of character that will move through the map. 

Wish list:
 - Better and more test cases
 - Handle invalid inputs better
 - Introduce sized characters stored in the module. There should be the possibilty to have several characters of different sizes which could, ideally, be added dynamically. This porbably involves different shortest paths on each node. Note that it would still be more efficient since the grid is built only once and it would be much more convenient to use the module.

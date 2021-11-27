# Tu Much Exploration @TUM
---
New repository for research @TUM looking into Multi-Robot Exploration by using various enhancements to existing Exploration techniques.

## Some known bugs for basic bug tracking

overall:  
fixed weird scan issue where part of the scans are ignored or something else for unknown reason  (hector_mapping ignores scans with too long range)

search:  
fixed costmap doesn't update on map/map_update refresh (temp fix by killing the tme_search node every time)  
fixed visualizeFrontiers breaks/ is broken  

assign:  
fixed some wrong topic names and minor bugs in assign  
fixed assign still crashes because of noneType Division  
fixed robots move to the same goal  
robots don't search for next goal after arriving/ mission success  
robots start randomly spinning and are not looking for new goals  

solutions:  
~~delete recovery procedures~~  
~~change move_base frequency~~  
~~xacro files fix~~  
~~turtlebot3 change robots~~  
change move_base  
change from proactive to reactive code  
~~add node between assign and move_base of robots~~  
blacklist goals somehow
change search time to be array

improvements:  
better distribution
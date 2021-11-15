# Tu Much Exploration @TUM
---
New repository for research @TUM looking into Multi-Robot Exploration by using various enhancements to existing Exploration techniques.

## Some known bugs for basic bug tracking

search: \
costmap doesn't update on map/map_update refresh (temp fix by killing the tme_search node every time)
fixed visualizeFrontiers breaks/ is broken

assign: \
fixed some wrong topic names and minor bugs in assign
fixed assign still crashes because of noneType Division
robots move to the same goal
robots don't search for next goal after arriving/ mission success
# search-best-hotel-place

First of all, the project need a input: open street maps export with hotels and sightseeing points especified. Nice tool to do this part is JOSM (https://josm.openstreetmap.de/).
Then the project run a Dijkstra algrithm to find the best point of map to build a hotel, moreover give a puntuation to each point of map. The puntuation depends on the distance between point and another hotels (bad points because are competitors) and distance between point and sightseeing (good points).
Finally, the output is a new map printed with GTK with all painted roads depends on puntuation.

**The project isn't finished, it's interisting and powerful if it's improved, there're some bugs.**

And need more features:
1. Edit map without external tool.
2. Only compare with hotels with similar characteristics.
3. Improve output.

I will export to Python because I'm more confortable.

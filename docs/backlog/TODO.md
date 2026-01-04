# Future work

1. Automatic position calculation of anchors
    - We can suppose a rectangular shape and use TWR on anchors in order to calculate it's position.
2. OTA
    - Special care needs to be taken regarding partitions. Keep in mind
3. Move current board definition to json objects that will define hw pinout and a file for preprocessor definitions.
4. Automatic calibration of antenna delay using 4 anchors and tags simultaneously.
    - Not sure how exactly it can be done and if it would be much useful. A lot of investigation needed.
5. Improve tdoa performance for lower than 8 anchors by making the tdoa slots dynamic based on number of anchors.
6. Espand the rtls-link-manager tool to support multiple configurations upload so many devices can be modified at the same time. The devices already have configuration files saved to them internally but also allow for local config files so we can upload them to newly flashed devices. 
    - We should also think about always having a default parameter configuration ( probably there already is ) so we can still work with newly flashed devices. 
7. Small enchancement on the stats of the discovery packets. Like showing the state of the tdoa tag ( if it sees all anchors, update rate, if it has sent the origin to the autopilot, etc)
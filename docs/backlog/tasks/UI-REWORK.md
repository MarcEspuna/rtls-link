# The task

We want to do a complete UI rework of the rlts-link-manager tool ( located at `tools/rtls-link-manager`). Since we plan to add new feature that extend the functionality way more from what was initially designed for we want to rethink how to organize, structure and present the information and functionality to the user. 

- Now the application will be able to configure anchors and tags and separate them on the UI so users can easly differentiate between anchors and tags on the UI.
- We want to make this application easy to use but also advanced for developers. We should give a simplified user options but also an "advanced" or "expert mode" tab for enabling full customizations
- We want to retrieve telemetry from the anchors and tags. It's very important to show to the users that the full system is working well, anchors and tags. 
- The UI should have a left panel where it should have two different tabs for the anchors menu and the tags menu.
    - On the anchors menu we should have the telemetry information from the anchors and same on the tags menu. The idea is that the application will be able to show a green status, yellow, orange or red on each device based on the provided telemetery ( overall health based on what we know on how the system should behave ). 
    - The telemetry from the devices does not need to be sent very often. It's purely for monitoring, no quick realtime is needed here.
    - There is a very important feature that we plan to add which is the ability for the tags to automatically determine the positions of the anchors. This will be made by simply looking at the distance measurements that the anchors are already doing and encoding into it's UWB packets. Then, if we just suppose a rectangular shape, the tags can self locate ( also we will stablish a 0,0,0 origin which will always be the anchor 0) the coordinates of the anchors can be known. 
- We should also have another tab with the absolute locations that we have saved on the local application. This provide the ability so save new locations, show them to the user and upload them to all the tags.
- Small detail: Drop the tag UWB id as a relevant data. Use the IP address for the main identification of the devices.
- The tags and the anchors will be able to have a special "debug" socket ( there is some legacy code that does that but we can completely replace that with a newer and proper implementation ) for sending debug information that will be handy when developing the automatic anchor positioning.
- We can simplify completely the ability for the devices to store multiple configurations. It basically complicates things and does not provide much real benefit since we generally want to have configurations in sync (except for the MAVID, when uploading settings to all devices the mavid should remain unchanged).

# Additional considerations

- Focus mainly on the desktop application for now. Since the devices won't have updated firmware accordingly, make sure to document all the relevant considerations and changes that will need to be made to the devices to complete the functionality on the entire system. If a device is not updated with the new functionality, the desktop app should handle it gracefully. 
- Make sure to first understand perfectly what we currently have on the application in order to not regress on any current functionality that we already have unless explicitly specified.
# Topside\_UI

This node is used for the nekton ui that will be displayed on the topside computer. The code for the ui is in the src folder.

## Note about components
Apparently the component tag name that is set as the first argument in the customElements.define call MUST be all lowercase and include a hyphen.

## Running the UI
To use the UI make sure to `launch` the rest of the code and then open the `/src/index.html` file in a web browser.


## Subscribing to a topic
First, set up a subscriber by making a new ROSLIB.Topic class with the name being the name of the topic you want to subscribe to and the messageType being the message type of the topic (You can find this by running `ros2 topic type <topic_name>` while the rest of the code is running):
```
// Subscribing to a Topic
var subscriber = new ROSLIB.Topic({
  ros: ros,
  name: "/nekton/robot_description",
  messageType: "std_msgs/String",
});
```




Then, make the callback function that will be called for each message that is recieved by the subscriber. The data recieved will be passed into the function as a javascript class. This particular topic uses `std_msgs/String` which looks like [this](https://docs.ros2.org/galactic/api/std_msgs/msg/String.html) so the only member for us to access is `data` which will contain the string we want to display:
```
function callback(message) {
  console.log(message.data);
}
```


Finally, have the subscriber subscribe to the topic:
```
subscriber.subscribe(callback);
```
Now when a message is published to the `/nekton/robot_description topic` the string will be printed in the web browsers console.

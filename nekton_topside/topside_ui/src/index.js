// Connecting to ROS
var ros = new ROSLIB.Ros({
  url: "ws://localhost:9090",
});

ros.on("connection", function () {
  console.log("Connected to websocket server.");
});

ros.on("error", function (error) {
  console.log("Error connecting to websocket server: ", error);
});

ros.on("close", function () {
  console.log("Connection to websocket server closed.");
});

// Subscribing to a Topic
var listener = new ROSLIB.Topic({
  ros: ros,
  name: "/nekton/pressure",
  messageType: "sensor_msgs/FluidPressure",
});

listener.subscribe(function (message) {
    const x = message.fluid_pressure / 10051.8; //not sure which formula is correct
    // const x = message.fluid_pressure / (1000 * 9.81);
    const myElement = document.getElementById('my-element');
    myElement.innerHTML = "Depth: " + x;
    // console.log("Received message on " + x);
  // listener.unsubscribe();
});

var laserScanListener = new ROSLIB.Topic({
  ros: ros,
  name: "/nekton/sonar_forward",
  messageType: "sensor_msgs/LaserScan",
});

laserScanListener.subscribe(function(laserScanMessage) {
  console.log("here");

  //Get angle info
  const angleMin = laserScanMessage.angle_min;
  const angleMax = laserScanMessage.angle_max;
  const angleIncrement = laserScanMessage.angle_increment;
  //Get range info
  const ranges = laserScanMessage.ranges;
  const rangeMin = laserScanMessage.range_min;
  const rangeMax = laserScanMessage.range_max;

  let x = [];
  let y = [];

  for (let i = 0; i < ranges.length; i++) {
    //Get current range to work with
    const range = ranges[i];

    //Filter out invalid ranges
    if (range >= rangeMin && range <= rangeMax) {
      const angle = angleMin + i * angleIncrement;
      const xVal = range * Math.cos(angle);
      const yVal = range * Math.sin(angle);

      x.push(-yVal);
      y.push(xVal);
    }
  }

  console.log("X points: ", -y);
  console.log("Y points: ", x);
  


  /*for(i = 0; i < pointcloud_message.data.length; i++) {
    if (i%3 == 0) {
      x.push(pointcloud_message.data[i].charCodeAt(0))
    } else if (i%3 == 1) {
      y.push(pointcloud_message.data[i].charCodeAt(0))
    } else if (i%3 == 2){
      z.push(pointcloud_message.data[i].charCodeAt(0))
    }*/
    
  
  //console.log(z);
  var trace1 = {
    type: "scatter",
    mode: "markers",
    marker: {
      color: "rgba(0, 255, 0, 0.8)",
      size: 4
    },
    x:x,
    y:y
  }
  
  var data = [trace1];
  
  var layout = {
      plot_bgcolor:"black",
      paper_bgcolor:"black",
      title: {
        text: '2D Laser Scan Visualization',
        font: {
          family: 'Courier New, monospace',
          size: 30,
          color: 'Red'
        }
      },
    xaxis: {
      type: "linear",
      title: 'X (meters)',
      range: [-10, 10],  // Restricted between -10 and 10
      autorange: false
    },
    yaxis: {
      type: "linear",
      title: 'Y (meters)',
      range: [0, 10],  // Restricted between 0 and 10
      autorange: false
    },
    height: 598,
    width: 1080,
    autosize: true,
    showlegend: false
  }
  const plotElement = document.getElementById('laser-scan-plot');
  Plotly.newPlot(plotElement, data, layout);
}); 

// Subscribe to sonar topic
// var sonar_listener = new ROSLIB.Topic({
//   ros: ros,
//   name: "/nekton/sonar_forward/sonar_points",
//   messageType: "std_msgs/Float32MultiArray",
// });

// sonar_listener.subscribe(function (sonar_message) {
//   // sonar_message holds x,y,z,i values in the following format:
//   // [x0, y0, z0, i0, x1, y1, z1, i1, x2, y2, z2, i2 ..... xn, yn, zn, in], where i = intensity

//   x = []
//   y= []
//   z = []
//   intensity = []

//   for(i = 0; i < sonar_message.data.length; i++) {
//     if (i%4 == 0) {
//       x.push(sonar_message.data[i])
//     } else if (i%4 == 1) {
//       y.push(sonar_message.data[i])
//     } else if (i%4 == 2){
//       z.push(sonar_message.data[i])
//     } else {
//       intensity.push(sonar_message.data[i])
//     }
//   }

//   var trace1 = {
//     type: "pointcloud",
//     mode: "markers",
//     marker: {
//       sizemin: 0.5,
//       sizemax: 35,
//       arearatio: 0,
//       color: "rgba(0, 255 0, 0.8)"
//     },
//     x:x,
//     y:y
//   }
  
//   var data = [trace1];
  
//   var layout = {
//       plot_bgcolor:"black",
//       paper_bgcolor:"black",
//       title: {
//         text: 'Forward Sonar Point Cloud',
//         font: {
//           family: 'Courier New, monospace',
//           size: 30,
//           color: 'Red'
//         }
//       },
//     xaxis: {
//       type: "linear",
//       range: [
//         -2.501411175139456, 4],
//       autorange: true
//     },
//     yaxis: {
//       type: "linear",
//       range: [-1.5,.25],
//       autorange: true
//     },
//     height: 598,
//     width: 1080,
//     autosize: true,
//     showlegend: false
//   }
//   Plotly.newPlot('myDiv', data, layout);
// }); 

// Subscribe to image_raw topic
var image_listener = new ROSLIB.Topic( {
  ros: ros,
  name: "/nekton/camera_forward/image_raw",
  messageType: "sensor_msgs/msg/Image",
});

//create canvas
const can = document.createElement("canvas");

const ctx = can.getContext("2d");

image_listener.subscribe(function (image_message) {
  // console.log("Received image message on " + image_listener.name + ": " + image_message.width + " "  + image_message.height);
  // console.log("is_bigendian: " + image_message.is_bigendian);
    
  //creating a canvas of correct size and obtaining a CanvasRenderingContext2D
  can.width = image_message.width;
  can.height = image_message.height; 

  //create an image buffer to hold the pixels
  const imgData = ctx.createImageData(image_message.width, image_message.height);
  const data = imgData.data;
  const inData = atob(image_message.data);

  var j = 0; i = 4; // j data in , i data out
  while( j < inData.length) {
      const w1 = inData.charCodeAt(j++);  // read 3 16 bit words represent 1 pixel
      const w2 = inData.charCodeAt(j++);
      const w3 = inData.charCodeAt(j++);
      if (!image_message.is_bigendian) {
          data[i++] = w1; // red
          data[i++] = w2; // green
          data[i++] = w3; // blue
      } else {
          data[i++] = (w1 >> 8) + ((w1 & 0xFF) << 8);
          data[i++] = (w2 >> 8) + ((w2 & 0xFF) << 8);
          data[i++] = (w3 >> 8) + ((w3 & 0xFF) << 8);
      }
      data[i++] = 255;  // alpha
  }

  //put pixel data into the canvas
    ctx.putImageData(imgData, 0, 0);

  //add canvas to the HTML
  document.body.appendChild(can);

})

// Creating a Service

var cmdVel = new ROSLIB.Topic({
  ros : ros,
  name : '/nekton/cmd_vel',
  messageType : 'geometry_msgs/Twist'
});

//Initializes the Keyboard Map. This way, NaN is not possible with the controls needed.
let keyboard = {w: 0, a: 0, s: 0, d: 0, q: 0, e: 0, ArrowUp: 0, ArrowDown: 0, ArrowRight: 0, ArrowLeft: 0, z: 0, c: 0};
//Feel Free to edit
let velocityMod = 14;
let twistMsg;

/*
NOTE: Maybe filter Twist Messages checking for Duplicates. 
Could affect performance but I am unsure atm.
*/
document.addEventListener('keydown', async (e) => {
  /*
  TODO: Add Control to modify velocity by some scalar.
  Was planning on using scroll wheel, but another option is
  checking if shift is down on a key press, then modify.
  */
  keyboard[e.key] = 1.0 + velocityMod;
  twistMsg = await generateTwist();
  //DEBUGGING! Feel Free to Remove
  // console.log(twistMsg);
  cmdVel.publish(twistMsg);
})

//Resets Value to Zero on key up
document.addEventListener('keyup', async (e) => {
  keyboard[e.key] = 0.0;
  twistMsg = await generateTwist();
  //DEBUGGING! Feel Free to Remove
  // console.log(twistMsg);
  cmdVel.publish(twistMsg);
})

//Function to get the current control values
const generateTwist = async () => {
  return (new ROSLIB.Message({
    linear : {
      x : keyboard.d - keyboard.a,
      y : keyboard.w - keyboard.s, 
      z : keyboard.q - keyboard.e
    },
    angular : {
      x : keyboard.ArrowUp - keyboard.ArrowDown,
      y : keyboard.c - keyboard.z,
      z : keyboard.ArrowRight - keyboard.ArrowLeft,
    }
  }));
}

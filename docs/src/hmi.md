<style>
    img[src*='#left'] {
     float: left;
    }
    img[src*='#right'] {
        float: right;
    }
    img[src*='#center'] {
        display: block;
        margin: auto;
    }
</style>

# M2 HMI ![MODAQ M2 HMI Example Design](img/m2_logo.png#right)

</p>
A Human-Machine Interface, or HMI, is a control panel or portal that enables high-speed, near realtime interaction with M2. It can display or plot values and allow for operator control of specifically coded elements running in M2- such as manual control of a relay. 

![](img/M2_HMI_screenshot.png#center)

Typically, the HMI would be referred to as the *frontend*, while the bulk of the MODAQ code responsible for the actual data acquisition, control outputs, and general logic is considered the *backend*. 

M2 uses a standard web development stack consisting of HTML, JavaScript (JS), and CSS to render the HMI. A web server runs on the M2 controller and users (clients) connect to the HMI using their web browser. For security purposes, the HMI is only accessible if the client is on the same subnet as the M2 controller. Remote access from anywhere in the world could be accomplished using a VPN gateway.  

The frontend and backend communicates through the use of a <a href="https://www.tutorialspoint.com/html5/html5_websocket.htm" target="_blank">websocket</a>, which provides a dynamic, bidirectional interface to exchange data through ROS2 topics or services.

## Getting Started

This will not be a detailed tutorial and it assumes some familiarity with the ROS2 environment, Linux terminal, and web server/client basics. These are the following minimum requirements to get an M2 HMI up and running:

Backend:

- ROS2 Humble installed on the M2 controller. This requirement should already be met if ROS2 was installed in the previous section on getting MODAQ up and running. 
- <a href="https://github.com/RobotWebTools/rosbridge_suite" target="_blank">rosbridge_suite</a>. This creates the backend service to allow frontend interaction with the ROS2 environment. 
- <a href="https://www.nginx.com/resources/wiki/start/topics/tutorials/install/#official-debian-ubuntu-packages" target="_blank">nginx</a> web server. Web servers such as lighttp, the venerable Apache, or numerous others could be used instead of nginx. The choice of web server is mostly inconsequential as long as it supports modern web technologies and is reasonably lightweight. We will only provide guidance for configuration and use of nginx. 
- At least one ROS2 node running on the M2 controller as publisher and optionally a subscriber node. 

Frontend:

- An IDE or text editor for composing the web documents. If you're using VS Code for ROS2, that will work fine.
- <a href="https://github.com/RobotWebTools/roslibjs" target="_blank">roslibjs</a>. This will be installed as part of the rosbridge_suite on the backend, but needs to be 'visible' to the web server. Visibility might involve providing the correct path to the library or our preference, place of copy of the <a href="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js" target="_blank">minified</a> (<a href="https://github.com/RobotWebTools/roslibjs/blob/ros2/build/roslib.min.js" target="_blank">alternative link</a>) roslib in the /js folder in your nginx html document root.[^1] 
- <a href="https://www.chartjs.org/" target="_blank">chart.js v3.3.2</a>[^2] (<a href="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/3.3.2/chart.min.js" target="_blank">minified</a>). This is only required if you want to have plots in your HMI. 
- <a href="https://nagix.github.io/chartjs-plugin-streaming/master/" target="_blank">chartjs-plugin-streaming</a> <a href="https://github.com/nagix/chartjs-plugin-streaming/releases/download/v2.0.0/chartjs-plugin-streaming.min.js" target="_blank">(minified)</a>. and its dependencies[^3] (which are noted in its Getting Started section of the chartjs-plugin-streaming webpage linked in the last sentence). This is only required if you intend to have plots in your HMI and want those plots to update in near realtime (AKA streaming).

!!! note

    The code uses offline versions of each of these libraries, which allows the HMI to operate without needing to have an active internet connection for dynamically loading the libraries from a Content Delivery Network (CDN). Because of this, the libraries are frozen in time and may not have the latest and greatest features and will lack security updates and bugfixes. At this point, the libraries appear to be sufficiently stable and since the HMI is served on a private subnet, security should not be an issue. It should be cautioned to only use the versions of those libraries that the code was originally written for, since different versions can break stuff.
    
    Further, getting offline versions of JS libraries is becoming more difficult. It used to be a given that a library would have a simple and clear link to the offline version, but that is slowly changing. Since most web servers are by nature 'online', it makes sense to pull the libraries as needed from a CDN which should assure the libraries are up to date, with all recent bugfixes and security patches. This is also smart from a marketing/tracking perspective, since the developer can have a better feel for the userbase and popularity of their scripts. However, CDNs can go down, scripts can be moved to different CDNs, developers can pull/change features or alter the licensing terms, or there can be network issues causing the script not to load, so for mission critical stuff and particularly if this server does not have continuous internet access, an offline version is necessary


### nginx Configuration

nginx can be install with the following terminal command:

```
sudo apt install nginx
```

Configuration of nginx will live in these folders:

```
/etc/nginx/sites-available
```

```
/etc/nginx/sites-enabled
```

The website (HMI) is hosted from the `/var/www/` folder by default. Multiple websites can be hosted simultaneously with nginx and they can be kept organized in their own folders. This discussion will assume a folder called `modaq_hmi` was created in `/var/www/`

Since the config and content folders live in root, permissions will be necessary to manipulate these files from a standard user account. Using temporary privilege escalation (`sudo`) will suffice for the nginx configuration files, but this can get tedious for the HMI content pages. Best Practices would be to edit these files in user space (i.e. a folder downstream of the user's /home folder) and include them in version control (github), then copy them to the production space (`/var/www/modaq_hmi`) when ready to take them live.

#### Set User Permissions to nginx Web Page Folder

While this is optional, it will make life easier during the dev process to add user permissions to thw `/var/www/modaq_hmi` folder. Otherwise, every single change will require sudo privilege escalation.  

This sets the local user 'modaq' with rwx privileges in the modaq_hmi folder recursively:

```
sudo setfacl -R -m u:modaq:rwx /var/www/modaq_hmi
```

It may be necessary to do a chmod afterwards:

```
sudo chmod -R 755 /var/www/modaq_hmi
```

#### Create nginx Virtual Server Configuration File for HMI

As noted earlier, nginx creates two folders for the server configuration. The `sites-available` folder contains the configurations for one or more websites, while the `sites-enabled` folder contain the configuration(s) for the site(s) that nginx is currently serving. Think of `sites-available` as the parking area and `sites-enabled` as the active/production area. Instead of actually moving or copying the configuration file to the `sites-enabled` folder, nginx expects only a <a href="https://www.geeksforgeeks.org/how-to-symlink-a-file-in-linux/" target="_blank">symbolic link</a> to the file in the `sites-available` folder. The following shows the contents of a configuration file that works for us:


```yaml
server {
    listen 80;
    listen [::]:80;

    root /var/www;

    # Add index.php to the list if you are using PHP
    index index.html index.htm;

    # For this to work, needs DNS resolution or entry in 
    # hosts files on client computer
    server_name modaq.hmi www.modaq.hmi;

    location / {
        # First attempt to serve request as file, then
        # as directory, then fall back to displaying a 404.
        try_files $uri $uri/ =404;
    } 
}
```

A couple of notes about this configuration: 

1. This configuration is for an unsecured connection (i.e. not https://) and is probably safe for the private subnet which serves this HMI. However, the commented section on `SSL configuration` could be enabled to provide secure connections- if certificate dependencies are met. Some web browsers might complain about connecting to the HMI citing that the connection is unsecure and potentially dangerous if SSL is not enabled. It may be necessary to click through some warnings or whitelist the site. 
2. The entry for `server_name` was an attempt allow users to type modaq.hmi into their browser instead of having to know the IP address of the controller. While that would be convenient, more work needs to be done to get it working, so we backburnered this for other priorities. 

#### Ubuntu Firewall Rules
If the Ubuntu firewall (`ufw`) is enabled on the controller, it may be necessary to create a firewall rule to allow nginx to access the network. This is done with a simple terminal command:

```
sudo ufw allow 'Nginx HTTP'
```

## ROSBridge Server
The primary mechanism providing connectivity between ROS2 nodes running on M2 and the nginx web server is a websocket that is managed through rosbridge and roslibjs. Rosbridge is the service running along with M2 ROS2 nodes and exposes all published topics to a websocket.

If the rosbridge_suite was properly installed, the rosbridge server can be started with this command:

`ros2 run rosbridge_server rosbridge_websocket`

It's recommended to add this as the last item in the M2 launch file so that it starts automatically with the rest of the ROS2 nodes.

```
Node(
    package='rosbridge_server',
    executable='rosbridge_websocket',
    name='rosbridge'
)
```

## Web Client Framework

 There are four main parts of the web code that are necessary for pub/sub with ROS2 on the M2 system
    
1. Establish the websocket connection to ROSBridge 
2. Create a listener for ROS2 topics (subscribe) and/or define a topic to publish
3. Handle incoming/outgoing messages
4. Create the user interface (actual webpage)

### Basic HMI Design Elements

The M2 HMI uses a simple but effective grid of tiles to organize the content. This is accomplished through a hierarchy of `<div>` tags to define the sections (or <i>div</i>isions) with attributes applied from CSS classes. 

In this public release, these basic features are included in the HMI:

1. Traffic Lights - These are multi-modal text boxes that can change color based on logic contained in the accompanying JS. The traffic lights can be used to indicate whether a parameter is nearing or exceeding defined thresholds or indicate an alarm condition.
2. Scalar Data -  Display the current numeric value of certain parameters from a subscribed topic.
3. Streaming Plots - The examples include single and multivariate plots, as well as dual y-axis plots. 
3. Boolean Tick Boxes - These allow publishing boolean values back to M2 that can be used for manual control or setting the state of certain features. 
4. Text/Numeric Entry Boxes - These can publish numeric or text values to M2.

Since everything is written in ordinary HTML/JS/CSS and a handful of publicly available JS libraries, users can easily leverage these examples to develop a customized HMI suitable for their application. 

The tile system (aka <a href="https://www.w3schools.com/css/css_grid.asp" target="_blank">grid</a>) is quite flexible, allowing one to many content tiles on each row. While the height of each row can be constrained, the example included in this repo allows the height to grow to fill the content. The width of each tile is constrained to a fraction of the overall width allowed by the top-level (parent) div container. 

Diving deeper, the parent div is configured with 4 available columns in the included `style.css` file and looking at the screen shot of the HMI page at the beginning of this section, the MODAQ banner is allowed to span 3 columns in width, while the rosbridge status tile is set to 1 column- for a grand total of 4 columns. 

#### roslib Subscriber

roslib.js is a javascript library used on the webserver that can subscribe to topics published on M2 and display the data in a webpage. In addition it allows for publication of topics from the webpage as well as using or creating services.

First, an object is created to establish the websocket connection to the ROS2 instance on the M2 controller (showing the most relevant code snippet):

`const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });`[^4]

Next, a listener object is created to subscribe to the desired topic being published on the controller. In this example we have a temperature and humidity sensor that is acquired in M2 using MODBUS TCP and the scaled data are published to the `ths` topic. 

```javascript
// MODBUS Temperature and Humidity Sensor
var ths_listener = new ROSLIB.Topic({
    ros: ros,
    name: '/ths',
    messageType: 'ths_modbus/msg/Th',
    throttle_rate: 1000 // throttle_rate sets update interval in ms
});
```

When a new message is received, it's copied to a variable with global scope for further manipulation within the curly braces. 

```javascript
ths_listener.subscribe(function (message) {
    // copy ROS data to var with global scope
    ths_data = message;
    // do stuff here with the topic data
});
```

#### roslib Publisher

The roslib publisher follows a similar pattern to the Subcriber, first a topic object is created:

```javascript
var hmiCtl = new ROSLIB.Topic({
    ros: ros,
    name: '/hmi_ctl',
    messageType: 'modaq_messages/msg/Hmi'
});
```
We had previously created a unique message schema for the HMI in ROS2 on the M2 controller and included it during the build process, which includes the parameters we want to publish from the HMI.  

An event listener is bound to the "Push to MODAQ" button in the HMI, triggering a read of the user controls... 

```javascript
var hmiPub = new ROSLIB.Message({
    override: user_override,
    do0: user_do0,
    do1: user_do1,
    do2: user_do2,
    do3: user_do3,
    int1: user_int1,
    message1: 'Hello Casey',
    message2: 'Hello Aidan',
    float2: 3.17,
    header: {
        frame_id: '1'
    }
}
```

and publishes them.
```
hmiCtl.publish(hmiPub);
```

#### Traffic Lights
The appearance of the traffic lights is done with CSS and the color change is triggered by some JS logic. 

```javascript
if (ths_data.humidity > 90) {
        document.getElementById("status1").innerHTML = "<span class='traffic_light false'>Humidity</span>";
    } else if (ths_data.humidity > 65) {
        document.getElementById("status1").innerHTML = "<span class='traffic_light warn'>Humidity</span>";
    } else {
        document.getElementById("status1").innerHTML = "<span class='traffic_light true'>Humidity</span>";
    }
```
In this snippet, if the humidity is >90, the traffic light identified by the tag `status1` is styled using a combination of two CSS classes: `traffic_light false`, which in this case will color it red. Similarly, if it's >65, it's colored yellow, otherwise it's green. 

#### Streaming Charts
We chose chart.js as the charting engine for the M2 HMI, since it's highly customizable, capable, and looks good. It also has great support (both from the developer and user community), which is important, since it can be a little complicated to use. Because of this complexity, this section will not go into detail on its use, but will point out a few details around getting the streaming to work correctly - particularly with multivariate/dual-axis plots. 

chart.js, without the `chartjs-plugin-streaming` plugin, expects to be passed a static numeric array and it renders into a plot
with the selected attributes. The streaming plugin (basically) creates a fifo buffer for the data points and animates them on the x-axis.

!!! tip

    limit the chart history in order to assure smooth operation- especially with multiple charts on a single HMI page.

The architecture of the HMI web stack purposely places the computational burden of chart composition and rendering on the web client (i.e. your browser) not the M2 controller. Therefore, low-powered computers may struggle with smooth animation if the HMI has a lot of plots and/or the chart history is too big. The example included in this repo updates the chart at 1 Hz and displays up to 1 minute of data history. The update rate and history depth can be adjusted to preference.

chart.js breaks up the chart composition into 3 blocks: setup, config, and render. The setup block defines the y-axis datasets, labels, and plot trace attributes. The config block expands on this by defining additional attributes and binding the dataset(s) to an object that gets rendered in the render block. This all expects the data to formatted as an array, however our data are constructed into objects. 

To overcome this, we define a object literal to map the y-axis label(s) to the label used to identify the desired element in the data object (example for the accelerometer data). 

```
const dataMap = { Ax: 'ax', Ay: 'ay', Az: 'az' };
```
Then we insert each data point into the chart object:

```
chart.data.datasets.forEach(dataset => {
    dataset.data.push({
        x: Date.now(),
        y: mpu6050_data[dataMap[dataset.label]],
    });
});
```
> Side Note - We typically use high quality inertial sensors in our projects and eagle-eyed readers will notice the MPU-6050 referenced in a few places in this documentation and the github repo. The MPU-6050 is a <$5 sensor that has some utility in the occasional low-cost/low demand applications (see: <a href="https://github.com/NREL/MODAQ-BB" target="_blank">MODAQ-BB</a>). 

Similarly, handling a multivariate data object and plotting with dual y-axes took some creativity, but is simpler as long as only 2 parameters are plotted:

```
chart.data.datasets.forEach(dataset => {
    dataset.data.push({
        x: Date.now(),
        y: dataset.label === 'Temperature' ? ths_data.temperature : ths_data.humidity
    });
});
```





[^1]: Using the minified version of any of the required JS libraries is totally optional. We prefer minified, since it's a smaller overall payload and can improve performance. The disadvantage to using minified libraries is that all unnecessary whitespace characters are removed, so inspecting or edited the library code can be very difficult- even though it's still in plain text.  
[^2]: We're using chartjs v3.3.2 since at the time we were creating the HMI there were compatibility issues with the plugin-streaming library on v4 and higher. That may have been resolved in later releases of plugin-streaming. 
[^3]: Since finding the dependencies as minified libraries in the correct version can be a bit of a pain, here are direct links: <a href="https://cdn.jsdelivr.net/npm/luxon@3.4.4/build/global/luxon.min.js" target="_blank">luxon v3.4.40<a/> and <a href="https://cdn.jsdelivr.net/npm/chartjs-adapter-luxon@1.0.0" target="_blank">chartjs-adapter-luxon v1.0.0</a>
[^4]: Replace 'localhost' with the IP address of the M2 controller hosting the HMI webpage. 


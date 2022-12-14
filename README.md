# Viam ROS Camera Wrapper

This has been tested with ROS 1 Noetic, Melodic testing is underway

## ROS Configuration

Our robot was configured with a standard USB camera on both the Melodic and Noetic releases. The [video_stream_opencv](http://wiki.ros.org/video_stream_opencv)
package was used.

If roscore is not running, make sure it is running in one window
```shell
roscore
```
***Note***: for our local testing we only start ROS when testing

In another window start our camera publisher node
```shell
roslaunch video_stream_opencv camera.launch
```
If there are multiple cameras we need to set the appropriate video stream provider argument.

List the camera topics
```shell
rostopic list 
/camera/camera_info
/camera/camera_stream/parameter_descriptions
/camera/camera_stream/parameter_updates
/camera/image_raw
/camera/image_raw/compressed
/camera/image_raw/compressed/parameter_descriptions
/camera/image_raw/compressed/parameter_updates
/camera/image_raw/compressedDepth
/camera/image_raw/compressedDepth/parameter_descriptions
/camera/image_raw/compressedDepth/parameter_updates
/camera/image_raw/theora
/camera/image_raw/theora/parameter_descriptions
/camera/image_raw/theora/parameter_updates
/rosout
/rosout_agg
```
### Noetic

Tested & working

### Melodic

Currently still testing with ROS Melodic

### To consider


### Possible issues
When installing `viam-sdk` on jetson running ROS 1 Melodic, it will be required to download and compile Python3.9

## Setup Viam Remote

```shell
git clone https://github.com/shawnbmccarthy/viam-ros-camera-wrapper
cd viam-ros-camera-wrapper
./setup_venv.sh
```
This will setup our python virtual environment and install the required packages

## Configure Robots remote resources

The viam server component interface allows us to create custom components which interface 
with the `viam-server` using the viam SDK's. Our example makes use of the [Python SDK](https://python.viam.dev/)
to create custom sensors.



1. log into [app.viam.com](https://app.viam.com)
2. access the robot you configured, and go to the remotes tab
![remotes_create.png](images%2Fremotes_create.png)
3. give the remote a name, for this example lets use `ros-camera-wrapper`, next select `Create Remote`, this will show the next page:
![remote_create_2.png](images%2Fremote_create_2.png)
4. in the `Heading Info` text box enter `localhost:9090`, this is the default binding that [camera_remote.py](camera_remote.py) is configured for. Now click `Save Config` at the bottom of the page, a confirmation message will appear.

Now that we have created our remote resource, we can start our proces
```shell
# run from viam-simple-sensor
./camera_remote.sh -l DEBUG
```

This will start a remote process with `DEBUG` log level. By default, only `WARNING` and above are logged, you should see
the output below:
```shell
2022-12-14 01:08:42,088         INFO    viam.rpc.server (server.py:81)  Serving on localhost:9090   
```

Now go back to [app.viam.com](https://app.viam.com) and select the `Control` tab of your robot and select `Sensors`, 
here we can select `Get All Readings` to see the output
![control.png](images%2Fcontrol.png)

To see how the code works view: [camera_remote.py](camera_remote.py) and [ros_camera.py](ros_camera.py)

### script usage
Below are the options that can be used 

```shell
./camera_remote.sh -h
usage: camera_remote.py [-h] [--host HOST] [--port PORT] [--log {DEBUG,INFO,WARNING,ERROR,FATAL}] [--topic TOPIC]

options:
  -h, --help            show this help message and exit
  --host HOST, -n HOST  hostname/ip rpc server will bind to
  --port PORT, -p PORT  port number to store
  --log {DEBUG,INFO,WARNING,ERROR,FATAL}, -l {DEBUG,INFO,WARNING,ERROR,FATAL}
                        log level to use
  --topic TOPIC, -t TOPIC ros camera topic
```
With no options the defaults are:
* host = localhost 
* port = 9090
* log = WARNING
* topic = camera/image_raw

## Further activities
We can also configure our viam server to manage the remote process, ensuring that when viam server is running the remote server
will also run. 

# contact

For any questions, please email me at: [shawn@viam.com](mailto:shawn@viam.com)

# References
1. [viam](https://viam.com)
1. [viam cloud](https://app.viam.com)
1. [viam docs](https://docs.viam.com)
1. [viam python sdk](https://python.viam.dev)
1. [sdk as server](https://docs.viam.com/product-overviews/sdk-as-server/)
1. [subclassing components](https://python.viam.dev/examples/example.html#subclass-a-component)
1. [drive a rover](https://www.viam.com/resources/try-viam)

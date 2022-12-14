# Viam ROS Camera Wrapper

This has been tested with ROS 1 Noetic, Melodic testing is underway

## Ros Requirements

Our robot was configured with a standard USB camera on both the Melodic and Noetic releases. The [video_stream_opencv](http://wiki.ros.org/video_stream_opencv)
package was used.

```shell
roslaunch video_stream_opencv camera.launch
```
If there are multiple cameras we need to set the appropriate video stream provider argument.

### Noetic

### Melodic

### To consider


### Possible issues
When installing `viam-sdk` on jetson running ROS 1 Melodic, it will be required to download and compile Python3.9

Will provide more info later

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
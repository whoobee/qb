var app = new Vue({
    el: '#app',
    // storing the state of the page
    data: {
        connected: false,
        ros: null,
        logs: [],
        loading: false,
        rosbridge_address: 'ws://192.168.0.67:9090',
        port: '9090',
        mapViewer: null,
        mapGridClientNav: null,
        interval: null,
        nav_cancel_topic: null,
    },
    // helper methods to connect to ROS
    methods: {
        connect: function() {
            this.loading = true
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })
            this.ros.on('connection', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                this.connected = true
                this.loading = false

                this.mapViewer = new ROS2D.Viewer({
                    divID: 'map',
                    width: 640,
                    height: 480
                })

                // Setup the map client.
                this.mapGridClientNav = NAV2D.OccupancyGridClientNav({
                    ros: this.ros,
                    rootObject: this.mapViewer.scene,
                    viewer : this.mapViewer,
                    serverName : '/move_base',
                    //image: '../res/mirobot.png',
                    withOrientation: true,
                    robot_pose: '/robot_pose',
                })

                this.nav_cancel_topic = new ROSLIB.Topic({
                    ros : this.ros,
                    name : '/move_base/cancel',
                    messageType : 'actionlib_msgs/GoalID'
                  });
            })
            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                this.connected = false
                this.loading = false
                document.getElementById('map').innerHTML = ''
            })


            // Get the URDF value from ROS.
            var param = new ROSLIB.Param({
                ros : this.ros,
                name : 'robot_description'
            });
  
            param.get(function(param) {
                // Setup the loader for the URDF.
                var urdfModel = new ROSLIB.UrdfModel({
                string : param
                });
  
            console.log(urdfModel);
            });


        },
        stop_nav: function() {
            var cancel_goal = new ROSLIB.Message({
                stamp : {
                  secs : 0,
                  nsecs : 0
                },
                id : ""
                });
                this.nav_cancel_topic.publish(cancel_goal);
        },
        disconnect: function() {
            this.ros.close()
        },
    },
    mounted() {
        this.interval = setInterval(() => {
            if (this.ros != null && this.ros.isConnected) {
                this.ros.getNodes((data) => { }, (error) => { })
            }
        }, 10000)
    },
})
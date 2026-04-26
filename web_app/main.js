var app = new Vue({
    el: '#app',

    data: {
        connected: false,
        ros: null,
        loading: false,

        rosbridge_address: 'ws://localhost:9090',
        selectedMode: 'simulation',

        logs: [],
        nav2Logs: [],
        missionLogs: [],

        showSystemLogs: false,
        showNav2Logs: false,
        showMissionLogs: false,

        heartbeatInterval: null,

        nav2Running: false,
        localizationDone: false,
        missionRunning: false,
        missionTriggered: false,

        navGoal: {
            x: '',
            y: '',
            yaw: ''
        },

        position: {
            x: '--',
            y: '--',
            yaw: '--'
        },

        velocity: {
            linear: '--',
            angular: '--'
        },

        odomTopicName: '/odom',
        cmdVelTopicName: '/diffbot_base_controller/cmd_vel_unstamped',

        nav2StatusTopic: null,
        missionStatusTopic: null,
        nav2LogTopic: null,
        missionLogTopic: null,
        navToPoseTopic: null,
        odomTopic: null,
        cmdVelTopic: null,

        elevatorDirection: 'up'
    },

    computed: {
        canStartMission() {
            return this.connected && this.nav2Running && this.localizationDone;
        }
    },

    methods: {
        // ==================================================
        // Logs
        // ==================================================
        addLog(message) {
            const timestamp = new Date().toLocaleTimeString();
            this.logs.unshift(`[${timestamp}] ${message}`);

            if (this.logs.length > 300) {
                this.logs.pop();
            }
        },

        addNav2Log(message) {
            const timestamp = new Date().toLocaleTimeString();
            this.nav2Logs.unshift(`[${timestamp}] ${message}`);

            if (this.nav2Logs.length > 500) {
                this.nav2Logs.pop();
            }
        },

        addMissionLog(message) {
            const timestamp = new Date().toLocaleTimeString();
            this.missionLogs.unshift(`[${timestamp}] ${message}`);

            if (this.missionLogs.length > 500) {
                this.missionLogs.pop();
            }
        },

        formatNumber(value, digits = 2) {
            if (value === null || value === undefined || isNaN(value)) {
                return '--';
            }

            return Number(value).toFixed(digits);
        },

        quaternionToYaw(q) {
            const sinyCosp = 2.0 * (q.w * q.z + q.x * q.y);
            const cosyCosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

            return Math.atan2(sinyCosp, cosyCosp);
        },

        // ==================================================
        // ROSBridge connection
        // ==================================================
        connect() {
            if (!this.rosbridge_address) {
                this.addLog('Please enter a ROSBridge address.');
                return;
            }

            this.loading = true;
            this.addLog(`Trying to connect to ${this.rosbridge_address}`);

            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address,
                groovyCompatibility: false
            });

            this.ros.on('connection', () => {
                this.connected = true;
                this.loading = false;

                this.addLog('Connected to ROSBridge.');

                this.setupWebTopics();
                this.setupRobotTopics();
                this.startHeartbeat();
            });

            this.ros.on('error', (error) => {
                this.loading = false;
                this.addLog('ROSBridge connection error.');
                console.error(error);
            });

            this.ros.on('close', () => {
                this.connected = false;
                this.loading = false;

                this.addLog('Disconnected from ROSBridge.');

                this.stopHeartbeat();
                this.cleanupWebTopics();
                this.cleanupRobotTopics();

                this.ros = null;

                this.nav2Running = false;
                this.localizationDone = false;
                this.missionRunning = false;
                this.missionTriggered = false;

                this.resetRobotStatus();
            });
        },

        disconnect() {
            if (this.ros) {
                this.addLog('Closing ROSBridge connection...');
                this.ros.close();
            }
        },

        startHeartbeat() {
            this.stopHeartbeat();

            this.heartbeatInterval = setInterval(() => {
                if (this.ros && this.ros.isConnected) {
                    this.ros.getNodes(
                        (nodes) => {
                            this.addLog(`Heartbeat OK. ROS nodes available: ${nodes.length}`);
                        },
                        (error) => {
                            this.addLog('Heartbeat failed.');
                            console.error(error);
                        }
                    );
                }
            }, 10000);
        },

        stopHeartbeat() {
            if (this.heartbeatInterval) {
                clearInterval(this.heartbeatInterval);
                this.heartbeatInterval = null;
            }
        },

        // ==================================================
        // Topics from web process manager
        // ==================================================
        setupWebTopics() {
            this.cleanupWebTopics();

            this.nav2StatusTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/web/status/nav2',
                messageType: 'std_msgs/String'
            });

            this.missionStatusTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/web/status/mission',
                messageType: 'std_msgs/String'
            });

            this.nav2LogTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/web/log/nav2',
                messageType: 'std_msgs/String'
            });

            this.missionLogTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/web/log/mission',
                messageType: 'std_msgs/String'
            });

            this.navToPoseTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/web/nav_to_pose',
                messageType: 'geometry_msgs/PoseStamped'
            });

            this.nav2StatusTopic.subscribe((msg) => {
                this.nav2Running = msg.data === 'running';

                if (!this.nav2Running) {
                    this.localizationDone = false;
                    this.missionRunning = false;
                    this.missionTriggered = false;
                }
            });

            this.missionStatusTopic.subscribe((msg) => {
                this.missionRunning = msg.data === 'running';

                if (!this.missionRunning) {
                    this.missionTriggered = false;
                }
            });

            this.nav2LogTopic.subscribe((msg) => {
                this.addNav2Log(msg.data);
            });

            this.missionLogTopic.subscribe((msg) => {
                this.addMissionLog(msg.data);
            });

            this.addLog('Subscribed to web status and log topics.');
        },

        cleanupWebTopics() {
            if (this.nav2StatusTopic) {
                this.nav2StatusTopic.unsubscribe();
                this.nav2StatusTopic = null;
            }

            if (this.missionStatusTopic) {
                this.missionStatusTopic.unsubscribe();
                this.missionStatusTopic = null;
            }

            if (this.nav2LogTopic) {
                this.nav2LogTopic.unsubscribe();
                this.nav2LogTopic = null;
            }

            if (this.missionLogTopic) {
                this.missionLogTopic.unsubscribe();
                this.missionLogTopic = null;
            }

            this.navToPoseTopic = null;
        },

        // ==================================================
        // Robot topics: odom + cmd_vel
        // ==================================================
        setupRobotTopics() {
            this.cleanupRobotTopics();

            this.odomTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: this.odomTopicName,
                messageType: 'nav_msgs/Odometry'
            });

            this.odomTopic.subscribe(this.handleOdomMessage);

            this.cmdVelTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: this.cmdVelTopicName,
                messageType: 'geometry_msgs/Twist'
            });

            this.addLog(`Subscribed to odom topic: ${this.odomTopicName}`);
            this.addLog(`Prepared cmd_vel topic: ${this.cmdVelTopicName}`);
        },

        cleanupRobotTopics() {
            if (this.odomTopic) {
                this.odomTopic.unsubscribe();
                this.odomTopic = null;
            }

            this.cmdVelTopic = null;
        },

        resetRobotStatus() {
            this.position.x = '--';
            this.position.y = '--';
            this.position.yaw = '--';

            this.velocity.linear = '--';
            this.velocity.angular = '--';
        },

        handleOdomMessage(msg) {
            const px = msg.pose.pose.position.x;
            const py = msg.pose.pose.position.y;
            const yaw = this.quaternionToYaw(msg.pose.pose.orientation);

            const linear = msg.twist.twist.linear.x;
            const angular = msg.twist.twist.angular.z;

            this.position.x = this.formatNumber(px, 2);
            this.position.y = this.formatNumber(py, 2);
            this.position.yaw = this.formatNumber(yaw, 2);

            this.velocity.linear = this.formatNumber(linear, 2);
            this.velocity.angular = this.formatNumber(angular, 2);
        },

        // ==================================================
        // Service calls
        // ==================================================
        callTriggerService(serviceName, label, onSuccess) {
            if (!this.connected || !this.ros) {
                this.addLog('Cannot call service. ROSBridge is not connected.');
                return;
            }

            this.addLog(`Calling service: ${serviceName}`);

            const service = new ROSLIB.Service({
                ros: this.ros,
                name: serviceName,
                serviceType: 'std_srvs/srv/Trigger'
            });

            const request = new ROSLIB.ServiceRequest({});

            service.callService(
                request,
                (response) => {
                    if (response.success) {
                        this.addLog(`${label}: ${response.message || 'success'}`);

                        if (onSuccess) {
                            onSuccess(response);
                        }
                    } else {
                        this.addLog(`${label}: failed - ${response.message || 'no message'}`);
                    }
                },
                (error) => {
                    this.addLog(`${label}: service call error`);
                    console.error(error);
                }
            );
        },

        getServiceName(commandName) {
            const services = {
                simulation: {
                    nav2: '/web/start_nav2_stack',
                    localization: '/web/start_self_localizationunstamped',
                    mission: '/web/start_bt_mission'
                },

                real: {
                    nav2: '/web/start_nav2_stack_real',
                    localization: '/web/start_self_localization',
                    mission: '/web/start_bt_mission_real'
                }
            };

            return services[this.selectedMode][commandName];
        },

        getModeLabel() {
            return this.selectedMode === 'real' ? 'Real robot' : 'Simulation';
        },

        startNav2Stack() {
            const serviceName = this.getServiceName('nav2');

            this.callTriggerService(
                serviceName,
                `Start Nav2 Stack - ${this.getModeLabel()}`,
                () => {
                    this.nav2Running = true;
                    this.localizationDone = false;
                    this.showNav2Logs = true;
                }
            );
        },

        startSelfLocalization() {
            if (!this.nav2Running) {
                this.addLog('Cannot localize. Start Nav2 Stack first.');
                return;
            }

            const serviceName = this.getServiceName('localization');

            this.callTriggerService(
                serviceName,
                `Start Self Localization - ${this.getModeLabel()}`,
                () => {
                    this.localizationDone = true;
                    this.addLog('Robot marked as localized.');
                    this.addNav2Log('Robot marked as localized from web panel.');
                }
            );
        },

        startBtMission() {
            if (!this.canStartMission) {
                this.addLog('Cannot start mission. Nav2 must be running and robot must be localized.');
                return;
            }

            const serviceName = this.getServiceName('mission');

            this.callTriggerService(
                serviceName,
                `Run BT Mission - ${this.getModeLabel()}`,
                () => {
                    this.missionRunning = true;
                    this.missionTriggered = false;
                    this.showMissionLogs = true;
                }
            );
        },

        triggerOrAbortMission() {
            if (!this.missionRunning) {
                this.addLog('Mission launch is not running yet.');
                return;
            }

            if (this.missionTriggered) {
                this.callTriggerService(
                    '/web/stop_bt_mission',
                    'Abort Mission',
                    () => {
                        this.missionTriggered = false;
                        this.missionRunning = false;
                    }
                );

                return;
            }

            this.callTriggerService(
                '/start_shelf_mission',
                'Trigger Shelf Mission',
                () => {
                    this.missionTriggered = true;
                }
            );
        },

        // ==================================================
        // Nav To Pose
        // ==================================================
        sendNavToPose() {
            if (!this.nav2Running) {
                this.addLog('Nav2 stack is not running.');
                return;
            }

            if (!this.navToPoseTopic) {
                this.addLog('Nav To Pose topic is not ready.');
                return;
            }

            const x = Number(this.navGoal.x);
            const y = Number(this.navGoal.y);
            const yaw = Number(this.navGoal.yaw);

            if (isNaN(x) || isNaN(y) || isNaN(yaw)) {
                this.addLog('Nav To Pose failed. X, Y and Yaw must be numbers.');
                return;
            }

            const qz = Math.sin(yaw / 2.0);
            const qw = Math.cos(yaw / 2.0);

            const message = new ROSLIB.Message({
                header: {
                    frame_id: 'map'
                },
                pose: {
                    position: {
                        x: x,
                        y: y,
                        z: 0.0
                    },
                    orientation: {
                        x: 0.0,
                        y: 0.0,
                        z: qz,
                        w: qw
                    }
                }
            });

            this.navToPoseTopic.publish(message);

            this.addLog(`Nav To Pose sent: x=${x}, y=${y}, yaw=${yaw}`);
            this.addNav2Log(`Nav To Pose sent: x=${x}, y=${y}, yaw=${yaw}`);
        },

        // ==================================================
        // Joystick
        // ==================================================
        sendJoystickCommand(linearX, angularZ) {
            if (!this.connected || !this.cmdVelTopic) {
                this.addLog('Cannot send joystick command. cmd_vel topic is not ready.');
                return;
            }

            const message = new ROSLIB.Message({
                linear: {
                    x: linearX,
                    y: 0.0,
                    z: 0.0
                },
                angular: {
                    x: 0.0,
                    y: 0.0,
                    z: angularZ
                }
            });

            this.cmdVelTopic.publish(message);

            this.addNav2Log(`Joystick cmd_vel: linear.x=${linearX}, angular.z=${angularZ}`);
        },

        // ==================================================
        // Elevator
        // ==================================================
        toggleElevator() {
            if (!this.connected || !this.ros) {
                this.addLog('Cannot publish elevator command. ROSBridge is not connected.');
                return;
            }

            if (this.elevatorDirection === 'up') {
                this.publishElevatorBurst('/elevator_up', 'up');
                this.elevatorDirection = 'down';
            } else {
                this.publishElevatorBurst('/elevator_down', 'down');
                this.elevatorDirection = 'up';
            }
        },

        publishElevatorBurst(topicName, direction) {
            const topic = new ROSLIB.Topic({
                ros: this.ros,
                name: topicName,
                messageType: 'std_msgs/String'
            });

            const message = new ROSLIB.Message({
                data: ''
            });

            let count = 0;
            const maxCount = 5;

            const timer = setInterval(() => {
                topic.publish(message);
                count += 1;

                if (count >= maxCount) {
                    clearInterval(timer);
                    this.addMissionLog(`Elevator ${direction.toUpperCase()} command published ${maxCount} times on ${topicName}`);
                    this.addLog(`Elevator ${direction.toUpperCase()} command done.`);
                }
            }, 200);
        }
    },

    beforeDestroy() {
        this.stopHeartbeat();
        this.cleanupWebTopics();
        this.cleanupRobotTopics();

        if (this.ros) {
            this.ros.close();
        }
    }
});
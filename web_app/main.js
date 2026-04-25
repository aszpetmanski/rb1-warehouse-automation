var app = new Vue({
    el: '#app',

    data: {
        connected: false,
        ros: null,
        logs: [],
        nav2Logs: [],
        missionLogs: [],
        loading: false,

        rosbridge_address: 'ws://localhost:9090',

        heartbeatInterval: null,

        selectedMode: 'simulation',

        nav2Running: false,
        missionRunning: false,
        missionTriggered: false,

        navGoal: {
            x: '',
            y: '',
            yaw: ''
        },

        nav2StatusTopic: null,
        missionStatusTopic: null,
        nav2LogTopic: null,
        missionLogTopic: null,
        navToPoseTopic: null,

        elevatorDirection: 'up',
        elevatorTopic: null,
        elevatorTimer: null
    },

    methods: {
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
                this.stopElevatorPublisher();

                this.ros = null;
                this.nav2Running = false;
                this.missionRunning = false;
                this.missionTriggered = false;
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
                    localization: '/web/start_self_localization_unstamped',
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
                }
            );
        },

        startSelfLocalization() {
            const serviceName = this.getServiceName('localization');

            this.callTriggerService(
                serviceName,
                `Start Self Localization - ${this.getModeLabel()}`
            );
        },

        startBtMission() {
            const serviceName = this.getServiceName('mission');

            this.callTriggerService(
                serviceName,
                `Run BT Mission - ${this.getModeLabel()}`,
                () => {
                    this.missionRunning = true;
                    this.missionTriggered = false;
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
        },

        toggleElevator() {
            if (!this.connected || !this.ros) {
                this.addLog('Cannot publish elevator command. ROSBridge is not connected.');
                return;
            }

            if (this.elevatorDirection === 'up') {
                this.startElevatorPublisher('/elevator_up', 'up');
                this.elevatorDirection = 'down';
            } else {
                this.startElevatorPublisher('/elevator_down', 'down');
                this.elevatorDirection = 'up';
            }
        },

        startElevatorPublisher(topicName, direction) {
            this.stopElevatorPublisher();

            this.elevatorTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: topicName,
                messageType: 'std_msgs/String'
            });

            const message = new ROSLIB.Message({
                data: ''
            });

            this.elevatorTopic.publish(message);

            this.elevatorTimer = setInterval(() => {
                if (this.connected && this.elevatorTopic) {
                    this.elevatorTopic.publish(message);
                }
            }, 200);

            this.addLog(`Elevator ${direction.toUpperCase()} publishing at 5 Hz on ${topicName}`);
        },

        stopElevatorPublisher() {
            if (this.elevatorTimer) {
                clearInterval(this.elevatorTimer);
                this.elevatorTimer = null;
            }

            this.elevatorTopic = null;
        }
    },

    beforeDestroy() {
        this.stopHeartbeat();
        this.cleanupWebTopics();
        this.stopElevatorPublisher();

        if (this.ros) {
            this.ros.close();
        }
    }
});
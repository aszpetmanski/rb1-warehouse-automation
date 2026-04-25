var app = new Vue({
    el: '#app',

    data: {
        connected: false,
        ros: null,
        logs: [],
        loading: false,

        rosbridge_address: 'ws://localhost:9090',

        heartbeatInterval: null,

        selectedMode: 'simulation'
},

    methods: {
        addLog(message) {
            const timestamp = new Date().toLocaleTimeString();
            this.logs.unshift(`[${timestamp}] ${message}`);
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
                this.ros = null;
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

        getServiceName(commandName) {
    const services = {
        simulation: {
            nav2: '/web/start_nav2_stack',
            localization: '/web/start_self_localization',
            mission: '/web/start_bt_mission'
        },

        real: {
            nav2: '/web/start_nav2_stack_real',
            localization: '/web/start_self_localization_unstamped',
            mission: '/web/start_bt_mission_real'
        }
    };

    return services[this.selectedMode][commandName];
},

        getModeLabel() {
            if (this.selectedMode === 'real') {
                return 'Real robot';
            }

            return 'Simulation';
        },

        startNav2Stack() {
            const serviceName = this.getServiceName('nav2');
            this.callTriggerService(serviceName, `Start Nav2 Stack - ${this.getModeLabel()}`);
        },

        startSelfLocalization() {
            const serviceName = this.getServiceName('localization');
            this.callTriggerService(serviceName, `Start Self Localization - ${this.getModeLabel()}`);
        },

        startBtMission() {
            const serviceName = this.getServiceName('mission');
            this.callTriggerService(serviceName, `Run BT Mission - ${this.getModeLabel()}`);
        },

        triggerMission() {
            this.callTriggerService('/start_shelf_mission', 'Trigger Shelf Mission');
        },

        callTriggerService(serviceName, label) {
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
                    } else {
                        this.addLog(`${label}: failed - ${response.message || 'no message'}`);
                    }
                },
                (error) => {
                    this.addLog(`${label}: service call error`);
                    console.error(error);
                }
            );
        }
    },

    beforeDestroy() {
        this.stopHeartbeat();

        if (this.ros) {
            this.ros.close();
        }
    }
});
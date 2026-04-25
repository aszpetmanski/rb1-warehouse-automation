var app = new Vue({
    el: '#app',

    data: {
        connected: false,
        ros: null,
        logs: [],
        loading: false,

        rosbridge_address: 'ws://localhost:9090',

        heartbeatInterval: null
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
        }
    },

    beforeDestroy() {
        this.stopHeartbeat();

        if (this.ros) {
            this.ros.close();
        }
    }
});
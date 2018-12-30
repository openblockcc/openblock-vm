const ROSLIB = require('roslib');

class RosUtil extends ROSLIB.Ros {
    constructor (runtime, extensionId, options) {
        super(options);

        this.runtime = runtime;
        this.extensionId = extensionId;
        this.everConnected = false;

        this.on('connection', () => {
            this.everConnected = true;
            this.runtime.emit(this.runtime.constructor.PERIPHERAL_CONNECTED);
        });

        this.on('close', () => {
            if (this.everConnected) {
                this.runtime.emit(this.runtime.constructor.PERIPHERAL_DISCONNECT_ERROR, {
                    message: `Scratch lost connection to`,
                    extensionId: this.extensionId
                });
            }
        });

        this.on('error', () => {
            this.runtime.emit(this.runtime.constructor.PERIPHERAL_REQUEST_ERROR, {
                message: `Scratch lost connection to`,
                extensionId: this.extensionId
            });
        });
    }

    getTopic (name) {
        const ros = this;
        if (name && !name.startsWith('/')) name = `/${name}`;
        return new Promise(resolve => {
            ros.getTopicType(
                name,
                type =>
                    resolve(new ROSLIB.Topic({
                        ros: ros,
                        name: name,
                        messageType: type
                    })));
        });
    }

    getService (name) {
        const ros = this;
        return new Promise(resolve => {
            ros.getServiceType(
                name,
                type =>
                    resolve(new ROSLIB.Service({
                        ros: ros,
                        name: name,
                        serviceType: type
                    })));
        });
    }

    getParam (name) {
        return this.Param({
            ros: this,
            name: name
        });
    }

    getRosType (val) {
        switch (typeof val) {
        case 'boolean':
            return 'std_msgs/Bool';
        case 'number':
            return (val % 1 === 0) ? 'std_msgs/Int32' : 'std_msgs/Float64';
        default:
            return 'std_msgs/String';
        }
    }
}

module.exports = RosUtil;

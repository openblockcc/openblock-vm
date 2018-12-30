const ROSLIB = require('roslib');

class RosUtil extends ROSLIB.Ros {
    constructor (options) {
        super(options);

        this.on('close', function () {
            const msg = `
            ** Error connecting to ROS!! **

Make sure to enable connections with:
    roslaunch rosbridge_server rosbridge_websocket.launch

Click 'ok' to reconnect.`;

            if (confirm(msg)) this.connect(options.url);
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

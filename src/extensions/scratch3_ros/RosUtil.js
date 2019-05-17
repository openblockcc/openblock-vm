const math = require('mathjs');
const JSON = require('circular-json');
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
                this.runtime.emit(this.runtime.constructor.PERIPHERAL_CONNECTION_LOST_ERROR, {
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

    publishTopic (name, msg) {
        return new Promise(resolve => {
            this.getTopic(name).then(rosTopic => {
                if (!rosTopic.messageType) resolve();
                rosTopic.publish(msg);
                resolve(true)
            });
        });
    }

    subscribeTopic (name, callback, unsubscribe=true) {
        return new Promise(resolve => {
            this.getTopic(name).then(rosTopic => {
                if (!rosTopic.messageType) resolve();
                rosTopic.subscribe(msg => {
                    if (unsubscribe) rosTopic.unsubscribe();
                    resolve(callback(msg));
                });
            });
        });
    }

    callService (name, req) {
        return new Promise(resolve => {
            this.getService(name).then(rosService => {
                if (!rosService.serviceType) resolve();
                rosService.callService(req,
                    res => {
                        rosService.unadvertise();
                        resolve(res);
                    });
            });
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

class Scratch3RosBase {

    constructor (extensionName, extensionId, runtime) {
        this.extensionName = extensionName;
        this.extensionId = extensionId;
        this.runtime = runtime;

        this.runtime.registerPeripheralExtension(this.extensionId, this);

        math.config({matrix: 'Array'});

        this.topicNames = ['/topic'];
        this.serviceNames = ['/service'];
        this.paramNames = ['/param'];
    }

    // Peripheral connection functions
    scan () {
        // Not going to really 'scan' anything
        // When running from github.io, only connections to localhost can be safely established
        // Otherwise we need to use WebSocketSecure or host our own http site
        this.connect('ws://localhost:9090');
    }

    connect (url) {
        this.ros = new RosUtil(this.runtime, this.extensionId, {url: url});
    }

    disconnect () {
        this.ros.socket.close();
    }

    isConnected () {
        if (this.ros) return this.ros.isConnected;
        return false;
    }

    // JSON utility
    _isJSON (value) {
        return value && typeof value === 'object' && value.constructor === Object;
    }

    _tryParse (value, reject) {
        if (typeof value !== 'string') return value;
        try {
            return JSON.parse(value);
        } catch (err) {
            return reject;
        }
    }

    // Variable utility
    _getVariableValue (variable, type) {
        if (typeof variable === 'string') {
            variable = this.runtime.getEditingTarget().lookupVariableByNameAndType(variable, type);
        }
        return variable && this._tryParse(variable.value, variable.value);
    }

    _changeVariableVisibility ({VAR, SLOT}, visible) {
        const target = this.runtime.getEditingTarget();
        const variable = target.lookupVariableByNameAndType(VAR);
        const id = variable && `${variable.id}${VAR}.${SLOT}`;
        if (!id) return;

        if (visible && !(this.runtime.monitorBlocks._blocks[id])) {
            const isLocal = !(this.runtime.getTargetForStage().variables[variable.id]);
            const targetId = isLocal ? target.id : null;
            this.runtime.monitorBlocks.createBlock({
                id: id,
                targetId: targetId,
                opcode: 'ros_getSlot',
                fields: {OBJECT: {value: VAR}, SLOT: {value: SLOT}}
            });
        }

        this.runtime.monitorBlocks.changeBlock({
            id: id,
            element: 'checkbox',
            value: visible
        }, this.runtime);
    }

    // Dynamic menus
    _updateTopicList () {
        if (this.ros) {
            const that = this;
            this.ros.getTopics(topics => {
                that.topicNames = topics.topics.sort();
            });
        }
        return this.topicNames.map(val => ({value: val, text: val}));
    }

    _updateServiceList () {
        if (this.ros) {
            const that = this;
            this.ros.getServices(services => {
                that.serviceNames = services.sort();
            });
        }
        return this.serviceNames.map(val => ({value: val, text: val}));
    }

    _updateParamList () {
        if (this.ros) {
            const that = this;
            this.ros.getParams(params => {
                that.paramNames = params.sort();
            });
        }
        return this.paramNames.map(val => ({value: val, text: val}));
    }

    _updateVariableList () {
        let varlist;
        try {
            varlist = this.runtime.getEditingTarget().getAllVariableNamesInScopeByType();
        } catch (err) {
            return [{value: 'my variable', text: 'my variable'}];
        }

        if (varlist.length === 0) return [{value: 'my variable', text: 'my variable'}];
        return varlist.map(val => ({value: val, text: val}));
    }

    // TODO: allow returning Promises for dynamic menus or update them periodically
}

module.exports = Scratch3RosBase;

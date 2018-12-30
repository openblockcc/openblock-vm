const JSON = require('circular-json');
const BlockType = require('../../extension-support/block-type');
const ArgumentType = require('../../extension-support/argument-type');
const RosUtil = require('./RosUtil');

const icon = 'data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAADTUlEQVRIS63VTWgUZxgH8P+7Mzsf+2U2SrPJKBoEA1WKUMzupaKHpgoetadCnaBN9FRaLLRQEW17KKIed1V21dJLbU/FRqiixx219NDaVi+NmnWNZT+cze7Ozsf7yoxEYpnMRJI5DQzP+5v3P8/7DEHANVHM7iXAVyAkwQCNgZYjDp3KH7jzZ1DdwmckBLirrE29KQgcTNOBZTrQdYP2es631LSPnp34zQqDlgTIcvTlOowxNBsG6o3uHw6he899eOt+EBIGeBHxUW5ElDjEYwKSKdFbzzAsVB61/q4lWlsvv3/XXAwJBOaL9pe29slE3MgoOSHF+N0DAwnwfASNehe1WudkYVw7sixgYfFkMXtSikU/VZQU3LhmHjVZ17Sz59Tbt/2QJe3glUIGMnEhe3PgjcR2Ny5dN/Df0873ebX8gS/wshUBUMK+PKveuhzWGZPF0fdiceHq4FAK3a6FyoyuFca13GKA14ruw8qM/ldhXNscBnz03duDUSY83jCchu04ePBvs5ZXtTW+wGQpW12/IZ1xP9r0dAM9g2bOH9Rmg5DDpW0ZwvFVD7CpW1ctqNqQP1DM/ZJRErvdFqw+bqHdMccKqvZr4AEsbdsVj4lTbkTttolqtXWloGp7FtlB7uv+1fIX6bSM9pyJJ9W5a3m1PAYC5ou4H7mUvT44mNwZTwio17to1Dsn8qp21Bc4cGl0WCLcP+vW9wkRQrxddNrW6cyw9NmxnTfthUX7ftgs9LcTP8dkfmxIWeW16cOHTdO22EheLU8v2qYTxezn6bT8zeo1MS/T2dk5GB3rHgNOMd6+SsFxnE3eJcAnohwdUZQkQAjqtTYadePHwri2L/CgHbuxg3/ywPhdUZJbJOnF3GnpPS9fw7ABAkgij3h8flS46REYPRvVig5KoebV8oXAg3bw4ugmjkV+6u+Xt/T1SSDE/wy6sTxrdiHHBYgCH4q8ssqLjOPHZYk/kkiKEUHg4Y5qBgbLpDBNG62W5Rhde4rjsGdISUEUgxHf1zx0MfsWdTBGCHIAeQeABbAyBdPmfziTpdz+SASlMOT1Z9GCoJeCLAtwrTBk2UAYsiKAH+JN2YreXDHg/wilzJ3Oz1YUmEcYY2fce0LIx88BFi6vvp70RPYAAAAASUVORK5CYII=';

class Scratch3RosBase {
    constructor (runtime) {
        // When running from github.io, only connections to localhost can be safely established
        this.ros = new RosUtil({url: 'ws://localhost:9090'});
        this.topicNames = ['/topic'];
        this.serviceNames = ['/service'];
        this.paramNames = ['/param'];
        this.runtime = runtime;
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
        const that = this;
        that.ros.getTopics(topics => {
            that.topicNames = topics.topics.sort();
        });
        return that.topicNames.map(val => ({value: val, text: val}));
    }

    _updateServiceList () {
        const that = this;
        that.ros.getServices(services => {
            that.serviceNames = services.sort();
        });
        return that.serviceNames.map(val => ({value: val, text: val}));
    }

    _updateParamList () {
        const that = this;
        that.ros.getParams(params => {
            that.paramNames = params.sort();
        });
        return that.paramNames.map(val => ({value: val, text: val}));
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

    // TODO: allow returning Promises for dynamic menus
}

class Scratch3RosBlocks extends Scratch3RosBase {
    subscribeTopic ({TOPIC}) {
        const that = this;
        return new Promise(resolve => {
            that.ros.getTopic(TOPIC).then(
                rosTopic => {
                    if (!rosTopic.messageType) resolve();
                    rosTopic.subscribe(msg => {
                        // rosTopic.unsubscribe();
                        if (rosTopic.messageType === 'std_msgs/String') {
                            msg.data = that._tryParse(msg.data);
                        }
                        msg.toString = function () { return JSON.stringify(this); };
                        msg.constructor = Object;
                        resolve(msg);
                    });
                });
        });
    }

    publishTopic ({MSG, TOPIC}) {
        const ROS = this.ros;
        let msg = this._getVariableValue(MSG);
        if (msg === null || typeof msg === 'undefined') msg = this._tryParse(MSG);
        if (!this._isJSON(msg)) msg = {data: msg};

        ROS.getTopic(TOPIC).then(rosTopic => {
            if (!rosTopic.name) return;
            const keys = Object.keys(msg);
            if (rosTopic.messageType) {
                if (rosTopic.messageType === 'std_msgs/String' &&
                    !(keys.length === 1 && keys[0] === 'data')) {
                    msg = {data: JSON.stringify(msg)};
                }
            } else {
                if (!(keys.length === 1 && keys[0] === 'data')) {
                    msg = {data: JSON.stringify(msg)};
                }
                rosTopic.messageType = ROS.getRosType(msg.data);
            }
            rosTopic.publish(msg);
        });
    }

    callService ({REQUEST, SERVICE}) {
        const ROS = this.ros;
        const req = this._getVariableValue(REQUEST) || this._tryParse(REQUEST);

        return new Promise(resolve => {
            ROS.getService(SERVICE).then(rosService =>
                rosService.callService(req,
                    res => {
                        rosService.unadvertise();
                        resolve(res);
                    }));
        });
    }

    getParamValue ({NAME}) {
        const that = this;
        return new Promise(resolve => {
            const param = that.ros.getParam(NAME);
            param.get(val => {
                if (that._isJSON(val)) {
                    val.toString = function () { return JSON.stringify(this); };
                }
                resolve(val);
            });
        });
    }

    setParamValue ({NAME, VALUE}) {
        const param = this.ros.getParam(NAME);
        param.set(this._tryParse(VALUE, VALUE));
    }

    getSlot ({OBJECT, SLOT}, util) {
        const evalSlot = function (obj, slots) {
            const slotArr = slots.split(/\.|\[|\]/).filter(Boolean);
            for (let i = 0; i < slotArr.length; i++) {
                if (!obj) return;
                obj = obj[slotArr[i]];
            }
            return obj;
        };

        const variable = util.target.lookupVariableByNameAndType(OBJECT);
        const obj = this._getVariableValue(variable) || this._tryParse(OBJECT);
        const res = (this._isJSON(obj) || void 0) && evalSlot(obj, SLOT);
        if (util.thread.updateMonitor) {
            if (typeof res === 'undefined') {
                const name = `${OBJECT}.${SLOT}`;
                const id = variable ?
                    variable.id + name :
                    Object.keys(util.runtime.monitorBlocks._blocks)
                        .find(key => key.search(name) >= 0);

                util.runtime.monitorBlocks.deleteBlock(id);
                util.runtime.requestRemoveMonitor(id);
            } else return JSON.stringify(res);
        }
        if (this._isJSON(res)) res.toString = function () { return JSON.stringify(this); };
        return res;
    }

    setSlot ({OBJECT, SLOT, VALUE}, util) {
        const setNestedValue = function (obj, slots, value) {
            const last = slots.length - 1;
            for (let i = 0; i < last; i++) {
                obj = obj[ slots[i] ] = obj[ slots[i] ] || {};
            }
            obj = obj[slots[last]] = value;
        };

        const variable = util.target.lookupVariableByNameAndType(OBJECT);
        if (!variable) return;
        const variableValue = this._getVariableValue(variable);

        if (this._isJSON(variableValue)) {
            // Clone object to avoid overwriting parent variables
            variable.value = JSON.parse(JSON.stringify(variableValue));
        } else variable.value = {};
        const slt = SLOT.split('.');
        const val = Array.isArray(VALUE) ?
            VALUE.map(v => this._tryParse(v, v)) :
            this._tryParse(VALUE, VALUE);

        setNestedValue(variable.value, slt, val);

        // TODO: cloud variables
    }

    showVariable (args) {
        this._changeVariableVisibility(args, true);
    }

    hideVariable (args) {
        this._changeVariableVisibility(args, false);
    }

    getInfo () {
        const stringArg = defValue => ({
            type: ArgumentType.STRING,
            defaultValue: defValue
        });
        const variableArg = {
            type: ArgumentType.STRING,
            menu: 'variablesMenu',
            defaultValue: this._updateVariableList()[0].text
        };
        const topicArg = {
            type: ArgumentType.STRING,
            menu: 'topicsMenu',
            defaultValue: this.topicNames[0]
        };
        const serviceArg = {
            type: ArgumentType.STRING,
            menu: 'servicesMenu',
            defaultValue: this.serviceNames[0]
        };
        const paramArg = {
            type: ArgumentType.STRING,
            menu: 'paramsMenu',
            defaultValue: this._updateParamList()[0].text
        };

        return {
            id: 'ros',
            name: 'ROS',

            colour: '#8BC34A',
            colourSecondary: '#7CB342',
            colourTertiary: '#689F38',

            menuIconURI: icon,

            blocks: [
                {
                    opcode: 'subscribeTopic',
                    blockType: BlockType.REPORTER,
                    text: 'Get message from [TOPIC]',
                    arguments: {
                        TOPIC: topicArg
                    }
                },
                {
                    opcode: 'publishTopic',
                    blockType: BlockType.COMMAND,
                    text: 'Publish [MSG] to [TOPIC]',
                    arguments: {
                        MSG: variableArg,
                        TOPIC: topicArg
                    }
                },
                '---',
                {
                    opcode: 'callService',
                    blockType: BlockType.REPORTER,
                    text: 'Send [REQUEST] to [SERVICE]',
                    arguments: {
                        REQUEST: variableArg,
                        SERVICE: serviceArg
                    }
                },
                '---',
                {
                    opcode: 'getParamValue',
                    blockType: BlockType.REPORTER,
                    text: 'Get rosparam [NAME]',
                    arguments: {
                        NAME: paramArg
                    }
                },
                {
                    opcode: 'setParamValue',
                    blockType: BlockType.COMMAND,
                    text: 'Set rosparam [NAME] to [VALUE]',
                    arguments: {
                        NAME: paramArg,
                        VALUE: stringArg(0)
                    }
                },
                '---',
                {
                    opcode: 'getSlot',
                    blockType: BlockType.REPORTER,
                    text: 'Get [OBJECT] [SLOT]',
                    arguments: {
                        OBJECT: variableArg,
                        SLOT: stringArg('data')
                    }
                },
                {
                    opcode: 'setSlot',
                    blockType: BlockType.COMMAND,
                    text: 'Set [OBJECT] [SLOT] to [VALUE]',
                    arguments: {
                        OBJECT: variableArg,
                        SLOT: stringArg('data'),
                        VALUE: stringArg('Hello!')
                    }
                },
                {
                    opcode: 'showVariable',
                    blockType: BlockType.COMMAND,
                    text: 'Show [VAR] [SLOT]',
                    arguments: {
                        VAR: variableArg,
                        SLOT: stringArg('data')
                    }
                },
                {
                    opcode: 'hideVariable',
                    blockType: BlockType.COMMAND,
                    text: 'Hide [VAR] [SLOT]',
                    arguments: {
                        VAR: variableArg,
                        SLOT: stringArg('data')
                    }
                }
            ],
            menus: {
                topicsMenu: '_updateTopicList',
                servicesMenu: '_updateServiceList',
                variablesMenu: '_updateVariableList',
                paramsMenu: '_updateParamList'
            }
        };
    }
}

module.exports = Scratch3RosBlocks;

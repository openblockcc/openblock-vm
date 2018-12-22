const ArgumentType = require('../../extension-support/argument-type');
const BlockType = require('../../extension-support/block-type');
const formatMessage = require('format-message');
const RosUtil = require('./RosUtil');

const icon = "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAADTUlEQVRIS63VTWgUZxgH8P+7Mzsf+2U2SrPJKBoEA1WKUMzupaKHpgoetadCnaBN9FRaLLRQEW17KKIed1V21dJLbU/FRqiixx219NDaVi+NmnWNZT+cze7Ozsf7yoxEYpnMRJI5DQzP+5v3P8/7DEHANVHM7iXAVyAkwQCNgZYjDp3KH7jzZ1DdwmckBLirrE29KQgcTNOBZTrQdYP2es631LSPnp34zQqDlgTIcvTlOowxNBsG6o3uHw6he899eOt+EBIGeBHxUW5ElDjEYwKSKdFbzzAsVB61/q4lWlsvv3/XXAwJBOaL9pe29slE3MgoOSHF+N0DAwnwfASNehe1WudkYVw7sixgYfFkMXtSikU/VZQU3LhmHjVZ17Sz59Tbt/2QJe3glUIGMnEhe3PgjcR2Ny5dN/Df0873ebX8gS/wshUBUMK+PKveuhzWGZPF0fdiceHq4FAK3a6FyoyuFca13GKA14ruw8qM/ldhXNscBnz03duDUSY83jCchu04ePBvs5ZXtTW+wGQpW12/IZ1xP9r0dAM9g2bOH9Rmg5DDpW0ZwvFVD7CpW1ctqNqQP1DM/ZJRErvdFqw+bqHdMccKqvZr4AEsbdsVj4lTbkTttolqtXWloGp7FtlB7uv+1fIX6bSM9pyJJ9W5a3m1PAYC5ou4H7mUvT44mNwZTwio17to1Dsn8qp21Bc4cGl0WCLcP+vW9wkRQrxddNrW6cyw9NmxnTfthUX7ftgs9LcTP8dkfmxIWeW16cOHTdO22EheLU8v2qYTxezn6bT8zeo1MS/T2dk5GB3rHgNOMd6+SsFxnE3eJcAnohwdUZQkQAjqtTYadePHwri2L/CgHbuxg3/ywPhdUZJbJOnF3GnpPS9fw7ABAkgij3h8flS46REYPRvVig5KoebV8oXAg3bw4ugmjkV+6u+Xt/T1SSDE/wy6sTxrdiHHBYgCH4q8ssqLjOPHZYk/kkiKEUHg4Y5qBgbLpDBNG62W5Rhde4rjsGdISUEUgxHf1zx0MfsWdTBGCHIAeQeABbAyBdPmfziTpdz+SASlMOT1Z9GCoJeCLAtwrTBk2UAYsiKAH+JN2YreXDHg/wilzJ3Oz1YUmEcYY2fce0LIx88BFi6vvp70RPYAAAAASUVORK5CYII=";

class Scratch3RosBlocks {

    constructor(runtime) {
        var rosIP = prompt('Input IP address:');
        alert('Remember to enable connections with ROS:\n\n roslaunch rosbridge_server rosbridge_websocket.launch');

        this.ros = new RosUtil({ url : 'ws://' + rosIP + ':9090' });
        this.topicNames = ['topic'];
        this.serviceNames = ['service'];
        this.runtime = runtime;
    };

    makeMessage({TOPIC}) {
        var ROS = this.ros;
        return new Promise( function(resolve) {
            ROS.getMessageDetailsByTopic(TOPIC).then( function(result) {
                var example = ROS.messageExample(result[0], result);
                resolve(example); });
        });
    };

    makeRequest({SERVICE}) {
        var ROS = this.ros;
        return new Promise( function(resolve) {
            ROS.getRequestDetailsByService(SERVICE).then( function(result) {
                var example = ROS.messageExample(result[0], result);
                resolve(example); });
        });
    };

    printObject({OBJ}) {
        alert(JSON.stringify(OBJ,null,2));
    };

    subscribeTopic({TOPIC}) {
        var ROS = this.ros;
        return new Promise( function(resolve) {
            ROS.getTopic(TOPIC).then(
                rosTopic =>
                    rosTopic.subscribe(msg => { rosTopic.unsubscribe();
                                                resolve(msg); }));
        });
    };

    publishTopic({MSG, TOPIC}, util) {
        var msg = this._getVariableValue(MSG) || JSON.parse(MSG);

        this.ros.getTopic(TOPIC).then(
            rosTopic => rosTopic.publish(msg));
    };

    callService({REQUEST, SERVICE}, util) {
        var req = this._getVariableValue(REQUEST) || JSON.parse(REQUEST);

        var ROS = this.ros;
        return new Promise( function(resolve) {
            ROS.getService(SERVICE).then(
                rosService => rosService.callService(req,
                                                     res => { rosService.unadvertise();
                                                              resolve(res); }));
        });
    };

    getSlot({OBJECT, SLOT}, util) {
        var obj = this._getVariableValue(OBJECT) || JSON.parse(OBJECT);
        return eval('obj' + '.' + SLOT);
    };

    setSlot({VAR, SLOT, VALUE}, util) {
        function setNestedValue(obj, slots, value) {
            var last = slots.length - 1;
            for(var i = 0; i < last; i++)
                obj = obj[ slots[i] ] = obj[ slots[i] ] || {};

            obj = obj[slots[last]] = value;
        };

        function tryParse(val) {
            try {
                return JSON.parse(val);
            } catch(err) { return val; }
        };

        const variable = util.target.lookupVariableByNameAndType(VAR);
        if (!variable) return;

        if (typeof(variable.value) !== 'object') variable.value = {};
        var slt = SLOT.split('.');
        if (Array.isArray(VALUE)) var val = VALUE.map(tryParse);
        else var val = tryParse(VALUE);

        setNestedValue(variable.value, slt, val);

        // TODO: cloud variables
    };

    waitInterpolation() {
        var ROS = this.ros;
        return new Promise( function(resolve) {
            ROS.getTopic('/move_group/result').then(
                rosTopic =>
                    rosTopic.subscribe(msg => { rosTopic.unsubscribe();
                                                resolve(); }));
        });
    };

    _getVariableValue(name, type) {
        var target = this.runtime.getEditingTarget();
        var variable = target.lookupVariableByNameAndType(name, type);
        return variable && variable.value;
    };

    _updateTopicList() {
        var that = this;
        that.ros.getTopics( function(topics){
            that.topicNames = topics.topics.sort(); });

        return that.topicNames.map(function(val) { return {value: val, text: val}; });
    };

    _updateServiceList() {
        var that = this;
        that.ros.getServices( function(services){
            that.serviceNames = services.sort(); });
        return that.serviceNames.map(function(val) { return {value: val, text: val}; });
    };

    _updateVariablesList() {
        try {
            var varlist = this.runtime.getEditingTarget().getAllVariableNamesInScopeByType();
        } catch(err) { return [{value: 'my variable', text: 'my variable'}] }

        if (varlist.length == 0)
            return [{value: 'my variable', text: 'my variable'}];
        else
            return varlist.map(function(val) {return {value: val, text: val}; });
    };

    getInfo() {
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
                        TOPIC: {
                            type: ArgumentType.STRING,
                            menu: 'topicsMenu',
                            defaultValue: this.topicNames[0]
                        }
                    }
                },
                {
                    opcode: 'makeMessage',
                    blockType: BlockType.REPORTER,
                    text: 'Create message for [TOPIC]',
                    arguments: {
                        TOPIC: {
                            type: ArgumentType.STRING,
                            menu: 'topicsMenu',
                            defaultValue: this.topicNames[0]
                        }
                    }
                },
                {
                    opcode: 'publishTopic',
                    blockType: BlockType.COMMAND,
                    text: 'Publish [MSG] to [TOPIC]',
                    arguments: {
                        MSG: {
                            type: ArgumentType.STRING,
                            menu: 'variablesMenu',
                            defaultValue: this._updateVariablesList()[0].text
                        },
                        TOPIC: {
                            type: ArgumentType.STRING,
                            menu: 'topicsMenu',
                            defaultValue: this.topicNames[0]
                        }
                    }
                },
                {
                    opcode: 'makeRequest',
                    blockType: BlockType.REPORTER,
                    text: 'Create request for [SERVICE]',
                    arguments: {
                        SERVICE: {
                            type: ArgumentType.STRING,
                            menu: 'servicesMenu',
                            defaultValue: this.serviceNames[0]
                        }
                    }
                },
                {
                    opcode: 'callService',
                    blockType: BlockType.REPORTER,
                    text: 'Send [REQUEST] to [SERVICE]',
                    arguments: {
                        REQUEST: {
                            type: ArgumentType.STRING,
                            menu: 'variablesMenu',
                            defaultValue: this._updateVariablesList()[0].text
                        },
                        SERVICE: {
                            type: ArgumentType.STRING,
                            menu: 'servicesMenu',
                            defaultValue: this.serviceNames[0]
                        }
                    }
                },
                {
                    opcode: 'printObject',
                    blockType: BlockType.COMMAND,
                    text: 'Print [OBJ]',
                    arguments: {
                        OBJ: {
                            type: ArgumentType.STRING,
                            defaultValue: 'object'
                        }
                    }
                },
                {
                    opcode: 'getSlot',
                    blockType: BlockType.REPORTER,
                    text: 'Get [OBJECT] [SLOT]',
                    arguments: {
                        OBJECT: {
                            type: ArgumentType.STRING,
                            menu: 'variablesMenu',
                            defaultValue: this._updateVariablesList()[0].text
                        },
                            SLOT: {
                                type: ArgumentType.STRING,
                                defaultValue: 'data'
                            }
                    }
                },
                {
                    opcode: 'setSlot',
                    blockType: BlockType.COMMAND,
                    text: 'Set [VAR] [SLOT] to [VALUE]',
                    arguments: {
                        VAR: {
                            type: ArgumentType.STRING,
                            menu: 'variablesMenu',
                            defaultValue: this._updateVariablesList()[0].text
                        },
                            SLOT: {
                                type: ArgumentType.STRING,
                                defaultValue: 'data'
                            },
                            VALUE: {
                                type: ArgumentType.STRING,
                                defaultValue: 'Hello!'
                            }
                    }
                },
                {
                    opcode: 'waitInterpolation',
                    blockType: BlockType.COMMAND,
                    text: 'Wait interpolation',
                }
            ],
            menus: {
                topicsMenu: '_updateTopicList',
                servicesMenu: '_updateServiceList',
                variablesMenu: '_updateVariablesList'
            }
        }
    }
}

module.exports = Scratch3RosBlocks;

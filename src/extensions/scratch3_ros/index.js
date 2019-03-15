const math = require('mathjs');
const JSON = require('circular-json');
const BlockType = require('../../extension-support/block-type');
const ArgumentType = require('../../extension-support/argument-type');
const Scratch3RosBase = require('./RosUtil');

const icon = 'data:image/svg+xml;base64,PD94bWwgdmVyc2lvbj0iMS4wIiBlbmNvZGluZz0iVVRGLTgiIHN0YW5kYWxvbmU9Im5vIj8+CjwhLS0gQ3JlYXRlZCB3aXRoIElua3NjYXBlIChodHRwOi8vd3d3Lmlua3NjYXBlLm9yZy8pIC0tPgoKPHN2ZwogICB4bWxuczpkYz0iaHR0cDovL3B1cmwub3JnL2RjL2VsZW1lbnRzLzEuMS8iCiAgIHhtbG5zOmNjPSJodHRwOi8vY3JlYXRpdmVjb21tb25zLm9yZy9ucyMiCiAgIHhtbG5zOnJkZj0iaHR0cDovL3d3dy53My5vcmcvMTk5OS8wMi8yMi1yZGYtc3ludGF4LW5zIyIKICAgeG1sbnM6c3ZnPSJodHRwOi8vd3d3LnczLm9yZy8yMDAwL3N2ZyIKICAgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIgogICB4bWxuczpzb2RpcG9kaT0iaHR0cDovL3NvZGlwb2RpLnNvdXJjZWZvcmdlLm5ldC9EVEQvc29kaXBvZGktMC5kdGQiCiAgIHhtbG5zOmlua3NjYXBlPSJodHRwOi8vd3d3Lmlua3NjYXBlLm9yZy9uYW1lc3BhY2VzL2lua3NjYXBlIgogICBpZD0ic3ZnMiIKICAgdmVyc2lvbj0iMS4xIgogICBpbmtzY2FwZTp2ZXJzaW9uPSIwLjkxIHIxMzcyNSIKICAgd2lkdGg9IjIyLjYxMjM0NSIKICAgaGVpZ2h0PSIyMi44NzgzNTkiCiAgIHZpZXdCb3g9IjAgMCAyMi42MTIzNDUgMjIuODc4MzU5IgogICBzb2RpcG9kaTpkb2NuYW1lPSJyb3Mtc21hbGwuc3ZnIgogICBpbmtzY2FwZTpleHBvcnQtZmlsZW5hbWU9Ii90bXAvcm9zLnN2ZyIKICAgaW5rc2NhcGU6ZXhwb3J0LXhkcGk9IjM1LjY0MzU2NiIKICAgaW5rc2NhcGU6ZXhwb3J0LXlkcGk9IjM1LjY0MzU2NiI+CiAgPG1ldGFkYXRhCiAgICAgaWQ9Im1ldGFkYXRhOCI+CiAgICA8cmRmOlJERj4KICAgICAgPGNjOldvcmsKICAgICAgICAgcmRmOmFib3V0PSIiPgogICAgICAgIDxkYzpmb3JtYXQ+aW1hZ2Uvc3ZnK3htbDwvZGM6Zm9ybWF0PgogICAgICAgIDxkYzp0eXBlCiAgICAgICAgICAgcmRmOnJlc291cmNlPSJodHRwOi8vcHVybC5vcmcvZGMvZGNtaXR5cGUvU3RpbGxJbWFnZSIgLz4KICAgICAgICA8ZGM6dGl0bGU+PC9kYzp0aXRsZT4KICAgICAgPC9jYzpXb3JrPgogICAgPC9yZGY6UkRGPgogIDwvbWV0YWRhdGE+CiAgPGRlZnMKICAgICBpZD0iZGVmczYiIC8+CiAgPHNvZGlwb2RpOm5hbWVkdmlldwogICAgIHBhZ2Vjb2xvcj0iI2ZmZmZmZiIKICAgICBib3JkZXJjb2xvcj0iIzY2NjY2NiIKICAgICBib3JkZXJvcGFjaXR5PSIxIgogICAgIG9iamVjdHRvbGVyYW5jZT0iMTAiCiAgICAgZ3JpZHRvbGVyYW5jZT0iMTAiCiAgICAgZ3VpZGV0b2xlcmFuY2U9IjEwIgogICAgIGlua3NjYXBlOnBhZ2VvcGFjaXR5PSIwIgogICAgIGlua3NjYXBlOnBhZ2VzaGFkb3c9IjIiCiAgICAgaW5rc2NhcGU6d2luZG93LXdpZHRoPSIxODYzIgogICAgIGlua3NjYXBlOndpbmRvdy1oZWlnaHQ9IjEwNTYiCiAgICAgaWQ9Im5hbWVkdmlldzQiCiAgICAgc2hvd2dyaWQ9ImZhbHNlIgogICAgIGlua3NjYXBlOnpvb209IjEyLjcwMjcwMiIKICAgICBpbmtzY2FwZTpjeD0iNy4yNjc2Mjg5IgogICAgIGlua3NjYXBlOmN5PSIyLjI2Njk3ODkiCiAgICAgaW5rc2NhcGU6d2luZG93LXg9IjU3IgogICAgIGlua3NjYXBlOndpbmRvdy15PSIyNCIKICAgICBpbmtzY2FwZTp3aW5kb3ctbWF4aW1pemVkPSIxIgogICAgIGlua3NjYXBlOmN1cnJlbnQtbGF5ZXI9InN2ZzIiCiAgICAgZml0LW1hcmdpbi10b3A9IjAiCiAgICAgZml0LW1hcmdpbi1sZWZ0PSIwIgogICAgIGZpdC1tYXJnaW4tcmlnaHQ9IjAiCiAgICAgZml0LW1hcmdpbi1ib3R0b209IjAiIC8+CiAgPHBhdGgKICAgICBzdHlsZT0iZmlsbDojNDEwMDY5O2ZpbGwtb3BhY2l0eToxIgogICAgIGQ9Ik0gMS44MTk1NCwyMS4xMTI5NDQgQyAwLjcwMTAwNTUxLDIwLjE3MTc2MiAwLjYxMDY1NjkyLDE4LjUyOTE5IDEuNjIxODA0OSwxNy41MTgwNCAzLjUyOTA2NzEsMTUuNjEwNzc5IDYuNDgwNjc5NSwxOC4wMzc2MDggNS4yMTk2MzgyLDIwLjQ3NjE5MyA0LjYyNjkzNzUsMjEuNjIyMzQ4IDIuODI1Nzk4OCwyMS45NTk2NTYgMS44MTk1NCwyMS4xMTI5NDQgWiBtIDguNDU3NjIzLDAuMTc5MzQyIEMgOS4xNTI3MzczLDIwLjY4MjE4IDguODI1NzM0NiwxOC43MjkwNjYgOS42NzgzNjExLDE3LjcxNTc3OCBjIDAuOTY0MTQ2OSwtMS4xNDU4MjUgMi41MzkxNzQ5LC0xLjE1NDIzNyAzLjQ5NDY5MjksLTAuMDE4NjYgMC45MDIwNTUsMS4wNzIwMzEgMC43ODIxNTUsMi41MTk2MzIgLTAuMjgyOTIyLDMuNDE1ODM0IC0wLjY0ODY3MiwwLjU0NTgyMyAtMS43OTI4MTcsMC42MjQzNTEgLTIuNjEyOTY5LDAuMTc5MzM5IHogbSA3LjkzNTU2NiwtMC4yODAxMzQgYyAtMS45ODY4NzksLTEuNzQ0NTAxIC0wLjA1OTMzLC01LjAxMDUzNiAyLjM2MDQxOCwtMy45OTk0OTggMS41Nzc5NTEsMC42NTkzMTIgMS45MDk2MDQsMi43NzQyNzQgMC42MjAzMjIsMy45NTU4MTIgLTAuOTI5MDE5LDAuODUxMzg0IC0yLjA0MjI0MSwwLjg2NzY5OCAtMi45ODA3NCwwLjA0MzY4IHogTSAxLjkwNjkzNjYsMTIuODUyNDk1IEMgMS4yODY2NywxMi4zNjQ1OTUgMC44NjU5NTc0OCwxMS41NjE4MDIgMC44NjU5NTc0OCwxMC44NjYxMjggYyAwLC0wLjc0NzgyOSAwLjcwNjk5NTYyLC0xLjczMzA4MTUgMS41MzQ5MDcwMiwtMi4xMzkwMDg0IDEuOTE3NTg5OCwtMC45NDAxOTUyIDMuODU1NDQ2LDEuNDAxNTI0NCAyLjgxODc3MzcsMy40MDYyMjk0IC0wLjU2MTUyMzYsMS4wODU4NjcgLTIuMzUyMjc3NywxLjQ3NDYxOCAtMy4zMTI3MDE2LDAuNzE5MTQ2IHogbSA4LjE5OTk2NDQsLTAuMDMwOSBDIDkuNDM5NDQ4NCwxMi4yNTczNzggOS4yMDg4MDI1LDExLjc0MTQ5NiA5LjIwODgwMjUsMTAuODEyODQzIGMgMCwtMS41OTY1OTI2IDEuNjA0NzQyNSwtMi43MzQ5OTMyIDMuMDIxMTM2NSwtMi4xNDMxODU4IDEuNzI4NjM1LDAuNzIyMjcxIDIuMDgwMTY4LDIuOTA1NjE0OCAwLjY2MDE5Myw0LjEwMDQ0MjggLTAuNzQ4ODA2LDAuNjMwMDc5IC0yLjA2OTg3MywwLjY1NDUyIC0yLjc4MzIzMSwwLjA1MTUgeiBtIDguMDc5NDcyLC0wLjE4NDU4NyBjIC0xLjI4OTA3OSwtMS4xODEzNTMgLTAuOTgyMDAxLC0zLjE3OTI4OTYgMC41OTU5MzEsLTMuODc3MjgyMiAyLjA3NTY4LC0wLjkxODE3MiAzLjkyNDY0MywxLjE3NTI5OTggMi45MTU3NjcsMy4zMDEzNTAyIC0wLjM2NjgwOSwwLjc3Mjk5MiAtMS4wODk2ODksMS4xNzg1ODIgLTIuMTAwNTc4LDEuMTc4NTgyIC0wLjU5MTY4MiwwIC0wLjg5NDc1LC0wLjEyOTQzIC0xLjQxMTEyLC0wLjYwMjY1IHogTSAyLjM1MjgyNTksNC43NTk5Mzk0IEMgMS41NjU1MDMyLDQuMzYxMzcyNyAwLjg2NTk1NzQ4LDMuMzYxODI5MSAwLjg2NTk1NzQ4LDIuNjM1NDMyOSBjIDAsLTEuMjQ1OTY2OSAxLjA3MzE0MDMyLC0yLjM3MzUzMjE5IDIuMjU4OTY3MDIsLTIuMzczNTMyMTkgMS40NTAwNjY3LDAgMi4zNzY2NjU4LDAuOTE4NTQyODkgMi4zNzUyNTM2LDIuMzU0NTk5MTkgLTAuMDAxNjksMS43NDIxNzM2IC0xLjY4MDk0NjQsMi44ODU3ODA3IC0zLjE0NzM1MjIsMi4xNDM0Mzk1IHogbSA4LjIzMDcyNjEsMC4wNjU5NzUgQyA5LjcwMDExNTMsNC40MzAxMDIgOS4yMDg4MDI1LDMuNjgxMjUwNCA5LjIwODgwMjUsMi43MzA1NDU4IGMgMCwtMS41NzA2NTcxIDAuODQ3MzYyNSwtMi40Njg2NDQ3OCAyLjMyOTQ3MjUsLTIuNDY4NjQ0NzggMC43MjYxLDAgMC45NTQ4MTksMC4wOTU0NzMgMS40NzUwNzIsMC42MTU3MjY3MyAwLjgwNTUxMywwLjgwNTUxMTE1IDAuOTg3MzE3LDEuNzI0ODQwMDUgMC41MjUwNDYsMi42NTUwMzU2NSAtMC42MTY5OTcsMS4yNDE1NDQ4IC0xLjg1Nzk3NywxLjc4NDY4NzMgLTIuOTU0ODQxLDEuMjkzMjUwOSB6IG0gOC4xNTkxMSwtMC4wOTI3MjQgYyAtMS41MzkyMzMsLTAuNzAzODI1NSAtMS44MjY4OCwtMi43MDQyMjc4IC0wLjU1NjI4OSwtMy44Njg2Mzc2NSAxLjUzMzQyMywtMS40MDUyNzU5NCAzLjgxNzc4MywtMC4zMDU3ODE1IDMuODEzNTY5LDEuODM1NTIzODUgLTAuMDAzNCwxLjY2MDQwOCAtMS43MjM3OSwyLjczNDMxNjYgLTMuMjU3MjgsMi4wMzMxMTM4IHoiCiAgICAgaWQ9InBhdGg0MTM4IgogICAgIGlua3NjYXBlOmNvbm5lY3Rvci1jdXJ2YXR1cmU9IjAiCiAgICAgc29kaXBvZGk6bm9kZXR5cGVzPSJzc3Nzc3Nzc3Nzc3Nzc3Nzc3Nzc3Nzc3Nzc3Nzc3NzY3Nzc3Nzc3Nzc3NzIiAvPgo8L3N2Zz4K'

class Scratch3RosBlocks extends Scratch3RosBase {

    constructor(runtime) {
        super('ROS', 'ros', runtime);
    }

    subscribeTopic ({TOPIC}) {
        const that = this;
        return new Promise(resolve => {
            that.ros.getTopic(TOPIC).then(
                rosTopic => {
                    if (!rosTopic.messageType) resolve();
                    rosTopic.subscribe(msg => {
                        // rosTopic.unsubscribe();
                        if (rosTopic.messageType === 'std_msgs/String') {
                            msg.data = that._tryParse(msg.data, msg.data);
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
            ROS.getService(SERVICE).then(rosService => {
                if (!rosService.serviceType) resolve();
                rosService.callService(req,
                    res => {
                        rosService.unadvertise();
                        resolve(res);
                    });
            });
        });
    }

    getParamValue ({NAME}) {
        const that = this;
        return new Promise(resolve => {
            const param = that.ros.getParam(NAME);
            param.get(val => {
                if (val === null) resolve();
                if (that._isJSON(val)) {
                    val.toString = function () { return JSON.stringify(this); };
                }
                resolve(val);
            });
        });
    }

    setParamValue ({NAME, VALUE}) {
        const param = this.ros.getParam(NAME);
        const val = Array.isArray(VALUE) ?
            VALUE.map(v => this._tryParse(v, v)) :
            this._tryParse(VALUE, VALUE);
        param.set(val);
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

    setSlot ({VAR, SLOT, VALUE}, util) {
        const setNestedValue = function (obj, slots, value) {
            const last = slots.length - 1;
            for (let i = 0; i < last; i++) {
                const slot = slots[i];
                if (!obj.hasOwnProperty(slot) || typeof obj[slot] !== 'object') {
                    obj[slot] = isNaN(parseInt([slots[i + 1]])) ? {} : [];
                }
                obj = obj[slot];
            }
            obj[slots[last]] = value;
        };

        const variable = util.target.lookupVariableByNameAndType(VAR);
        if (!variable) return;
        const variableValue = this._getVariableValue(variable);

        if (this._isJSON(variableValue)) {
            // Clone object to avoid overwriting parent variables
            variable.value = JSON.parse(JSON.stringify(variableValue));
        } else variable.value = {};
        const slt = SLOT.split(/\.|\[|\]/).filter(Boolean);
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

    solveFormula ({EXPRESSION, OBJECT}) {
        const obj = this._getVariableValue(OBJECT) || this._tryParse(OBJECT);
        let binds;
        if (this._isJSON(obj)) {
            binds = JSON.parse(JSON.stringify(obj));
            delete binds.toString;
            delete binds.constructor;
        } else {
            binds = {};
            binds[OBJECT] = obj;
        }

        try {
            const result = math.eval(EXPRESSION, binds);
            if (math.typeof(result) === 'Unit') return result.toNumber();
            return result;
        } catch (err) {
            return;
        }
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
            id: this.extensionId,
            name: this.extensionName,
            showStatusButton: true,

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
                    text: 'Set [VAR] [SLOT] to [VALUE]',
                    arguments: {
                        VAR: variableArg,
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
                },
                '---',
                {
                    opcode: 'solveFormula',
                    blockType: BlockType.REPORTER,
                    text: '[EXPRESSION] binding [OBJECT]',
                    arguments: {
                        EXPRESSION: stringArg('(data + 1) ^ 2'),
                        OBJECT: variableArg
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

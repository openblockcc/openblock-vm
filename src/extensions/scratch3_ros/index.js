//import * as ROSLIB from 'roslib';

const ArgumentType = require('../../extension-support/argument-type');
const BlockType = require('../../extension-support/block-type');
const formatMessage = require('format-message');
const RosUtil = require('./RosUtil');

const icon = "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAADTUlEQVRIS63VTWgUZxgH8P+7Mzsf+2U2SrPJKBoEA1WKUMzupaKHpgoetadCnaBN9FRaLLRQEW17KKIed1V21dJLbU/FRqiixx219NDaVi+NmnWNZT+cze7Ozsf7yoxEYpnMRJI5DQzP+5v3P8/7DEHANVHM7iXAVyAkwQCNgZYjDp3KH7jzZ1DdwmckBLirrE29KQgcTNOBZTrQdYP2es631LSPnp34zQqDlgTIcvTlOowxNBsG6o3uHw6he899eOt+EBIGeBHxUW5ElDjEYwKSKdFbzzAsVB61/q4lWlsvv3/XXAwJBOaL9pe29slE3MgoOSHF+N0DAwnwfASNehe1WudkYVw7sixgYfFkMXtSikU/VZQU3LhmHjVZ17Sz59Tbt/2QJe3glUIGMnEhe3PgjcR2Ny5dN/Df0873ebX8gS/wshUBUMK+PKveuhzWGZPF0fdiceHq4FAK3a6FyoyuFca13GKA14ruw8qM/ldhXNscBnz03duDUSY83jCchu04ePBvs5ZXtTW+wGQpW12/IZ1xP9r0dAM9g2bOH9Rmg5DDpW0ZwvFVD7CpW1ctqNqQP1DM/ZJRErvdFqw+bqHdMccKqvZr4AEsbdsVj4lTbkTttolqtXWloGp7FtlB7uv+1fIX6bSM9pyJJ9W5a3m1PAYC5ou4H7mUvT44mNwZTwio17to1Dsn8qp21Bc4cGl0WCLcP+vW9wkRQrxddNrW6cyw9NmxnTfthUX7ftgs9LcTP8dkfmxIWeW16cOHTdO22EheLU8v2qYTxezn6bT8zeo1MS/T2dk5GB3rHgNOMd6+SsFxnE3eJcAnohwdUZQkQAjqtTYadePHwri2L/CgHbuxg3/ywPhdUZJbJOnF3GnpPS9fw7ABAkgij3h8flS46REYPRvVig5KoebV8oXAg3bw4ugmjkV+6u+Xt/T1SSDE/wy6sTxrdiHHBYgCH4q8ssqLjOPHZYk/kkiKEUHg4Y5qBgbLpDBNG62W5Rhde4rjsGdISUEUgxHf1zx0MfsWdTBGCHIAeQeABbAyBdPmfziTpdz+SASlMOT1Z9GCoJeCLAtwrTBk2UAYsiKAH+JN2YreXDHg/wilzJ3Oz1YUmEcYY2fce0LIx88BFi6vvp70RPYAAAAASUVORK5CYII=";

class Scratch3RosBlocks {

    constructor(runtime) {
	alert('Remember to enable connections with ROS:\n\n roslaunch rosbridge_server rosbridge_websocket.launch');

	this.ros = new RosUtil({ url : 'ws://localhost:9090' });
	this.topicNames = ['topic'];
	this.serviceNames = ['service'];
	this.runtime = runtime;
    };

    makeMessage({TOPIC}) {
	var ROS = this.ros;
	return new Promise( function(resolve) {
	    ROS.getMessageDetailsByTopic(TOPIC).then( function(result) {
		var example = ROS.messageExample(result[0], result);
		resolve(JSON.stringify(example)); });
	});
    };

    makeRequest({SERVICE}) {
	var ROS = this.ros;
	return new Promise( function(resolve) {
	    ROS.getRequestDetailsByService(SERVICE).then( function(result) {
		var example = ROS.messageExample(result[0], result);
		resolve(JSON.stringify(example)); });
	});
    };

    printObject({OBJ}) {
	alert(JSON.stringify(JSON.parse(OBJ),null,2));
    };

    subscribeTopic({TOPIC}) {
	var ROS = this.ros;
	return new Promise( function(resolve) {
	    ROS.getTopic(TOPIC).then(
		rosTopic =>
		    rosTopic.subscribe(msg => { rosTopic.unsubscribe();
						resolve(JSON.stringify(msg)); }));
	});
    };

    publishTopic({MSG, TOPIC}) {
	this.ros.getTopic(TOPIC).then(
	    rosTopic => rosTopic.publish(JSON.parse(MSG)));
    };

    callService({REQUEST, SERVICE}) {
	var ROS = this.ros;
	return new Promise( function(resolve) {
	    ROS.getService(SERVICE).then(
		rosService => rosService.callService(JSON.parse(REQUEST),
						     res => { rosService.unadvertise();
							      resolve(JSON.stringify(res)); }));
	});
    };

    getSlot({OBJ, SLOT}) {
	var res = eval('JSON.parse(OBJ)' + '.' + SLOT);
	// var slotArray = slot.split('.'),
	//     res = JSON.parse(obj);
	// for (var i=0, len=slotArray.length; i<len; i++)
	//     res = res[slotArray[i]];

	if (typeof(res) === 'object')
	    return JSON.stringify(res);
	else
	    return res;
    };

    setSlot({OBJ, SLOT, VALUE}) {
	var object = {};
	try {
	    let obj = JSON.parse(OBJ);
	    if (typeof(obj) === 'object') object = obj;
	} catch(err) {}
	eval('object.' + SLOT + '=' + VALUE);
	return JSON.stringify(object);
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
			    defaultValue: 'message'
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
			    defaultValue: 'request'
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
		    text: 'Get [OBJ] [SLOT]',
		    arguments: {
			OBJ: {
			    type: ArgumentType.STRING,
			    defaultValue: 'object'
			},
			SLOT: {
			    type: ArgumentType.STRING,
			    defaultValue: 'data'
			}
		    }
		},
		{
		    opcode: 'setSlot',
		    blockType: BlockType.REPORTER,
		    text: 'Set [OBJ] [SLOT] to [VALUE]',
		    arguments: {
			OBJ: {
			    type: ArgumentType.STRING,
			    defaultValue: 'object'
			},
			SLOT: {
			    type: ArgumentType.STRING,
			    defaultValue: 'data'
			},
			VALUE: {
			    type: ArgumentType.STRING,
			    defaultValue: '"Hello!"'
			}
		    }
		}
	    ],
	    menus: {
		topicsMenu: '_updateTopicList',
		servicesMenu: '_updateServiceList'
	    }
	}
    }
}

module.exports = Scratch3RosBlocks;

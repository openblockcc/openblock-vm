import * as ROSLIB from 'roslib'

const std_msgs_array = {
    byte: 'std_msgs/Byte',
    int8: 'std_msgs/Int8',
    int16: 'std_msgs/Int16',
    int32: 'std_msgs/Int32',
    int64: 'std_msgs/Int64',
    uint8: 'std_msgs/UInt8',
    uint16: 'std_msgs/UInt16',
    uint32: 'std_msgs/UInt32',
    uint64: 'std_msgs/UInt64',
    float32: 'std_msgs/Float32',
    float64: 'std_msgs/Float64'
}

class RosUtil extends ROSLIB.Ros {
    constructor(options) {
	super(options)
    };

    getMessageDetailsByTopic(topic) {
	var ros = this;
	return new Promise( function(resolve,reject) {	
	    ros.getTopicType(
		topic,
		type => ros.getMessageDetails(type,resolve))
	});
    };

    getRequestDetailsByService(service) {
	var ros = this;
	return new Promise( function(resolve,reject) {	
	    ros.getServiceType(
		service,
		type => ros.getServiceRequestDetails(type, details => resolve(details.typedefs)));
	});
    };

    getTopic(name) {
	var ros = this;
	return new Promise( function(resolve,reject) {
	    ros.getTopicType(
		name,
		type =>
		    resolve(new ROSLIB.Topic({
			ros : ros,
			name : name,
			messageType : type
		    })));
	});
    };

    getService(name) {
	var ros = this;
	return new Promise( function(resolve,reject) {
	    ros.getServiceType(
		name,
		type =>
		    resolve(new ROSLIB.Service({
			ros : ros,
			name : name,
			serviceType : type
		    })));
	});
    };

    messageExample(obj, list) {
	var result = {};
	for (var i=0, len = obj.fieldnames.length; i<len; i++) {

	    var ex = obj.examples[i];
	    switch (ex) {
	    case '[]':
	    case '{}':
		if (obj.fieldtypes[i] in std_msgs_array)
		    ex = ["0"];
		else {
		    ex = this.messageExample(
			list.find( object => object.type === obj.fieldtypes[i]), list);
		    if (ex === '[]') ex = [ex];
		};
	    default:
		result[obj.fieldnames[i]] = ex;
	    };
	};	

	return result;
    };

}

module.exports = RosUtil;

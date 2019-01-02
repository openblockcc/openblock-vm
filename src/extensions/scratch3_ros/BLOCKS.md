# Scratch 3 ROS Extension Blocks Description

## ROS Related

### Subscribe Topic
![subscribe](https://user-images.githubusercontent.com/20625381/50579761-6a68b200-0e8b-11e9-80ca-fdcf4f4136cd.png)
- *Text*: 'Get message from [TOPIC]'
- *Description*: Waits for and report the next message published to TOPIC
- *Arguments*:
  - *TOPIC*: An existing ROS topic pick from the dropdown list or a string naming a new topic
- *Fail cases*: When TOPIC is not advertised
- *Notes*:
  - Use inside a `forever` loop to handle every new message published to TOPIC
  - Topic existence can be verified with `if <subscribe> = undefined`

### Publish Topic
![publish](https://user-images.githubusercontent.com/20625381/50579765-72c0ed00-0e8b-11e9-9c53-b33ce6da3a41.png)
- *Text*: 'Publish [MSG] to [TOPIC]'
- *Description*: Publishes MSG to TOPIC and returns immediately
- *Arguments*:
  - *MSG*: Any Scratch object matching TOPIC type (usually JSON objects)
  - *TOPIC*: An existing ROS topic pick from the dropdown list or a string naming a new topic
- *Fail cases*: When MSG type does not match the message type of TOPIC
- *Notes*:
  - Use inside a `forever` loop with `wait` to publish constantly (however, publishing rates are limited and tend to be unstable)
  - If TOPIC is not advertised, it is newly advertised according to the following:
    - std_msgs/Int32, if MSG is an integer or a `{data: INTEGER}` like object
    - std_msgs/Float64, if MSG is a float or a `{data: FLOAT}` like object
    - std_msgs/Bool, if MSG is a boolean or a `{data: BOOL}` like object
    - std_msgs/String, for all other cases
  - Fields that are not specified in MSG but are required by TOPIC type are automatically initialized to the default value
  - A warning is emitted at rosbridge_server terminal if types mismatch

### Service Call
![call](https://user-images.githubusercontent.com/20625381/50579767-76ed0a80-0e8b-11e9-93e5-3f597b9a23ce.png)
- *Text*: 'Send [REQUEST] to [SERVICE]'
- *Description*: Sends REQUEST to SERVICE, then waits for and report the response
- *Arguments*:
  - *REQUEST*: A JSON object matching SERVICE type
  - *SERVICE*: An existing ROS service pick from the dropdown list or given by a string
- *Fail cases*: When REQUEST type does not match SERVICE type
- *Notes*:
  - Serving new services is not supported
  - Service existence can be verified with `if <call> = undefined`

### Get ROS Parameter
![getparam](https://user-images.githubusercontent.com/20625381/50579772-7d7b8200-0e8b-11e9-98f0-128bc150b56f.png)
- *Text*: 'Get rosparam [NAME]'
- *Description*: Reports the ROS parameter with given NAME
- *Arguments*:
  - *NAME*: An existing ROS parameter pick from the dropdown list or given by a string
- *Fail cases*: When NAME does not refer to any existing ROS parameter
- *Notes*:
  - If NAME is a namespace, a JSON object with all parameters under that namespace is reported
  - Parameter existence can be verified with `if <get> = undefined`

### Set ROS Parameter
![setparam](https://user-images.githubusercontent.com/20625381/50579774-810f0900-0e8b-11e9-8a28-fa35662bb2f4.png)
- *Text*: 'Set rosparam [NAME] to [VALUE]'
- *Description*: Sets the ROS parameter NAME to VALUE
- *Arguments*:
  - *NAME*: An existing ROS parameter pick from the dropdown list or given by a string
  - *VALUE*: Any Scratch object
- *Fail cases*: Unknown
- *Notes*:
  - If VALUE is a JSON object, parameters included in VALUE are set with namespace NAME

## JSON Objects Related

### Get JSON Object Slot
![getslot](https://user-images.githubusercontent.com/20625381/50579779-879d8080-0e8b-11e9-9a31-272fcf81d5fb.png)
- *Text*: 'Get [OBJECT] [SLOT]'
- *Description*: Reports the value stored at OBJECT SLOT
- *Arguments*:
  - *OBJECT*: A JSON object given by a Scratch variable pick from the dropdown list or by a string
  - *SLOT*: Key name or chain of keys separated by `.` or `[]`
- *Fail cases*: When OBJECT does not have any value stored at SLOT
- *Notes*:
  - Slot existence can be verified with `if <get> = undefined`

### Set JSON Object Slot
![setslot](https://user-images.githubusercontent.com/20625381/50579787-8bc99e00-0e8b-11e9-91a9-1cb9baddbdbd.png)
- *Text*: 'Set [VAR] [SLOT] to [VALUE]'
- *Description*: Stores VALUE at VAR SLOT
- *Arguments*:
  - *VAR*: A Scratch variable pick from the dropdown list or given by a string
  - *SLOT*: Key name or chain of keys separated by `.` or `[]`
  - *VALUE*: Any Scratch object
- *Fail cases*: When SLOT refers to JSON reserved keys such as `toString`, `constructor` etc
- *Notes*:
  - When VAR value is not a JSON object, a new JSON object is created and VALUE is assigned to its SLOT
  - Scratch lists are assigned as arrays

### Show JSON Object Slot
![show](https://user-images.githubusercontent.com/20625381/50579790-92f0ac00-0e8b-11e9-9720-55aca55fb317.png)
- *Text*: 'Show [VAR] [SLOT]'
- *Description*: Displays value stored at VAR SLOT in Scratch viewport
- *Arguments*:
  - *VAR*: A Scratch variable pick from the dropdown list or given by a string
  - *SLOT*: Key name or chain of keys separated by `.` or `[]`
- *Fail cases*: When VAR value does not have any value stored at SLOT
- *Notes*:
  - Automatically hides if VAR is renamed or changes in a way that SLOT is no more found

### Hide JSON Object Slot
![hide](https://user-images.githubusercontent.com/20625381/50579795-9d12aa80-0e8b-11e9-84d9-57b553305ba7.png)
- *Text*: 'Hide [VAR] [SLOT]'
- *Description*: Stop displaying the value stored at VAR SLOT in Scratch viewport
- *Arguments*:
  - *VAR*: A Scratch variable pick from the dropdown list or given by a string
  - *SLOT*: Key name or chain of keys separated by `.` or `[]`
- *Fail cases*: Unknown
- *Notes*:
  - Does nothing when VAR SLOT is not being displayed at Scratch viewport

## Arithmetic Related
![formula](https://user-images.githubusercontent.com/20625381/50579797-a3a12200-0e8b-11e9-8c71-792b80837635.png)
### Solve Formula with bindings
- *Text*: '[EXPRESSION] binding [OBJECT]'
- *Description*: Compute and report the value of EXPRESSION, using variable values given in OBJECT
- *Arguments*:
  - *EXPRESSION*: A valid mathematics expression
  - *OBJECT*: A Scratch variable pick from the dropdown list or a string giving a JSON object
- *Fail cases*: When EXPRESSION is an invalid mathematics expression; When OBJECT contains keys with invalid name (including spaces, numbers etc)
- *Notes*:
  - Any function provided by [mathjs](http://mathjs.org/docs/reference/functions.html) can be used
  - Evaluation failure can be verified with `if <solve> = undefined`

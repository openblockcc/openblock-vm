const formatMessage = require('format-message');
const Buffer = require('buffer').Buffer;

const ArgumentType = require('../../extension-support/argument-type');
const BlockType = require('../../extension-support/block-type');
const ProgramModeType = require('../../extension-support/program-mode-type');
const Serialport = require('../../io/serialport');
const Base64Util = require('../../util/base64-util');

const Firmata = require('../../lib/firmata/firmata');

/**
 * The list of USB device filters.
 * @readonly
 */
const PNPID_LIST = [
    // https://github.com/arduino/Arduino/blob/1.8.0/hardware/arduino/avr/boards.txt#L268-L275
    'USB\\VID_1B4F&PID_2B74',
    'USB\\VID_1B4F&PID_2B75'
];

/**
 * Configuration of serialport
 * @readonly
 */
const SERIAL_CONFIG = {
    baudRate: 57600,
    dataBits: 8,
    stopBits: 1
};

/**
 * Configuration of build and flash. Used by arduino_debug and avrdude.
 * @readonly
 */
const DIVECE_OPT = {
    type: 'arduino',
    fqbn: 'SparkFun:avr:makeymakey',
    partno: 'atmega32u4',
    programmerId: 'avr109',
    baudrate: '57600'
};

/**
 * A string to report connect firmata timeout.
 * @type {formatMessage}
 */
const ConnectFirmataTimeout = formatMessage({
    id: 'makeymakey.connection.connectFirmataTimeout',
    default: 'Timeout when try to connect firmata, please download the firmware first',
    description: 'label for connect firmata timeout'
});

/**
 * A time interval to send firmata hartbeat(in milliseconds).
 */
const FrimataHeartbeatInterval = 2000;

/**
 * A time interval to wait (in milliseconds) before reporting to the serialport socket
 * that heartbeat has stopped coming from the peripheral.
 */
const FrimataHeartbeatTimeout = 5000;

const Pins = {
    D0: 'PD0',
    D1: 'PD1',
    D2: 'PD2',
    D3: 'PD3',
    D4: 'PD4',
    D5: 'PD5',
    D14: 'MISO',
    D15: 'SCK',
    D16: 'MOSI',
    A0: 'A0',
    A1: 'A1',
    A2: 'A2',
    A3: 'A3',
    A4: 'A4',
    A5: 'A5'
};


const Level = {
    High: '1',
    Low: '0'
};

const Buadrate = {
    B4800: '4800',
    B9600: '9600',
    B19200: '19200',
    B38400: '38400',
    B57600: '57600',
    B115200: '115200'
};

const Eol = {
    Warp: 'warp',
    NoWarp: 'noWarp'
};

const Mode = {
    Input: 'INPUT',
    Output: 'OUTPUT',
    InputPullup: 'INPUT_PULLUP'
};

const InterrupMode = {
    Rising: 'RISING',
    Falling: 'FALLING',
    Change: 'CHANGE',
    Low: 'LOW'
};

const DataType = {
    WholeNumber: 'WHOLE_NUMBER',
    Decimal: 'DECIMAL',
    String: 'STRING'
};

/**
 * Manage communication with a Arduino Leonardo peripheral over a OpenBlock Link client socket.
 */
class MakeyMakey{

    /**
     * Construct a Arduino communication object.
     * @param {Runtime} runtime - the OpenBlock runtime
     * @param {string} deviceId - the id of the extension
     */
    constructor (runtime, deviceId) {
        /**
         * The OpenBlock runtime used to trigger the green flag button.
         * @type {Runtime}
         * @private
         */
        this._runtime = runtime;

        /**
         * The serialport connection socket for reading/writing peripheral data.
         * @type {SERIALPORT}
         * @private
         */
        this._serialport = null;
        this._runtime.registerPeripheralExtension(deviceId, this);
        this._runtime.setRealtimeBaudrate(SERIAL_CONFIG.baudRate);

        /**
         * The id of the extension this peripheral belongs to.
         */
        this._deviceId = deviceId;

        /**
        * Pending data list. If busy is set when send, the data will push into this array to
        * waitting to be sended.
        */
        this._pendingData = [];

        this.reset = this.reset.bind(this);
        this._onConnect = this._onConnect.bind(this);
        this._onMessage = this._onMessage.bind(this);

        /**
         * Firmata connection.
         * @type {?Firmata}
         * @private
         */
        this._firmata = null;

        /**
         * Timeout ID for firmata get hartbeat timeout.
         * @type {number}
         * @private
         */
        this._firmataTimeoutID = null;

        /**
         * Interval ID for firmata send hartbeat.
         * @type {number}
         * @private
         */
        this._firmataIntervelID = null;

        /**
         * A flag that is true while firmata is conncted.
         * @type {boolean}
         * @private
         */
        this._isFirmataConnected = false;
    }

    /**
     * Called by the runtime when user wants to upload code to a peripheral.
     * @param {string} code - the code want to upload.
     */
    upload (code) {
        const base64Str = Buffer.from(code).toString('base64');
        this._serialport.upload(base64Str, DIVECE_OPT, 'base64');
    }

    /**
     * Called by the runtime when user wants to upload realtime firmware to a peripheral.
     */
    uploadFirmware () {
        this.stopHeartbeat();
        this._serialport.uploadFirmware(DIVECE_OPT);
    }

    /**
     * Called by the runtime when user wants to scan for a peripheral.
     * @param {Array.<string>} pnpidList - the array of pnp id list
     */
    scan (pnpidList) {
        if (this._serialport) {
            this._serialport.disconnect();
        }
        this._serialport = new Serialport(this._runtime, this._deviceId, {
            filters: {
                pnpid: pnpidList ? pnpidList : PNPID_LIST
            }
        }, this._onConnect, this.reset);
    }

    /**
     * Called by the runtime when user wants to connect to a certain peripheral.
     * @param {number} id - the id of the peripheral to connect to.
     * @param {?number} baudrate - the baudrate.
     */
    connect (id, baudrate = null) {
        const config = SERIAL_CONFIG;
        if (baudrate) {
            config.baudRate = baudrate;
        }
        if (this._serialport) {
            this._serialport.connectPeripheral(id, {config: config});
        }
    }

    /**
     * Disconnect from the peripheral.
     */
    disconnect () {
        if (this._serialport) {
            this._serialport.disconnect();
        }

        this.reset();
    }

    /**
     * Reset all the state and timeout/interval ids.
     */
    reset () {
        if (this._firmata) {
            this._firmata.removeListener('reportversion', this.listenHeartbeat.bind(this));
            delete this._firmata;
        }
        if (this._firmataTimeoutID) {
            window.clearTimeout(this._firmataTimeoutID);
            this._firmataTimeoutID = null;
        }
        if (this._firmataIntervelID) {
            window.clearInterval(this._firmataIntervelID);
            this._firmataIntervelID = null;
        }
        this._isFirmataConnected = false;
    }

    /**
     * Return true if connected to the peripheral.
     * @return {boolean} - whether the peripheral is connected.
     */
    isConnected () {
        let connected = false;
        if (this._serialport) {
            connected = this._serialport.isConnected();
        }
        return connected;
    }

    /**
     * Set baudrate of the peripheral serialport.
     * @param {number} baudrate - the baudrate.
     */
    setBaudrate (baudrate) {
        this._serialport.setBaudrate(baudrate);
    }

    /**
     * Write data to the peripheral serialport.
     * @param {string} data - the data to write.
     */
    write (data) {
        if (!this.isConnected()) return;

        const base64Str = data.toString('base64');
        this._serialport.write(base64Str, 'base64');
    }

    /**
     * Send a message to the peripheral Serialport socket.
     * @param {Uint8Array} message - the message to write
     */
    send (message) {
        if (!this.isConnected()) return;

        const data = Base64Util.uint8ArrayToBase64(message);
        this._serialport.write(data, 'base64');
    }

    /**
     * Start send/recive heartbeat timer.
     * @private
     */
    startHeartbeat () {
        this._isFirmataConnected = false;

        this._firmataIntervelID = window.setInterval(() => {
            if (this._runtime.getCurrentIsRealtimeMode()) {
                // Send reportVersion request as heartbeat.
                this._firmata.reportVersion(() => { });
            }
        }, FrimataHeartbeatInterval);
        // Start a timer if heartbeat time out means failed to connect firmata.
        this._firmataTimeoutID = window.setTimeout(() => {
            this._isFirmataConnected = false;
            if (this._runtime.getCurrentIsRealtimeMode()) {
                this._serialport.handleRealtimeDisconnectError(ConnectFirmataTimeout);
            }
        }, FrimataHeartbeatTimeout);
    }

    /**
     * Stop send/recive heartbeat timer.
     * @private
     */
    stopHeartbeat () {
        window.clearInterval(this._firmataIntervelID);
        this._firmataIntervelID = null;
        window.clearTimeout(this._firmataTimeoutID);
        this._firmataTimeoutID = null;
    }

    /**
     * Listen the heartbeat and emit connection state event.
     * @private
     */
    listenHeartbeat () {
        if (!this._isFirmataConnected) {
            this._isFirmataConnected = true;
            if (this._runtime.getCurrentIsRealtimeMode()) {
                this._serialport.handleRealtimeConnectSucess();
            }
        }
        // Reset the timeout timer
        window.clearTimeout(this._firmataTimeoutID);
        this._firmataTimeoutID = window.setTimeout(() => {
            this._isFirmataConnected = false;
            if (this._runtime.getCurrentIsRealtimeMode()) {
                this._serialport.handleRealtimeDisconnectError(ConnectFirmataTimeout);
            }
        }, FrimataHeartbeatTimeout);
    }

    /**
     * Starts reading data from peripheral after serialport has connected to it.
     * @private
     */
    _onConnect () {
        this._serialport.read(this._onMessage);
        this._firmata = new Firmata(this.send.bind(this));

        if (this._runtime.getCurrentIsRealtimeMode()) {
            this.startHeartbeat();
        }

        this._runtime.on(this._runtime.constructor.PROGRAM_MODE_UPDATE, data => {
            if (data.isRealtimeMode) {
                this.startHeartbeat();
            } else {
                this.stopHeartbeat();
            }
        });
        this._runtime.on(this._runtime.constructor.PERIPHERAL_UPLOAD_SUCCESS, () => {
            if (this._runtime.getCurrentIsRealtimeMode()) {
                this.startHeartbeat();
            }
        });
        // Start the heartbeat listener.
        this._firmata.on('reportversion', this.listenHeartbeat.bind(this));
    }

    /**
     * Process the sensor data from the incoming serialport characteristic.
     * @param {object} base64 - the incoming serialport data.
     * @private
     */
    _onMessage (base64) {
        // parse data
        const data = Base64Util.base64ToUint8Array(base64);
        this._firmata.onReciveData(data);
    }

    /**
     * Return true if peripheral has connected to firmata and program mode is realtime.
     * @return {boolean} - whether the peripheral is ready for realtime mode communication.
     */
    isReady () {
        if (this._runtime.getCurrentIsRealtimeMode() && this._isFirmataConnected) {
            return true;
        }
        return false;
    }

    /**
     * @param {PIN} pin - the pin string to parse.
     * @return {number} - the pin number.
     */
    parsePin (pin) {
        if (pin.charAt(0) === 'A') {
            return parseInt(pin.slice(1), 10) + 14;
        }
        return parseInt(pin, 10);
    }

    /**
     * @param {LEVEL} level - the level string to parse.
     * @return {number} - the level in number.
     */
    parseLevel (level) {
        if (level === Level.High) {
            return 1;
        }
        return 0;
    }

    /**
     * @param {PIN} pin - the pin to set.
     * @param {MODE} mode - the pin mode to set.
     */
    setPinMode (pin, mode) {
        if (this.isReady()) {
            pin = this.parsePin(pin);
            switch (mode) {
            case Mode.Input:
                mode = this._firmata.MODES.INPUT;
                break;
            case Mode.Output:
                mode = this._firmata.MODES.OUTPUT;
                break;
            case Mode.InputPullup:
                mode = this._firmata.MODES.PULLUP;
                break;
            }
            this._firmata.pinMode(pin, mode);
        }
    }

    /**
     * @param {PIN} pin - the pin to set.
     * @param {LEVEL} level - the pin level to set.
     */
    setDigitalOutput (pin, level) {
        if (this.isReady()) {
            pin = this.parsePin(pin);
            level = parseInt(level, 10);
            this._firmata.digitalWrite(pin, level);
        }
    }

    /**
     * @param {PIN} pin - the pin to set.
     * @param {VALUE} value - the pwm value to set.
     */
    setPwmOutput (pin, value) {
        if (this.isReady()) {
            pin = this.parsePin(pin);
            if (value < 0) {
                value = 0;
            }
            if (value > 255) {
                value = 255;
            }
            this._firmata.pinMode(pin, this._firmata.MODES.PWM);
            this._firmata.pwmWrite(pin, value);
        }
    }

    /**
     * @param {PIN} pin - the pin to read.
     * @return {Promise} - a Promise that resolves when read from peripheral.
     */
    readDigitalPin (pin) {
        if (this.isReady()) {
            pin = this.parsePin(pin);
            return new Promise(resolve => {
                this._firmata.digitalRead(pin, value => {
                    resolve(value);
                });
            });
        }
    }

    /**
     * @param {PIN} pin - the pin to read.
     * @return {Promise} - a Promise that resolves when read from peripheral.
     */
    readAnalogPin (pin) {
        if (this.isReady()) {
            pin = this.parsePin(pin);
            // Shifting to analog pin number.
            pin = pin - 14;
            this._firmata.pinMode(pin, this._firmata.MODES.ANALOG);
            return new Promise(resolve => {
                this._firmata.analogRead(pin, value => {
                    resolve(value);
                });
            });
        }
    }
}

/**
 * OpenBlock blocks to interact with a Arduino Leonardo peripheral.
 */
class OpenBlockMakeyMakeyDevice {
    /**
     * @return {string} - the ID of this extension.
     */
    static get DEVICE_ID () {
        return 'makeymakey';
    }

    get PINS_MENU () {
        return [
            {
                text: 'D0',
                value: Pins.D0
            },
            {
                text: 'D1',
                value: Pins.D1
            },
            {
                text: 'D2',
                value: Pins.D2
            },
            {
                text: 'D3',
                value: Pins.D3
            },
            {
                text: 'D4',
                value: Pins.D4
            },
            {
                text: 'D5',
                value: Pins.D5
            },
            {
                text: 'D14',
                value: Pins.D14
            },
            {
                text: 'D15',
                value: Pins.D15
            },
            {
                text: 'D16',
                value: Pins.D16
            },
            {
                text: 'A0',
                value: Pins.A0
            },
            {
                text: 'A1',
                value: Pins.A1
            },
            {
                text: 'A2',
                value: Pins.A2
            },
            {
                text: 'A3',
                value: Pins.A3
            },
            {
                text: 'A4',
                value: Pins.A4
            },
            {
                text: 'A5',
                value: Pins.A5
            }
        ];
    }

    get MODE_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'arduinoUno.modeMenu.input',
                    default: 'Input',
                    description: 'label for input pin mode'
                }),
                value: Mode.Input
            },
            {
                text: formatMessage({
                    id: 'arduinoUno.modeMenu.output',
                    default: 'Output',
                    description: 'label for output pin mode'
                }),
                value: Mode.Output
            },
            {
                text: formatMessage({
                    id: 'arduinoUno.modeMenu.inputPullup',
                    default: 'Input-pullup',
                    description: 'label for input-pullup pin mode'
                }),
                value: Mode.InputPullup
            }
        ];
    }

    get DIGITAL_PINS_MENU () {
        return [
            {
                text: 'D0',
                value: Pins.D0
            },
            {
                text: 'D1',
                value: Pins.D1
            },
            {
                text: 'D2',
                value: Pins.D2
            },
            {
                text: 'D3',
                value: Pins.D3
            },
            {
                text: 'D4',
                value: Pins.D4
            },
            {
                text: 'D5',
                value: Pins.D5
            },
            {
                text: 'D14',
                value: Pins.D14
            },
            {
                text: 'D15',
                value: Pins.D15
            },
            {
                text: 'D16',
                value: Pins.D16
            }
        ];
    }

    get ANALOG_PINS_MENU () {
        return [
            {
                text: 'A0',
                value: Pins.A0
            },
            {
                text: 'A1',
                value: Pins.A1
            },
            {
                text: 'A2',
                value: Pins.A2
            },
            {
                text: 'A3',
                value: Pins.A3
            },
            {
                text: 'A4',
                value: Pins.A4
            },
            {
                text: 'A5',
                value: Pins.A5
            }
        ];
    }

    get LEVEL_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'arduinoUno.levelMenu.high',
                    default: 'High',
                    description: 'label for high level'
                }),
                value: Level.High
            },
            {
                text: formatMessage({
                    id: 'arduinoUno.levelMenu.low',
                    default: 'Low',
                    description: 'label for low level'
                }),
                value: Level.Low
            }
        ];
    }

    get PWM_PINS_MENU () {
        return [
            {
                text: 'D3',
                value: Pins.D3
            }
        ];
    }

    get SERVO_PINS_MENU () {
        return [
            {
                text: 'D0',
                value: Pins.D0
            },
            {
                text: 'D1',
                value: Pins.D1
            },
            {
                text: 'D2',
                value: Pins.D2
            },
            {
                text: 'D3',
                value: Pins.D3
            },
            {
                text: 'D4',
                value: Pins.D4
            },
            {
                text: 'D5',
                value: Pins.D5
            }
        ];
    }

    get INTERRUPT_PINS_MENU () {
        return [
            {
                text: 'D0',
                value: Pins.D0
            },
            {
                text: 'D1',
                value: Pins.D1
            },
            {
                text: 'D2',
                value: Pins.D2
            },
            {
                text: 'D3',
                value: Pins.D3
            }
        ];
    }

    get INTERRUP_MODE_MENU () {
        return [
            {
                text: 'rising edge',
                value: InterrupMode.Rising
            },
            {
                text: 'falling edge',
                value: InterrupMode.Falling
            },
            {
                text: 'change edge',
                value: InterrupMode.Change
            },
            {
                text: 'low',
                value: InterrupMode.Low
            }
        ];
    }

    get BAUDTATE_MENU () {
        return [
            {
                text: '4800',
                value: Buadrate.B4800
            },
            {
                text: '9600',
                value: Buadrate.B9600
            },
            {
                text: '19200',
                value: Buadrate.B19200
            },
            {
                text: '38400',
                value: Buadrate.B38400
            },
            {
                text: '57600',
                value: Buadrate.B57600
            },
            {
                text: '115200',
                value: Buadrate.B115200
            }
        ];
    }

    get EOL_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'arduinoUno.eolMenu.warp',
                    default: 'Warp',
                    description: 'label for warp print'
                }),
                value: Eol.Warp
            },
            {
                text: formatMessage({
                    id: 'arduinoUno.eolMenu.noWarp',
                    default: 'No warp',
                    description: 'label for no warp print'
                }),
                value: Eol.NoWarp
            }
        ];
    }

    get DATA_TYPE_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'arduinoUno.dataTypeMenu.wholeNumber',
                    default: 'whole number',
                    description: 'label for whole number'
                }),
                value: DataType.WholeNumber
            },
            {
                text: formatMessage({
                    id: 'arduinoUno.dataTypeMenu.decimal',
                    default: 'decimal',
                    description: 'label for decimal number'
                }),
                value: DataType.Decimal
            },
            {
                text: formatMessage({
                    id: 'arduinoUno.dataTypeMenu.string',
                    default: 'string',
                    description: 'label for string'
                }),
                value: DataType.String
            }
        ];
    }

    /**
     * Construct a set of Arduino blocks.
     * @param {Runtime} runtime - the OpenBlock runtime.
     */
    constructor (runtime) {
        /**
         * The OpenBlock runtime.
         * @type {Runtime}
         */
        this.runtime = runtime;

        // Create a new Arduino Leonardo peripheral instance
        this._peripheral = new MakeyMakey(this.runtime, OpenBlockMakeyMakeyDevice.DEVICE_ID);
    }

    /**
     * @returns {Array.<object>} metadata for this extension and its blocks.
     */
    getInfo () {
        return [
            {
                id: 'pin',
                name: formatMessage({
                    id: 'arduinoUno.category.pins',
                    default: 'Pins',
                    description: 'The name of the arduino leonardo device pin category'
                }),
                color1: '#4C97FF',
                color2: '#3373CC',
                color3: '#3373CC',

                blocks: [
                    {
                        opcode: 'setPinMode',
                        text: formatMessage({
                            id: 'arduinoUno.pins.setPinMode',
                            default: 'set pin [PIN] mode [MODE]',
                            description: 'makeymakey set pin mode'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'pins',
                                defaultValue: Pins.D0
                            },
                            MODE: {
                                type: ArgumentType.STRING,
                                menu: 'mode',
                                defaultValue: Mode.Input
                            }
                        }
                    },
                    {
                        opcode: 'setDigitalOutput',
                        text: formatMessage({
                            id: 'arduinoUno.pins.setDigitalOutput',
                            default: 'set digital pin [PIN] out [LEVEL]',
                            description: 'makeymakey set digital pin out'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'pins',
                                defaultValue: Pins.D0
                            },
                            LEVEL: {
                                type: ArgumentType.STRING,
                                menu: 'level',
                                defaultValue: Level.High
                            }
                        }
                    },
                    {

                        opcode: 'setPwmOutput',
                        text: formatMessage({
                            id: 'arduinoUno.pins.setPwmOutput',
                            default: 'set pwm pin [PIN] out [OUT]',
                            description: 'makeymakey set pwm pin out'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'pwmPins',
                                defaultValue: Pins.D3
                            },
                            OUT: {
                                type: ArgumentType.NUMBER,
                                defaultValue: '0'
                            }
                        }
                    },
                    '---',
                    {
                        opcode: 'readDigitalPin',
                        text: formatMessage({
                            id: 'arduinoUno.pins.readDigitalPin',
                            default: 'read digital pin [PIN]',
                            description: 'makeymakey read digital pin'
                        }),
                        blockType: BlockType.BOOLEAN,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'digitalPins',
                                defaultValue: Pins.D0
                            }
                        }
                    },
                    {
                        opcode: 'readAnalogPin',
                        text: formatMessage({
                            id: 'arduinoUno.pins.readAnalogPin',
                            default: 'read analog pin [PIN]',
                            description: 'makeymakey read analog pin'
                        }),
                        blockType: BlockType.REPORTER,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'analogPins',
                                defaultValue: Pins.A0
                            }
                        }
                    },
                    '---',
                    {

                        opcode: 'setServoOutput',
                        text: formatMessage({
                            id: 'arduinoUno.pins.setServoOutput',
                            default: 'set servo pin [PIN] out [OUT]',
                            description: 'makeymakey set servo pin out'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'servoPins',
                                defaultValue: Pins.D3
                            },
                            OUT: {
                                type: ArgumentType.ANGLE,
                                defaultValue: '0'
                            }
                        }
                    },
                    '---',
                    {

                        opcode: 'attachInterrupt',
                        text: formatMessage({
                            id: 'arduinoUno.pins.attachInterrupt',
                            default: 'attach interrupt pin [PIN] mode [MODE] executes',
                            description: 'makeymakey attach interrupt'
                        }),
                        blockType: BlockType.CONDITIONAL,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'interruptPins',
                                defaultValue: Pins.D3
                            },
                            MODE: {
                                type: ArgumentType.STRING,
                                menu: 'interruptMode',
                                defaultValue: InterrupMode.Rising
                            }
                        },
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {

                        opcode: 'detachInterrupt',
                        text: formatMessage({
                            id: 'arduinoUno.pins.detachInterrupt',
                            default: 'detach interrupt pin [PIN]',
                            description: 'makeymakey detach interrupt'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'interruptPins',
                                defaultValue: Pins.D3
                            }
                        },
                        programMode: [ProgramModeType.UPLOAD]
                    }
                ],
                menus: {
                    pins: {
                        items: this.PINS_MENU
                    },
                    mode: {
                        items: this.MODE_MENU
                    },
                    digitalPins: {
                        items: this.DIGITAL_PINS_MENU
                    },
                    analogPins: {
                        items: this.ANALOG_PINS_MENU
                    },
                    level: {
                        acceptReporters: true,
                        items: this.LEVEL_MENU
                    },
                    pwmPins: {
                        items: this.PWM_PINS_MENU
                    },
                    servoPins: {
                        items: this.SERVO_PINS_MENU
                    },
                    interruptPins: {
                        items: this.INTERRUPT_PINS_MENU
                    },
                    interruptMode: {
                        items: this.INTERRUP_MODE_MENU
                    }
                }
            },
            {
                id: 'serial',
                name: formatMessage({
                    id: 'arduinoUno.category.serial',
                    default: 'Serial',
                    description: 'The name of the arduino leonardo device serial category'
                }),
                color1: '#9966FF',
                color2: '#774DCB',
                color3: '#774DCB',

                blocks: [
                    {
                        opcode: 'serialBegin',
                        text: formatMessage({
                            id: 'arduinoUno.serial.serialBegin',
                            default: 'serial begin baudrate [VALUE]',
                            description: 'makeymakey serial begin'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            VALUE: {
                                type: ArgumentType.STRING,
                                menu: 'baudrate',
                                defaultValue: Buadrate.B9600
                            }
                        },
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {
                        opcode: 'serialPrint',
                        text: formatMessage({
                            id: 'arduinoUno.serial.serialPrint',
                            default: 'serial print [VALUE] [EOL]',
                            description: 'makeymakey serial print'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            VALUE: {
                                type: ArgumentType.STRING,
                                defaultValue: 'hello'
                            },
                            EOL: {
                                type: ArgumentType.STRING,
                                menu: 'eol',
                                defaultValue: Eol.Warp
                            }
                        },
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {
                        opcode: 'serialAvailable',
                        text: formatMessage({
                            id: 'arduinoUno.serial.serialAvailable',
                            default: 'serial available data length',
                            description: 'makeymakey serial available data length'
                        }),
                        blockType: BlockType.REPORTER,
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {
                        opcode: 'serialReadData',
                        text: formatMessage({
                            id: 'arduinoUno.serial.serialReadData',
                            default: 'serial read data',
                            description: 'makeymakey serial read data'
                        }),
                        blockType: BlockType.REPORTER,
                        programMode: [ProgramModeType.UPLOAD]
                    }
                ],
                menus: {
                    baudrate: {
                        items: this.BAUDTATE_MENU
                    },
                    eol: {
                        items: this.EOL_MENU
                    }
                }
            },
            {
                id: 'data',
                name: formatMessage({
                    id: 'arduinoUno.category.data',
                    default: 'Data',
                    description: 'The name of the arduino uno device data category'
                }),
                color1: '#CF63CF',
                color2: '#C94FC9',
                color3: '#BD42BD',
                blocks: [
                    {
                        opcode: 'dataMap',
                        text: formatMessage({
                            id: 'arduinoUno.data.dataMap',
                            default: 'map [DATA] from ([ARG0], [ARG1]) to ([ARG2], [ARG3])',
                            description: 'makeymakey data map'
                        }),
                        blockType: BlockType.REPORTER,
                        arguments: {
                            DATA: {
                                type: ArgumentType.NUMBER,
                                defaultValue: '50'
                            },
                            ARG0: {
                                type: ArgumentType.NUMBER,
                                defaultValue: '1'
                            },
                            ARG1: {
                                type: ArgumentType.NUMBER,
                                defaultValue: '100'
                            },
                            ARG2: {
                                type: ArgumentType.NUMBER,
                                defaultValue: '1'
                            },
                            ARG3: {
                                type: ArgumentType.NUMBER,
                                defaultValue: '1000'
                            }
                        },
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    '---',
                    {
                        opcode: 'dataConstrain',
                        text: formatMessage({
                            id: 'arduinoUno.data.dataConstrain',
                            default: 'constrain [DATA] between ([ARG0], [ARG1])',
                            description: 'makeymakey data constrain'
                        }),
                        blockType: BlockType.REPORTER,
                        arguments: {
                            DATA: {
                                type: ArgumentType.NUMBER,
                                defaultValue: '50'
                            },
                            ARG0: {
                                type: ArgumentType.NUMBER,
                                defaultValue: '1'
                            },
                            ARG1: {
                                type: ArgumentType.NUMBER,
                                defaultValue: '100'
                            }
                        },
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {
                        opcode: 'dataConvert',
                        text: formatMessage({
                            id: 'arduinoUno.data.dataConvert',
                            default: 'convert [DATA] to [TYPE]',
                            description: 'makeymakey data convert'
                        }),
                        blockType: BlockType.REPORTER,
                        arguments: {
                            DATA: {
                                type: ArgumentType.STRING,
                                defaultValue: '123'
                            },
                            TYPE: {
                                type: ArgumentType.STRING,
                                menu: 'dataType',
                                defaultValue: DataType.WholeNumber
                            }
                        },
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {
                        opcode: 'dataConvertASCIICharacter',
                        text: formatMessage({
                            id: 'arduinoUno.data.dataConvertASCIICharacter',
                            default: 'convert [DATA] to ASCII character',
                            description: 'makeymakey data convert to ASCII character'
                        }),
                        blockType: BlockType.REPORTER,
                        arguments: {
                            DATA: {
                                type: ArgumentType.NUMBER,
                                defaultValue: '97'
                            }
                        },
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {
                        opcode: 'dataConvertASCIINumber',
                        text: formatMessage({
                            id: 'arduinoUno.data.dataConvertASCIINumber',
                            default: 'convert [DATA] to ASCII nubmer',
                            description: 'makeymakey data convert to ASCII nubmer'
                        }),
                        blockType: BlockType.REPORTER,
                        arguments: {
                            DATA: {
                                type: ArgumentType.STRING,
                                defaultValue: 'a'
                            }
                        },
                        programMode: [ProgramModeType.UPLOAD]
                    }
                ],
                menus: {
                    dataType: {
                        items: this.DATA_TYPE_MENU
                    }
                }
            }
        ];
    }

    /**
     * Set pin mode.
     * @param {object} args - the block's arguments.
     * @return {Promise} - a Promise that resolves after the set pin mode is done.
     */
    setPinMode (args) {
        this._peripheral.setPinMode(args.PIN, args.MODE);
        return Promise.resolve();
    }

    /**
     * Set pin digital out level.
     * @param {object} args - the block's arguments.
     * @return {Promise} - a Promise that resolves after the set pin digital out level is done.
     */
    setDigitalOutput (args) {
        this._peripheral.setDigitalOutput(args.PIN, args.LEVEL);
        return Promise.resolve();
    }

    /**
     * Set pin pwm out value.
     * @param {object} args - the block's arguments.
     * @return {Promise} - a Promise that resolves after the set pin pwm out value is done.
     */
    setPwmOutput (args) {
        this._peripheral.setPwmOutput(args.PIN, args.OUT);
        return Promise.resolve();
    }

    /**
     * Read pin digital level.
     * @param {object} args - the block's arguments.
     * @return {boolean} - true if read high level, false if read low level.
     */
    readDigitalPin (args) {
        return this._peripheral.readDigitalPin(args.PIN);
    }

    /**
     * Read analog pin.
     * @param {object} args - the block's arguments.
     * @return {number} - analog value fo the pin.
     */
    readAnalogPin (args) {
        return this._peripheral.readAnalogPin(args.PIN);
    }
}

module.exports = OpenBlockMakeyMakeyDevice;

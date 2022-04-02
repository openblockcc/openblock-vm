const formatMessage = require('format-message');

const ArgumentType = require('../../extension-support/argument-type');
const BlockType = require('../../extension-support/block-type');
const ProgramModeType = require('../../extension-support/program-mode-type');

const CommonPeripheral = require('../common/common-peripheral');

/**
 * The list of USB device filters.
 * @readonly
 */
const PNPID_LIST = [
    // CH340
    'USB\\VID_1A86&PID_7523',
    // CH9102
    'USB\\VID_1A86&PID_55D4',
    // CP2102
    'USB\\VID_10C4&PID_EA60'
];

/**
 * Configuration of serialport
 * @readonly
 */
const SERIAL_CONFIG = {
    baudRate: 57600,
    dataBits: 8,
    stopBits: 1,
    rtscts: true
};

/**
 * Configuration for arduino-cli. TODO:  replace
 * @readonly
 */
const DIVECE_OPT = {
    type: 'microPython',
    // fqbn: 'esp32:esp32:esp32'
};

const Pins = {
    IO0: '0',
    IO1: '1',
    IO2: '2',
    IO3: '3',
    IO4: '4',
    IO5: '5',
    IO6: '6',
    IO7: '7',
    IO8: '8',
    IO9: '9',
    IO10: '10',
    IO11: '11',
    IO12: '12',
    IO13: '13',
    IO14: '14',
    IO15: '15',
    IO16: '16',
    IO17: '17',
    IO18: '18',
    IO19: '19',
    IO21: '21',
    IO22: '22',
    IO23: '23',
    IO25: '25',
    IO26: '26',
    IO27: '27',
    IO32: '32',
    IO33: '33',
    IO34: '34',
    IO35: '35',
    IO36: '36',
    IO39: '39'
};

const Level = {
    High: '1',
    Low: '0'
};

const Voltage = {
    V33: 'ATTN_11DB',
    V22: 'ATTN_6DB',
    V15: 'ATTN_2_5_DB',
    V12: 'ATTIN_0DB',
};

const SerialNo = {
    Serial1: '1',
    Serial2: '2'
};

const Buadrate = {
    B1200: '1200',
    B2300: '2400',
    B4800: '4800',
    B9600: '9600',
    B19200: '19200',
    B38400: '38400',
    B57600: '57600',
    B76800: '76800',
    B115200: '115200'
};

const Eol = {
    Warp: 'warp',
    NoWarp: 'noWarp'
};

const Mode = {
    Input: 'IN',
    Output: 'OUT',
    InputPullup: 'PULL_UP',
    InputPulldown: 'PULL_DOWN'
};

const InterrupMode = {
    Rising: '1',
    Falling: '2',
    Change: '3'
};

/**
 * Manage communication with a MicroPython esp32 peripheral over a OpenBlock Link client socket.
 */
class MicroPythonEsp32 extends CommonPeripheral{
    /**
     * Construct a MicroPython communication object.
     * @param {Runtime} runtime - the OpenBlock runtime
     * @param {string} deviceId - the id of the extension
     * @param {string} originalDeviceId - the original id of the peripheral, like xxx_arduinoUno
     */
    constructor (runtime, deviceId, originalDeviceId) {
        super(runtime, deviceId, originalDeviceId, PNPID_LIST, SERIAL_CONFIG, DIVECE_OPT);
    }
}

/**
 * OpenBlock blocks to interact with a MicroPython esp32 peripheral.
 */
class OpenBlockMicroPythonEsp32Device {
    /**
     * @return {string} - the ID of this extension.
     */
    static get DEVICE_ID () {
        return 'microPythonEsp32';
    }

    get PINS_MENU () {
        return [
            {
                text: 'IO0',
                value: Pins.IO0
            },
            {
                text: 'IO1',
                value: Pins.IO1
            },
            {
                text: 'IO2',
                value: Pins.IO2
            },
            {
                text: 'IO3',
                value: Pins.IO3
            },
            {
                text: 'IO4',
                value: Pins.IO4
            },
            {
                text: 'IO5',
                value: Pins.IO5
            },
            {
                text: 'IO6',
                value: Pins.IO6
            },
            {
                text: 'IO7',
                value: Pins.IO7
            },
            {
                text: 'IO8',
                value: Pins.IO8
            },
            {
                text: 'IO9',
                value: Pins.IO9
            },
            {
                text: 'IO10',
                value: Pins.IO10
            },
            {
                text: 'IO11',
                value: Pins.IO11
            },
            {
                text: 'IO12',
                value: Pins.IO12
            },
            {
                text: 'IO13',
                value: Pins.IO13
            },
            {
                text: 'IO14',
                value: Pins.IO14
            },
            {
                text: 'IO15',
                value: Pins.IO15
            },
            {
                text: 'IO16',
                value: Pins.IO16
            },
            {
                text: 'IO17',
                value: Pins.IO17
            },
            {
                text: 'IO18',
                value: Pins.IO18
            },
            {
                text: 'IO19',
                value: Pins.IO19
            },
            {
                text: 'IO21',
                value: Pins.IO21
            },
            {
                text: 'IO22',
                value: Pins.IO22
            },
            {
                text: 'IO23',
                value: Pins.IO23
            },
            {
                text: 'IO25',
                value: Pins.IO25
            },
            {
                text: 'IO26',
                value: Pins.IO26
            },
            {
                text: 'IO27',
                value: Pins.IO27
            },
            {
                text: 'IO32',
                value: Pins.IO32
            },
            {
                text: 'IO33',
                value: Pins.IO33
            },
            {
                text: 'IO34',
                value: Pins.IO34
            },
            {
                text: 'IO35',
                value: Pins.IO35
            },
            {
                text: 'IO36',
                value: Pins.IO36
            },
            {
                text: 'IO39',
                value: Pins.IO39
            }
        ];
    }

    
    get OUT_PINS_MENU () {
        return [
            {
                text: 'IO0',
                value: Pins.IO0
            },
            {
                text: 'IO1',
                value: Pins.IO1
            },
            {
                text: 'IO2',
                value: Pins.IO2
            },
            {
                text: 'IO3',
                value: Pins.IO3
            },
            {
                text: 'IO4',
                value: Pins.IO4
            },
            {
                text: 'IO5',
                value: Pins.IO5
            },
            {
                text: 'IO6',
                value: Pins.IO6
            },
            {
                text: 'IO7',
                value: Pins.IO7
            },
            {
                text: 'IO8',
                value: Pins.IO8
            },
            {
                text: 'IO9',
                value: Pins.IO9
            },
            {
                text: 'IO10',
                value: Pins.IO10
            },
            {
                text: 'IO11',
                value: Pins.IO11
            },
            {
                text: 'IO12',
                value: Pins.IO12
            },
            {
                text: 'IO13',
                value: Pins.IO13
            },
            {
                text: 'IO14',
                value: Pins.IO14
            },
            {
                text: 'IO15',
                value: Pins.IO15
            },
            {
                text: 'IO16',
                value: Pins.IO16
            },
            {
                text: 'IO17',
                value: Pins.IO17
            },
            {
                text: 'IO18',
                value: Pins.IO18
            },
            {
                text: 'IO19',
                value: Pins.IO19
            },
            {
                text: 'IO21',
                value: Pins.IO21
            },
            {
                text: 'IO22',
                value: Pins.IO22
            },
            {
                text: 'IO23',
                value: Pins.IO23
            },
            {
                text: 'IO25',
                value: Pins.IO25
            },
            {
                text: 'IO26',
                value: Pins.IO26
            },
            {
                text: 'IO27',
                value: Pins.IO27
            },
            {
                text: 'IO32',
                value: Pins.IO32
            },
            {
                text: 'IO33',
                value: Pins.IO33
            }
        ];
    }


    get MODE_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'microPythonEsp32.modeMenu.input',
                    default: 'input',
                    description: 'label for input pin mode'
                }),
                value: Mode.Input
            },
            {
                text: formatMessage({
                    id: 'microPythonEsp32.modeMenu.output',
                    default: 'output',
                    description: 'label for output pin mode'
                }),
                value: Mode.Output
            },
            {
                text: formatMessage({
                    id: 'microPythonEsp32.modeMenu.inputPullup',
                    default: 'input pull up',
                    description: 'label for input-pullup pin mode'
                }),
                value: Mode.InputPullup
            },
            {
                text: formatMessage({
                    id: 'microPythonEsp32.modeMenu.inputPulldown',
                    default: 'input pull down',
                    description: 'label for input-pulldown pin mode'
                }),
                value: Mode.InputPulldown
            }
        ];
    }

    get PWM_PINS_MENU () {
        return [
            {
                text: 'IO0',
                value: Pins.IO0
            },
            {
                text: 'IO2',
                value: Pins.IO2
            },
            {
                text: 'IO4',
                value: Pins.IO4
            },
            {
                text: 'IO5',
                value: Pins.IO5
            },
            {
                text: 'IO12',
                value: Pins.IO12
            },
            {
                text: 'IO13',
                value: Pins.IO13
            },
            {
                text: 'IO14',
                value: Pins.IO14
            },
            {
                text: 'IO15',
                value: Pins.IO15
            },
            {
                text: 'IO16',
                value: Pins.IO16
            },
            {
                text: 'IO17',
                value: Pins.IO17
            },
            {
                text: 'IO18',
                value: Pins.IO18
            },
            {
                text: 'IO19',
                value: Pins.IO19
            },
            {
                text: 'IO21',
                value: Pins.IO21
            },
            {
                text: 'IO22',
                value: Pins.IO22
            },
            {
                text: 'IO23',
                value: Pins.IO23
            },
            {
                text: 'IO25',
                value: Pins.IO25
            },
            {
                text: 'IO26',
                value: Pins.IO26
            },
            {
                text: 'IO27',
                value: Pins.IO27
            },
            {
                text: 'IO32',
                value: Pins.IO32
            }
        ];
    }

    get ANALOG_PINS_MENU () {
        return [
            {
                text: 'IO0',
                value: Pins.IO0
            },
            {
                text: 'IO2',
                value: Pins.IO2
            },
            {
                text: 'IO4',
                value: Pins.IO4
            },
            {
                text: 'IO12',
                value: Pins.IO12
            },
            {
                text: 'IO13',
                value: Pins.IO13
            },
            {
                text: 'IO14',
                value: Pins.IO14
            },
            {
                text: 'IO15',
                value: Pins.IO15
            },
            {
                text: 'IO25',
                value: Pins.IO25
            },
            {
                text: 'IO26',
                value: Pins.IO26
            },
            {
                text: 'IO27',
                value: Pins.IO27
            },
            {
                text: 'IO32',
                value: Pins.IO32
            },
            {
                text: 'IO33',
                value: Pins.IO33
            },
            {
                text: 'IO34',
                value: Pins.IO34
            },
            {
                text: 'IO35',
                value: Pins.IO35
            },
            {
                text: 'IO36',
                value: Pins.IO36
            },
            {
                text: 'IO39',
                value: Pins.IO39
            }
        ];
    }

    get VOLTAGE_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'microPythonEsp32.voltageMenu.v33',
                    default: '3.3V'
                }),
                value: Voltage.V33
            },
            {
                text: formatMessage({
                    id: 'microPythonEsp32.voltageMenu.v22',
                    default: '2.2V'
                }),
                value: Voltage.V22
            },
            {
                text: formatMessage({
                    id: 'microPythonEsp32.voltageMenu.v15',
                    default: '1.5V'
                }),
                value: Voltage.V15
            },
            {
                text: formatMessage({
                    id: 'microPythonEsp32.voltageMenu.v12',
                    default: '1.2V'
                }),
                value: Voltage.V12
            }
        ];
    }

    get LEVEL_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'microPythonEsp32.levelMenu.high',
                    default: 'high',
                    description: 'label for high level'
                }),
                value: Level.High
            },
            {
                text: formatMessage({
                    id: 'microPythonEsp32.levelMenu.low',
                    default: 'low',
                    description: 'label for low level'
                }),
                value: Level.Low
            }
        ];
    }

    get DAC_PINS_MENU () {
        return [
            {
                text: 'IO25',
                value: Pins.IO25
            },
            {
                text: 'IO26',
                value: Pins.IO26
            }
        ];
    }

    get ADC_PINS_MENU () {
        return [
            {
                text: 'ADC32',
                value: Pins.IO32
            },
            {
                text: 'ADC33',
                value: Pins.IO33
            },
            {
                text: 'ADC34',
                value: Pins.IO34
            },
            {
                text: 'ADC35',
                value: Pins.IO35
            },
            {
                text: 'ADC36',
                value: Pins.IO36
            },
            {
                text: 'ADC39',
                value: Pins.IO39
            }
        ];
    }

    get TOUCH_PINS_MENU () {
        return [
            {
                text: 'IO0',
                value: Pins.IO0
            },
            {
                text: 'IO2',
                value: Pins.IO2
            },
            {
                text: 'IO4',
                value: Pins.IO4
            },
            {
                text: 'IO12',
                value: Pins.IO12
            },
            {
                text: 'IO13',
                value: Pins.IO13
            },
            {
                text: 'IO14',
                value: Pins.IO14
            },
            {
                text: 'IO15',
                value: Pins.IO15
            },
            {
                text: 'IO27',
                value: Pins.IO27
            },
            {
                text: 'IO32',
                value: Pins.IO32
            },
            {
                text: 'IO33',
                value: Pins.IO33
            }
        ];
    }

    get INTERRUP_MODE_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'microPythonEsp32.InterrupModeMenu.risingEdge',
                    default: 'rising edge',
                    description: 'label for rising edge interrup'
                }),
                value: InterrupMode.Rising
            },
            {
                text: formatMessage({
                    id: 'microPythonEsp32.InterrupModeMenu.fallingEdge',
                    default: 'falling edge',
                    description: 'label for falling edge interrup'
                }),
                value: InterrupMode.Falling
            },
            {
                text: formatMessage({
                    id: 'microPythonEsp32.InterrupModeMenu.changeEdge',
                    default: 'change edge',
                    description: 'label for change edge interrup'
                }),
                value: InterrupMode.Change
            }
        ];
    }

    get SERIAL_NO_MENU () {
        return [
            {
                text: 'uart1',
                value: SerialNo.Serial1
            },
            {
                text: 'uart2',
                value: SerialNo.Serial2
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
                text: '76800',
                value: Buadrate.B76800
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
                    id: 'microPythonEsp32.eolMenu.warp',
                    default: 'warp',
                    description: 'label for warp print'
                }),
                value: Eol.Warp
            },
            {
                text: formatMessage({
                    id: 'microPythonEsp32.eolMenu.noWarp',
                    default: 'no-warp',
                    description: 'label for no warp print'
                }),
                value: Eol.NoWarp
            }
        ];
    }

    /**
     * Construct a set of MicroPython blocks.
     * @param {Runtime} runtime - the OpenBlock runtime.
     * @param {string} originalDeviceId - the original id of the peripheral, like xxx_arduinoUno
     */
    constructor (runtime, originalDeviceId) {
        /**
         * The OpenBlock runtime.
         * @type {Runtime}
         */
        this.runtime = runtime;

        // Create a new MicroPython esp32 peripheral instance
        this._peripheral = new MicroPythonEsp32(this.runtime,
            OpenBlockMicroPythonEsp32Device.DEVICE_ID, originalDeviceId);
    }

    /**
     * @returns {Array.<object>} metadata for this extension and its blocks.
     */
    getInfo () {
        return [
            {
                id: 'pin',
                name: formatMessage({
                    id: 'microPythonEsp32.category.pins',
                    default: 'Pins',
                    description: 'The name of the esp32 microPython device pin category'
                }),
                color1: '#4C97FF',
                color2: '#3373CC',
                color3: '#3373CC',

                blocks: [
                    {
                        opcode: 'esp32InitPinMode',
                        text: formatMessage({
                            id: 'microPythonEsp32.pins.esp32InitPinMode',
                            default: 'set pin [PIN] mode [MODE]',
                            description: 'microPythonEsp32 set pin mode'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'outPins',
                                defaultValue: Pins.IO0
                            },
                            MODE: {
                                type: ArgumentType.STRING,
                                menu: 'mode',
                                defaultValue: Mode.Input
                            }
                        }
                    },
                    {
                        opcode: 'esp32SetDigitalOutput',
                        text: formatMessage({
                            id: 'microPythonEsp32.pins.esp32SetDigitalOutput',
                            default: 'set digital pin [PIN] out [LEVEL]',
                            description: 'microPythonEsp32 set digital pin out'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'outPins',
                                defaultValue: Pins.IO0
                            },
                            LEVEL: {
                                type: ArgumentType.STRING,
                                menu: 'level',
                                defaultValue: Level.High
                            }
                        }
                    },
                    {
                        opcode: 'esp32SetPwmOutput',
                        text: formatMessage({
                            id: 'microPythonEsp32.pins.esp32SetPwmOutput',
                            default: 'set pwm pin [PIN] out [OUT]',
                            description: 'microPythonEsp32 set pwm pin out'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'outPins',
                                defaultValue: Pins.IO4
                            },
                            OUT: {
                                type: ArgumentType.NUMBER,
                                defaultValue: '0'
                            }
                        }
                    },
                    {
                        opcode: 'esp32SetPwmFrequency',
                        text: formatMessage({
                            id: 'microPythonEsp32.pins.esp32SetPwmFrequency',
                            default: 'set pwm pin [PIN] frequency [FREQ]',
                            description: 'microPythonEsp32 set pwm pin out'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'outPins',
                                defaultValue: Pins.IO4
                            },
                            FREQ: {
                                type: ArgumentType.NUMBER,
                                defaultValue: '2000'
                            }
                        }
                    },
                    {
                        opcode: 'esp32SetDACOutput',
                        text: formatMessage({
                            id: 'microPythonEsp32.pins.esp32SetDACOutput',
                            default: 'set dac pin [PIN] out [OUT]',
                            description: 'microPythonEsp32 set dac pin out'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'dacPins',
                                defaultValue: Pins.IO25
                            },
                            OUT: {
                                type: ArgumentType.NUMBER,
                                defaultValue: '0'
                            }
                        }
                    },
                    {
                        opcode: 'esp32SetAnalogVoltage',
                        text: formatMessage({
                            id: 'microPythonEsp32.pins.esp32SetAnalogVoltage',
                            default: 'set analog pin [PIN] reference voltage [VOLTAGE]',
                            description: 'microPythonEsp32 set analog pin reference voltage'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'adcPins',
                                defaultValue: Pins.IO32
                            },
                            VOLTAGE: {
                                type: ArgumentType.STRING,
                                menu: 'voltage',
                                defaultValue: Voltage.V33
                            }
                        }
                    },
                    '---',
                    {
                        opcode: 'esp32ReadDigitalPin',
                        text: formatMessage({
                            id: 'microPythonEsp32.pins.esp32ReadDigitalPin',
                            default: 'read digital pin [PIN]',
                            description: 'microPythonEsp32 read digital pin'
                        }),
                        blockType: BlockType.BOOLEAN,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'pins',
                                defaultValue: Pins.IO2
                            }
                        }
                    },
                    {
                        opcode: 'esp32ReadAnalogPin',
                        text: formatMessage({
                            id: 'microPythonEsp32.pins.esp32ReadAnalogPin',
                            default: 'read analog pin [PIN]',
                            description: 'microPythonEsp32 read analog pin'
                        }),
                        blockType: BlockType.REPORTER,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'adcPins',
                                defaultValue: Pins.IO32
                            }
                        }
                    },
                    {
                        opcode: 'esp32ReadTouchPin',
                        text: formatMessage({
                            id: 'microPythonEsp32.pins.esp32ReadTouchPin',
                            default: 'read touch pin [PIN]',
                            description: 'microPythonEsp32 read touch pin'
                        }),
                        blockType: BlockType.REPORTER,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'touchPins',
                                defaultValue: Pins.IO2
                            }
                        }
                    },
                    '---',
                    {
                        opcode: 'esp32SetServoOutput',
                        text: formatMessage({
                            id: 'microPythonEsp32.pins.setServoOutput',
                            default: 'set servo pin [PIN] out [OUT]',
                            description: 'microPythonEsp32 set servo pin out'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'outPins',
                                defaultValue: Pins.IO2
                            },
                            OUT: {
                                type: ArgumentType.ANGLE,
                                defaultValue: '0'
                            }
                        }
                    },
                    '---',
                    {

                        opcode: 'esp32AttachInterrupt',
                        text: formatMessage({
                            id: 'microPythonEsp32.pins.esp32AttachInterrupt',
                            default: 'attach interrupt pin [PIN] mode [MODE] executes',
                            description: 'microPythonEsp32 attach interrupt'
                        }),
                        blockType: BlockType.CONDITIONAL,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'pins',
                                defaultValue: Pins.IO2
                            },
                            MODE: {
                                type: ArgumentType.STRING,
                                menu: 'interruptMode',
                                defaultValue: InterrupMode.Rising
                            }
                        },
                        programMode: [ProgramModeType.UPLOAD]
                    }
                ],
                menus: {
                    pins: {
                        items: this.PINS_MENU
                    },
                    outPins: {
                        items: this.OUT_PINS_MENU
                    },
                    mode: {
                        items: this.MODE_MENU
                    },
                    pwmPins: {
                        items: this.PWM_PINS_MENU
                    },
                    voltage: {
                        items: this.VOLTAGE_MENU
                    },
                    level: {
                        acceptReporters: true,
                        items: this.LEVEL_MENU
                    },
                    dacPins: {
                        items: this.DAC_PINS_MENU
                    },
                    adcPins: {
                        items: this.ADC_PINS_MENU
                    },
                    touchPins: {
                        items: this.TOUCH_PINS_MENU
                    },
                    interruptMode: {
                        items: this.INTERRUP_MODE_MENU
                    }
                }
            },
            {
                id: 'serial',
                name: formatMessage({
                    id: 'microPythonEsp32.category.serial',
                    default: 'Serial',
                    description: 'The name of the microPython esp32 device serial category'
                }),
                color1: '#9966FF',
                color2: '#774DCB',
                color3: '#774DCB',

                blocks: [
                    {
                        opcode: 'esp32SerialBegin',
                        text: formatMessage({
                            id: 'microPythonEsp32.serial.esp32SerialBegin',
                            default: 'serial [NO] begin baudrate [VALUE]',
                            description: 'microPythonEsp32 multi serial begin'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            NO: {
                                type: ArgumentType.NUMBER,
                                menu: 'serialNo',
                                defaultValue: SerialNo.Serial1
                            },
                            VALUE: {
                                type: ArgumentType.STRING,
                                menu: 'baudrate',
                                defaultValue: Buadrate.B115200
                            }
                        },
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {
                        opcode: 'esp32SerialPrint',
                        text: formatMessage({
                            id: 'microPythonEsp32.serial.esp32SerialPrint',
                            default: 'serial [NO] print [VALUE] [EOL]',
                            description: 'microPythonEsp32 multi serial print'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            NO: {
                                type: ArgumentType.NUMBER,
                                menu: 'serialNo',
                                defaultValue: SerialNo.Serial1
                            },
                            VALUE: {
                                type: ArgumentType.STRING,
                                defaultValue: 'Hello OpenBlock'
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
                        opcode: 'esp32SerialAvailable',
                        text: formatMessage({
                            id: 'microPythonEsp32.serial.esp32SerialAvailable',
                            default: 'serial [NO]  is available?',
                            description: 'microPythonEsp32 multi serial available data length'
                        }),
                        arguments: {
                            NO: {
                                type: ArgumentType.NUMBER,
                                menu: 'serialNo',
                                defaultValue: SerialNo.Serial1
                            }
                        },
                        blockType: BlockType.BOOLEAN,
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {
                        opcode: 'esp32SerialReadString',
                        text: formatMessage({
                            id: 'microPythonEsp32.serial.esp32SerialReadString',
                            default: 'serial [NO] read string',
                            description: 'microPythonEsp32 multi serial read string'
                        }),
                        arguments: {
                            NO: {
                                type: ArgumentType.NUMBER,
                                menu: 'serialNo',
                                defaultValue: SerialNo.Serial1
                            }
                        },
                        blockType: BlockType.REPORTER,
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {
                        opcode: 'esp32SerialReadARow',
                        text: formatMessage({
                            id: 'microPythonEsp32.serial.esp32SerialReadARow',
                            default: 'serial [NO] read a row',
                            description: 'microPythonEsp32 multi serial read a row'
                        }),
                        arguments: {
                            NO: {
                                type: ArgumentType.NUMBER,
                                menu: 'serialNo',
                                defaultValue: SerialNo.Serial1
                            }
                        },
                        blockType: BlockType.REPORTER,
                        programMode: [ProgramModeType.UPLOAD]
                    }
                ],
                menus: {
                    baudrate: {
                        items: this.BAUDTATE_MENU
                    },
                    serialNo: {
                        items: this.SERIAL_NO_MENU
                    },
                    eol: {
                        items: this.EOL_MENU
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
    esp32InitPinMode (args) {
        this._peripheral.esp32InitPinMode(args.PIN, args.MODE);
        return Promise.resolve();
    }

    /**
     * Set pin digital out level.
     * @param {object} args - the block's arguments.
     * @return {Promise} - a Promise that resolves after the set pin digital out level is done.
     */
    esp32SetDigitalOutput (args) {
        this._peripheral.esp32SetDigitalOutput(args.PIN, args.LEVEL);
        return Promise.resolve();
    }

    /**
     * Set pin pwm out value.
     * @param {object} args - the block's arguments.
     * @return {Promise} - a Promise that resolves after the set pin pwm out value is done.
     */
    esp32SetPwmOutput (args) {
        this._peripheral.esp32SetPwmOutput(args.PIN, args.OUT);
        return Promise.resolve();
    }

    /**
     * Set pin pwm out frequency.
     * @param {object} args - the block's arguments.
     * @return {Promise} - a Promise that resolves after the set pin pwm out frequency is done.
     */
    esp32SetPwmFrequency (args) {
        this._peripheral.esp32SetPwmFrequency(args.PIN, args.OUT);
        return Promise.resolve();
    }

    /**
     * Set dac pin out value.
     * @param {object} args - the block's arguments.
     * @return {Promise} - a Promise that resolves after the set dac pin out value is done.
     */
    esp32SetDACOutput (args) {
        this._peripheral.esp32SetDACOutput(args.PIN, args.OUT);
        return Promise.resolve();
    }

    /**
     * Set analog pin reference voltage.
     * @param {object} args - the block's arguments.
     * @return {Promise} - a Promise that resolves after the set analog pin reference voltage is done.
     */
    esp32SetAnalogVoltage (args) {
        this._peripheral.esp32SetAnalogVoltage(args.PIN, args.OUT);
        return Promise.resolve();
    }

    /**
     * Read pin digital level.
     * @param {object} args - the block's arguments.
     * @return {boolean} - true if read high level, false if read low level.
     */
    esp32ReadDigitalPin (args) {
        return this._peripheral.esp32ReadDigitalPin(args.PIN);
    }

    /**
     * Read analog pin.
     * @param {object} args - the block's arguments.
     * @return {number} - analog value fo the pin.
     */
    esp32ReadAnalogPin (args) {
        return this._peripheral.esp32ReadAnalogPin(args.PIN);
    }

    /**
     * Read touch pin.
     * @param {object} args - the block's arguments.
     * @return {number} - read value of touch pin.
     */
    esp32ReadTouchPin (args) {
        return this._peripheral.esp32ReadTouchPin(args.PIN);
    }

    /**
     * Set servo out put.
     * @param {object} args - the block's arguments.
     * @return {Promise} - a Promise that resolves after the set servo out value is done.
     */
    esp32SetServoOutput (args) {
        this._peripheral.esp32SetServoOutput(args.PIN, args.OUT);
        return Promise.resolve();
    }
}

module.exports = OpenBlockMicroPythonEsp32Device;

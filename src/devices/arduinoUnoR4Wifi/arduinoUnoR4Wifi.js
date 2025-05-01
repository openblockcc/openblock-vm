const formatMessage = require('format-message');

const ArgumentType = require('../../extension-support/argument-type');
const BlockType = require('../../extension-support/block-type');
const ProgramModeType = require('../../extension-support/program-mode-type');

const ArduinoPeripheral = require('../common/arduino-peripheral');

/**
 * The list of USB device filters.
 * @readonly
 */
const PNPID_LIST = [
    // https://github.com/arduino/ArduinoCore-renesas/blob/1.4.0/boards.txt#L123-L126
    'USB\\VID_2341&PID_1002',
    'USB\\VID_2341&PID_006D'
];

/**
 * Configuration of serialport
 * @readonly
 */
const SERIAL_CONFIG = {
    baudRate: 9600,
    dataBits: 8,
    stopBits: 1
};

/**
 * Configuration for arduino-cli.
 * @readonly
 */
const DIVECE_OPT = {
    type: 'arduino',
    fqbn: 'arduino:renesas_uno:unor4wifi'
};

const Pins = {
    D0: '0',
    D1: '1',
    D2: '2',
    D3: '3',
    D4: '4',
    D5: '5',
    D6: '6',
    D7: '7',
    D8: '8',
    D9: '9',
    D10: '10',
    D11: '11',
    D12: '12',
    D13: '13',
    A0: 'A0',
    A1: 'A1',
    A2: 'A2',
    A3: 'A3',
    A4: 'A4',
    A5: 'A5'
};

const Level = {
    High: 'HIGH',
    Low: 'LOW'
};

const LedState = {
    On: '1',
    Off: '0'
};

const Buadrate = {
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
    Integer: 'INTEGER',
    Decimal: 'DECIMAL',
    String: 'STRING'
};

/**
 * Manage communication with a Arduino Uno R4 WiFi peripheral over a OpenBlock Link client socket.
 */
class ArduinoUnoR4Wifi extends ArduinoPeripheral{
    /**
     * Construct a Arduino communication object.
     * @param {Runtime} runtime - the OpenBlock runtime
     * @param {string} deviceId - the id of the extension
     * @param {string} originalDeviceId - the original id of the peripheral, like xxx_arduinoUno
     */
    constructor (runtime, deviceId, originalDeviceId) {
        super(runtime, deviceId, originalDeviceId, PNPID_LIST, SERIAL_CONFIG, DIVECE_OPT);
    }
}

/**
 * OpenBlock blocks to interact with a Arduino Uno R4 WiFi peripheral.
 */
class OpenBlockArduinoUnoR4WifiDevice {
    /**
     * @return {string} - the ID of this extension.
     */
    get DEVICE_ID () {
        return 'arduinoUnoR4Wifi';
    }

    get PINS_MENU () {
        return [
            {
                text: '0',
                value: Pins.D0
            },
            {
                text: '1',
                value: Pins.D1
            },
            {
                text: '2',
                value: Pins.D2
            },
            {
                text: '3',
                value: Pins.D3
            },
            {
                text: '4',
                value: Pins.D4
            },
            {
                text: '5',
                value: Pins.D5
            },
            {
                text: '6',
                value: Pins.D6
            },
            {
                text: '7',
                value: Pins.D7
            },
            {
                text: '8',
                value: Pins.D8
            },
            {
                text: '9',
                value: Pins.D9
            },
            {
                text: '10',
                value: Pins.D10
            },
            {
                text: '11',
                value: Pins.D11
            },
            {
                text: '12',
                value: Pins.D12
            },
            {
                text: '13',
                value: Pins.D13
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
                    id: 'arduinoUnoR4Wifi.modeMenu.input',
                    default: 'input',
                    description: 'label for input pin mode'
                }),
                value: Mode.Input
            },
            {
                text: formatMessage({
                    id: 'arduinoUnoR4Wifi.modeMenu.output',
                    default: 'output',
                    description: 'label for output pin mode'
                }),
                value: Mode.Output
            },
            {
                text: formatMessage({
                    id: 'arduinoUnoR4Wifi.modeMenu.inputPullup',
                    default: 'input-pullup',
                    description: 'label for input-pullup pin mode'
                }),
                value: Mode.InputPullup
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
                    id: 'arduinoUnoR4Wifi.levelMenu.high',
                    default: 'high',
                    description: 'label for high level'
                }),
                value: Level.High
            },
            {
                text: formatMessage({
                    id: 'arduinoUnoR4Wifi.levelMenu.low',
                    default: 'low',
                    description: 'label for low level'
                }),
                value: Level.Low
            }
        ];
    }

    get PWM_PINS_MENU () {
        return [
            {
                text: '3',
                value: Pins.D3
            },
            {
                text: '5',
                value: Pins.D5
            },
            {
                text: '6',
                value: Pins.D6
            },
            {
                text: '9',
                value: Pins.D9
            },
            {
                text: '10',
                value: Pins.D10
            },
            {
                text: '11',
                value: Pins.D11
            }
        ];
    }

    get INTERRUPT_PINS_MENU () {
        return [
            {
                text: '2',
                value: Pins.D2
            },
            {
                text: '3',
                value: Pins.D3
            }
        ];
    }

    get INTERRUP_MODE_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'arduinoUnoR4Wifi.InterrupModeMenu.risingEdge',
                    default: 'rising edge',
                    description: 'label for rising edge interrup'
                }),
                value: InterrupMode.Rising
            },
            {
                text: formatMessage({
                    id: 'arduinoUnoR4Wifi.InterrupModeMenu.fallingEdge',
                    default: 'falling edge',
                    description: 'label for falling edge interrup'
                }),
                value: InterrupMode.Falling
            },
            {
                text: formatMessage({
                    id: 'arduinoUnoR4Wifi.InterrupModeMenu.changeEdge',
                    default: 'change edge',
                    description: 'label for change edge interrup'
                }),
                value: InterrupMode.Change
            },
            {
                text: formatMessage({
                    id: 'arduinoUnoR4Wifi.InterrupModeMenu.low',
                    default: 'low',
                    description: 'label for low interrup'
                }),
                value: InterrupMode.Low
            }
        ];
    }

    get LEDSTATE_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'arduinoUnoR4Wifi.ledState.on',
                    default: 'on',
                    description: 'label for led state on'
                }),
                value: LedState.On
            },
            {
                text: formatMessage({
                    id: 'arduinoUnoR4Wifi.ledState.off',
                    default: 'off',
                    description: 'label for led state off'
                }),
                value: LedState.Off
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
                    id: 'arduinoUnoR4Wifi.eolMenu.warp',
                    default: 'warp',
                    description: 'label for warp print'
                }),
                value: Eol.Warp
            },
            {
                text: formatMessage({
                    id: 'arduinoUnoR4Wifi.eolMenu.noWarp',
                    default: 'no-warp',
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
                    id: 'arduinoUnoR4Wifi.dataTypeMenu.integer',
                    default: 'integer',
                    description: 'label for integer'
                }),
                value: DataType.Integer
            },
            {
                text: formatMessage({
                    id: 'arduinoUnoR4Wifi.dataTypeMenu.decimal',
                    default: 'decimal',
                    description: 'label for decimal number'
                }),
                value: DataType.Decimal
            },
            {
                text: formatMessage({
                    id: 'arduinoUnoR4Wifi.dataTypeMenu.string',
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
     * @param {string} originalDeviceId - the original id of the peripheral, like xxx_arduinoUno
     */
    constructor (runtime, originalDeviceId) {
        /**
         * The OpenBlock runtime.
         * @type {Runtime}
         */
        this.runtime = runtime;

        // Create a new Arduino uno R4 WiFi peripheral instance
        this._peripheral = new ArduinoUnoR4Wifi(this.runtime, this.DEVICE_ID, originalDeviceId);

        this._peripheral.numDigitalPins = 14;
    }

    /**
     * @returns {Array.<object>} metadata for this extension and its blocks.
     */
    getInfo () {
        return [
            {
                id: 'pin',
                name: formatMessage({
                    id: 'arduinoUnoR4Wifi.category.pins',
                    default: 'Pins',
                    description: 'The name of the arduino uno device pin category'
                }),
                color1: '#4C97FF',
                color2: '#3373CC',
                color3: '#3373CC',

                blocks: [
                    {
                        opcode: 'setPinMode',
                        text: formatMessage({
                            id: 'arduinoUnoR4Wifi.pins.setPinMode',
                            default: 'set pin [PIN] mode [MODE]',
                            description: 'arduinoUnoR4Wifi set pin mode'
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
                            id: 'arduinoUnoR4Wifi.pins.setDigitalOutput',
                            default: 'set digital pin [PIN] out [LEVEL]',
                            description: 'arduinoUnoR4Wifi set digital pin out'
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
                            id: 'arduinoUnoR4Wifi.pins.setPwmOutput',
                            default: 'set pwm pin [PIN] out [OUT]',
                            description: 'arduinoUnoR4Wifi set pwm pin out'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'pwmPins',
                                defaultValue: Pins.D3
                            },
                            OUT: {
                                type: ArgumentType.UINT8_NUMBER,
                                defaultValue: '255'
                            }
                        }
                    },
                    '---',
                    {
                        opcode: 'readDigitalPin',
                        text: formatMessage({
                            id: 'arduinoUnoR4Wifi.pins.readDigitalPin',
                            default: 'read digital pin [PIN]',
                            description: 'arduinoUnoR4Wifi read digital pin'
                        }),
                        blockType: BlockType.BOOLEAN,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'pins',
                                defaultValue: Pins.D0
                            }
                        }
                    },
                    {
                        opcode: 'readAnalogPin',
                        text: formatMessage({
                            id: 'arduinoUnoR4Wifi.pins.readAnalogPin',
                            default: 'read analog pin [PIN]',
                            description: 'arduinoUnoR4Wifi read analog pin'
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
                            id: 'arduinoUnoR4Wifi.pins.setServoOutput',
                            default: 'set servo pin [PIN] out [OUT]',
                            description: 'arduinoUnoR4Wifi set servo pin out'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'pwmPins',
                                defaultValue: Pins.D3
                            },
                            OUT: {
                                type: ArgumentType.HALF_ANGLE,
                                defaultValue: '90'
                            }
                        }
                    },
                    '---',
                    {

                        opcode: 'attachInterrupt',
                        text: formatMessage({
                            id: 'arduinoUnoR4Wifi.pins.attachInterrupt',
                            default: 'attach interrupt pin [PIN] mode [MODE] executes',
                            description: 'arduinoUnoR4Wifi attach interrupt'
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
                            id: 'arduinoUnoR4Wifi.pins.detachInterrupt',
                            default: 'detach interrupt pin [PIN]',
                            description: 'arduinoUnoR4Wifi detach interrupt'
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
                    interruptPins: {
                        items: this.INTERRUPT_PINS_MENU
                    },
                    interruptMode: {
                        items: this.INTERRUP_MODE_MENU
                    }
                }
            },
            {
                id: 'display',
                name: formatMessage({
                    id: 'arduinoUnoR4Wifi.category.display',
                    default: 'Display',
                    description: 'The name of the Arduino Uno R4 Wifi device display category'
                }),
                color1: '#4CBFE6',
                color2: '#2E8EB8',
                color3: '#2E8EB8',
                blocks: [
                    {
                        opcode: 'showImage',
                        text: formatMessage({
                            id: 'arduinoUnoR4Wifi.display.showImage',
                            default: 'show image [VALUE]',
                            description: 'Arduino Uno R4 Wifi show image'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            VALUE: {
                                type: ArgumentType.MATRIX8X12,
                                defaultValue: '001100011000010010100100010001000100001000001000000100010000000010100000000001000000000000000000' // eslint-disable-line max-len
                            }
                        }
                    },
                    {
                        opcode: 'showImageUntil',
                        text: formatMessage({
                            id: 'arduinoUnoR4Wifi.display.showImageUntil',
                            default: 'show image [VALUE] for [TIME] secs',
                            description: 'Arduino Uno R4 Wifi show image for some times'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            VALUE: {
                                type: ArgumentType.MATRIX8X12,
                                defaultValue: '001100011000010010100100010001000100001000001000000100010000000010100000000001000000000000000000' // eslint-disable-line max-len
                            },
                            TIME: {
                                type: ArgumentType.NUMBER,
                                defaultValue: '1'
                            }
                        }
                    },
                    {
                        opcode: 'showUntilScrollDone',
                        text: formatMessage({
                            id: 'arduinoUnoR4Wifi.display.showUntilScrollDone',
                            default: 'show [TEXT] until scroll done',
                            description: 'Arduino Uno R4 Wifi show until scroll done'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            TEXT: {
                                type: ArgumentType.STRING,
                                defaultValue: 'Hello OpenBlock'
                            }
                        }
                    },
                    {
                        opcode: 'clearDisplay',
                        text: formatMessage({
                            id: 'arduinoUnoR4Wifi.display.clearDisplay',
                            default: 'clear screen',
                            description: 'Arduino Uno R4 Wifi clear display'
                        }),
                        blockType: BlockType.COMMAND
                    },
                    {
                        opcode: 'lightPixelAt',
                        text: formatMessage({
                            id: 'arduinoUnoR4Wifi.display.lightPixelAt',
                            default: 'light [STATE] at the x: [X] axis, y: [Y] axis',
                            description: 'Arduino Uno R4 Wifi light pixel at'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            STATE: {
                                type: ArgumentType.STRING,
                                menu: 'ledState',
                                defaultValue: LedState.On
                            },
                            X: {
                                type: ArgumentType.NUMBER,
                                defaultValue: '0'
                            },
                            Y: {
                                type: ArgumentType.NUMBER,
                                defaultValue: '0'
                            }
                        }
                    }
                ],
                menus: {
                    ledState: {
                        items: this.LEDSTATE_MENU
                    }
                }
            },
            {
                id: 'serial',
                name: formatMessage({
                    id: 'arduinoUnoR4Wifi.category.serial',
                    default: 'Serial',
                    description: 'The name of the arduino uno device serial category'
                }),
                color1: '#9966FF',
                color2: '#774DCB',
                color3: '#774DCB',

                blocks: [
                    {
                        opcode: 'serialBegin',
                        text: formatMessage({
                            id: 'arduinoUnoR4Wifi.serial.serialBegin',
                            default: 'serial begin baudrate [VALUE]',
                            description: 'arduinoUnoR4Wifi serial begin'
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
                            id: 'arduinoUnoR4Wifi.serial.serialPrint',
                            default: 'serial print [VALUE] [EOL]',
                            description: 'arduinoUnoR4Wifi serial print'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
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
                        opcode: 'serialAvailable',
                        text: formatMessage({
                            id: 'arduinoUnoR4Wifi.serial.serialAvailable',
                            default: 'serial available data length',
                            description: 'arduinoUnoR4Wifi serial available data length'
                        }),
                        blockType: BlockType.REPORTER,
                        disableMonitor: true,
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {
                        opcode: 'serialReadAByte',
                        text: formatMessage({
                            id: 'arduinoUnoR4Wifi.serial.serialReadAByte',
                            default: 'serial read a byte',
                            description: 'arduinoUnoR4Wifi serial read a byte'
                        }),
                        blockType: BlockType.REPORTER,
                        disableMonitor: true,
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
                    id: 'arduinoUnoR4Wifi.category.data',
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
                            id: 'arduinoUnoR4Wifi.data.dataMap',
                            default: 'map [DATA] from ([ARG0], [ARG1]) to ([ARG2], [ARG3])',
                            description: 'arduinoUnoR4Wifi data map'
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
                    {
                        opcode: 'dataConstrain',
                        text: formatMessage({
                            id: 'arduinoUnoR4Wifi.data.dataConstrain',
                            default: 'constrain [DATA] between ([ARG0], [ARG1])',
                            description: 'arduinoUnoR4Wifi data constrain'
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
                    '---',
                    {
                        opcode: 'dataConvert',
                        text: formatMessage({
                            id: 'arduinoUnoR4Wifi.data.dataConvert',
                            default: 'convert [DATA] to [TYPE]',
                            description: 'arduinoUnoR4Wifi data convert'
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
                                defaultValue: DataType.Integer
                            }
                        },
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {
                        opcode: 'dataConvertASCIICharacter',
                        text: formatMessage({
                            id: 'arduinoUnoR4Wifi.data.dataConvertASCIICharacter',
                            default: 'convert [DATA] to ASCII character',
                            description: 'arduinoUnoR4Wifi data convert to ASCII character'
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
                            id: 'arduinoUnoR4Wifi.data.dataConvertASCIINumber',
                            default: 'convert [DATA] to ASCII nubmer',
                            description: 'arduinoUnoR4Wifi data convert to ASCII nubmer'
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

    /**
     * Set servo out put.
     * @param {object} args - the block's arguments.
     * @return {Promise} - a Promise that resolves after the set servo out value is done.
     */
    setServoOutput (args) {
        this._peripheral.setServoOutput(args.PIN, args.OUT);
        return Promise.resolve();
    }
}

module.exports = OpenBlockArduinoUnoR4WifiDevice;

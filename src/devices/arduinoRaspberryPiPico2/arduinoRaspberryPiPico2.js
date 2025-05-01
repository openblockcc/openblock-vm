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
    // https://github.com/earlephilhower/arduino-pico/blob/4.4.4/boards.txt
    'USB\\VID_2E8A&PID_000F',
    'USB\\VID_2E8A&PID_010F',
    'USB\\VID_2E8A&PID_400F',
    'USB\\VID_2E8A&PID_410F',
    'USB\\VID_2E8A&PID_800F',
    'USB\\VID_2E8A&PID_810F',
    'USB\\VID_2E8A&PID_C00F',
    'USB\\VID_2E8A&PID_C10F'
];

/**
 * Configuration of serialport
 * @readonly
 */
const SERIAL_CONFIG = {
    baudRate: 115200,
    dataBits: 8,
    stopBits: 1,
    dtr: true,
    rts: false
};

/**
 * Configuration for arduino-cli.
 * @readonly
 */
const DIVECE_OPT = {
    type: 'arduino',
    // Flash Size: "4MB (no FS)"
    // CPU Speed: "150 MHz"
    // CPU Architecture: "ARM"
    // Optimize: "Small (-Os) (standard)"
    // Profiling: "Disabled"
    // RTTI: "Disabled"
    // Stack Protector: "Disabled"
    // C++ Exceptions: "Disabled"
    // Debug Port: "Disabled"
    // Debug Level: "None"
    // USB Stack: "Pico SDK"
    // IP/Bluetooth Stack: "IPv4 Only"
    // Upload Method: "Default (UF2)
    fqbn: 'rp2040:rp2040:rpipico2:flash=4194304_0,arch=arm,freq=150,opt=Small,profile=Disabled,rtti=Disabled,stackprotect=Disabled,exceptions=Disabled,dbgport=Disabled,dbglvl=None,usbstack=picosdk,ipbtstack=ipv4only,uploadmethod=default', // eslint-disable-line max-len
    // The time to wait for USB re-recognition after uploading is completed. Darwin and linux will take more time to
    // rerecognize device, therefore, this parameter should be tested and confirmed under Linux or Darwin system.
    postUploadDelay: 3000
};

const Pins = {
    GP0: '0',
    GP1: '1',
    GP2: '2',
    GP3: '3',
    GP4: '4',
    GP5: '5',
    GP6: '6',
    GP7: '7',
    GP8: '8',
    GP9: '9',
    GP10: '10',
    GP11: '11',
    GP12: '12',
    GP13: '13',
    GP14: '14',
    GP15: '15',
    GP16: '16',
    GP17: '17',
    GP18: '18',
    GP19: '19',
    GP20: '20',
    GP21: '21',
    GP22: '22',
    GP23: '23',
    GP24: '24',
    GP25: '25',
    GP26: '26',
    GP27: '27',
    GP28: '28'
};

const Level = {
    High: 'HIGH',
    Low: 'LOW'
};

const SerialNo = {
    USB: '0',
    Serial1: '1',
    Serial2: '2'
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
    InputPullup: 'INPUT_PULLUP',
    InputPulldown: 'INPUT_PULLDOWN'
};

const InterrupMode = {
    Rising: 'RISING',
    Falling: 'FALLING',
    Change: 'CHANGE',
    LowLevel: 'LOW',
    HighLevel: 'HIGH'
};

const DataType = {
    Integer: 'INTEGER',
    Decimal: 'DECIMAL',
    String: 'STRING'
};

/**
 * Manage communication with a Arduino RaspberryPiPico2 peripheral over a OpenBlock Link client socket.
 */
class ArduinoRaspberryPiPico2 extends CommonPeripheral{
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
 * OpenBlock blocks to interact with a Arduino RaspberryPiPico2 peripheral.
 */
class OpenBlockArduinoRaspberryPiPico2Device {
    /**
     * @return {string} - the ID of this extensGPn.
     */
    get DEVICE_ID () {
        return 'arduinoRaspberryPiPico2';
    }

    get PINS_MENU () {
        return [
            {
                text: 'GP0',
                value: Pins.GP0
            },
            {
                text: 'GP1',
                value: Pins.GP1
            },
            {
                text: 'GP2',
                value: Pins.GP2
            },
            {
                text: 'GP3',
                value: Pins.GP3
            },
            {
                text: 'GP4',
                value: Pins.GP4
            },
            {
                text: 'GP5',
                value: Pins.GP5
            },
            {
                text: 'GP6',
                value: Pins.GP6
            },
            {
                text: 'GP7',
                value: Pins.GP7
            },
            {
                text: 'GP8',
                value: Pins.GP8
            },
            {
                text: 'GP9',
                value: Pins.GP9
            },
            {
                text: 'GP10',
                value: Pins.GP10
            },
            {
                text: 'GP11',
                value: Pins.GP11
            },
            {
                text: 'GP12',
                value: Pins.GP12
            },
            {
                text: 'GP13',
                value: Pins.GP13
            },
            {
                text: 'GP14',
                value: Pins.GP14
            },
            {
                text: 'GP15',
                value: Pins.GP15
            },
            {
                text: 'GP16',
                value: Pins.GP16
            },
            {
                text: 'GP17',
                value: Pins.GP17
            },
            {
                text: 'GP18',
                value: Pins.GP18
            },
            {
                text: 'GP19',
                value: Pins.GP19
            },
            {
                text: 'GP20',
                value: Pins.GP20
            },
            {
                text: 'GP21',
                value: Pins.GP21
            },
            {
                text: 'GP22',
                value: Pins.GP22
            },
            // {
            //     text: 'GP23',
            //     value: Pins.GP23
            // },
            // {
            //     text: 'GP24',
            //     value: Pins.GP24
            // },
            {
                text: 'LED',
                value: Pins.GP25
            },
            {
                text: 'GP26',
                value: Pins.GP26
            },
            {
                text: 'GP27',
                value: Pins.GP27
            },
            {
                text: 'GP28',
                value: Pins.GP28
            }
        ];
    }

    get DEFAULT_PIN () {
        return Pins.GP0;
    }

    get MODE_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'arduinoRaspberryPiPico2.modeMenu.input',
                    default: 'input',
                    description: 'label for input pin mode'
                }),
                value: Mode.Input
            },
            {
                text: formatMessage({
                    id: 'arduinoRaspberryPiPico2.modeMenu.output',
                    default: 'output',
                    description: 'label for output pin mode'
                }),
                value: Mode.Output
            },
            {
                text: formatMessage({
                    id: 'arduinoRaspberryPiPico2.modeMenu.inputPullup',
                    default: 'input-pullup',
                    description: 'label for input-pullup pin mode'
                }),
                value: Mode.InputPullup
            },
            {
                text: formatMessage({
                    id: 'arduinoRaspberryPiPico2.modeMenu.inputPulldown',
                    default: 'input-pulldown',
                    description: 'label for input-pulldown pin mode'
                }),
                value: Mode.InputPulldown
            }
        ];
    }

    get LEVEL_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'arduinoRaspberryPiPico2.levelMenu.high',
                    default: 'high',
                    description: 'label for high level'
                }),
                value: Level.High
            },
            {
                text: formatMessage({
                    id: 'arduinoRaspberryPiPico2.levelMenu.low',
                    default: 'low',
                    description: 'label for low level'
                }),
                value: Level.Low
            }
        ];
    }

    get ANALOG_PINS_MENU () {
        return [
            {
                text: 'GP26',
                value: Pins.GP26
            },
            {
                text: 'GP27',
                value: Pins.GP27
            },
            {
                text: 'GP28',
                value: Pins.GP28
            }
        ];
    }

    get INTERRUP_MODE_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'arduinoRaspberryPiPico2.InterrupModeMenu.risingEdge',
                    default: 'rising edge',
                    description: 'label for rising edge interrup'
                }),
                value: InterrupMode.Rising
            },
            {
                text: formatMessage({
                    id: 'arduinoRaspberryPiPico2.InterrupModeMenu.fallingEdge',
                    default: 'falling edge',
                    description: 'label for falling edge interrup'
                }),
                value: InterrupMode.Falling
            },
            {
                text: formatMessage({
                    id: 'arduinoRaspberryPiPico2.InterrupModeMenu.changeEdge',
                    default: 'change edge',
                    description: 'label for change edge interrup'
                }),
                value: InterrupMode.Change
            },
            {
                text: formatMessage({
                    id: 'arduinoRaspberryPiPico2.InterrupModeMenu.low',
                    default: 'low level',
                    description: 'label for low level interrup'
                }),
                value: InterrupMode.LowLevel
            },
            {
                text: formatMessage({
                    id: 'arduinoRaspberryPiPico2.InterrupModeMenu.high',
                    default: 'high level',
                    description: 'label for high level interrup'
                }),
                value: InterrupMode.HighLevel
            }
        ];
    }

    get SERIAL_NO_MENU () {
        return [
            {
                text: 'USB',
                value: SerialNo.USB
            },
            {
                text: '1',
                value: SerialNo.Serial1
            },
            {
                text: '2',
                value: SerialNo.Serial2
            }
        ];
    }

    get DEFAULT_SERIAL_RX_PIN () {
        return Pins.GP1;
    }

    get DEFAULT_SERIAL_TX_PIN () {
        return Pins.GP0;
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
                    id: 'arduinoRaspberryPiPico2.eolMenu.warp',
                    default: 'warp',
                    description: 'label for warp print'
                }),
                value: Eol.Warp
            },
            {
                text: formatMessage({
                    id: 'arduinoRaspberryPiPico2.eolMenu.noWarp',
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
                    id: 'arduinoRaspberryPiPico2.dataTypeMenu.integer',
                    default: 'integer',
                    description: 'label for integer'
                }),
                value: DataType.Integer
            },
            {
                text: formatMessage({
                    id: 'arduinoRaspberryPiPico2.dataTypeMenu.decimal',
                    default: 'decimal',
                    description: 'label for decimal number'
                }),
                value: DataType.Decimal
            },
            {
                text: formatMessage({
                    id: 'arduinoRaspberryPiPico2.dataTypeMenu.string',
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

        // Create a new Arduino RaspberryPiPico peripheral instance
        this._peripheral = new ArduinoRaspberryPiPico2(this.runtime, this.DEVICE_ID, originalDeviceId);
    }

    /**
     * @returns {Array.<object>} metadata for this extension and its blocks.
     */
    getInfo () {
        return [
            {
                id: 'pin',
                name: formatMessage({
                    id: 'arduinoRaspberryPiPico2.category.pins',
                    default: 'Pins',
                    description: 'The name of the arduino RaspberryPiPico2 device pin category'
                }),
                color1: '#4C97FF',
                color2: '#3373CC',
                color3: '#3373CC',

                blocks: [
                    {
                        opcode: 'setPinMode',
                        text: formatMessage({
                            id: 'arduinoRaspberryPiPico2.pins.setPinMode',
                            default: 'set pin [PIN] mode [MODE]',
                            description: 'arduinoRaspberryPiPico2 set pin mode'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'pins',
                                defaultValue: this.DEFAULT_PIN
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
                            id: 'arduinoRaspberryPiPico2.pins.setDigitalOutput',
                            default: 'set digital pin [PIN] out [LEVEL]',
                            description: 'arduinoRaspberryPiPico2 set digital pin out'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'pins',
                                defaultValue: this.DEFAULT_PIN
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
                            id: 'arduinoRaspberryPiPico2.pins.setPwmOutput',
                            default: 'set pwm pin [PIN] out [OUT]',
                            description: 'arduinoRaspberryPiPico2 set pwm pin out'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'pins',
                                defaultValue: this.DEFAULT_PIN
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
                            id: 'arduinoRaspberryPiPico2.pins.readDigitalPin',
                            default: 'read digital pin [PIN]',
                            description: 'arduinoRaspberryPiPico2 read digital pin'
                        }),
                        blockType: BlockType.BOOLEAN,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'pins',
                                defaultValue: this.DEFAULT_PIN
                            }
                        }
                    },
                    {
                        opcode: 'readAnalogPin',
                        text: formatMessage({
                            id: 'arduinoRaspberryPiPico2.pins.readAnalogPin',
                            default: 'read analog pin [PIN]',
                            description: 'arduinoRaspberryPiPico2 read analog pin'
                        }),
                        blockType: BlockType.REPORTER,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'analogPins',
                                defaultValue: Pins.GP26
                            }
                        }
                    },
                    '---',
                    {

                        opcode: 'setServoOutput',
                        text: formatMessage({
                            id: 'arduinoRaspberryPiPico2.pins.setServoOutput',
                            default: 'set servo pin [PIN] out [OUT]',
                            description: 'arduinoRaspberryPiPico2 set servo pin out'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'pins',
                                defaultValue: this.DEFAULT_PIN
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
                            id: 'arduinoRaspberryPiPico2.pins.attachInterrupt',
                            default: 'attach interrupt pin [PIN] mode [MODE] executes',
                            description: 'arduinoRaspberryPiPico2 attach interrupt'
                        }),
                        blockType: BlockType.CONDITIONAL,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'pins',
                                defaultValue: this.DEFAULT_PIN
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
                            id: 'arduinoRaspberryPiPico2.pins.detachInterrupt',
                            default: 'detach interrupt pin [PIN]',
                            description: 'arduinoRaspberryPiPico2 detach interrupt'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'pins',
                                defaultValue: this.DEFAULT_PIN
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
                    interruptMode: {
                        items: this.INTERRUP_MODE_MENU
                    }
                }
            },
            {
                id: 'serial',
                name: formatMessage({
                    id: 'arduinoRaspberryPiPico2.category.serial',
                    default: 'Serial',
                    description: 'The name of the arduino RaspberryPiPico device serial category'
                }),
                color1: '#9966FF',
                color2: '#774DCB',
                color3: '#774DCB',

                blocks: [
                    {
                        opcode: 'raspberryPiPicoMultiSerialBegin',
                        text: formatMessage({
                            id: 'arduinoRaspberryPiPico2.serial.raspberryPiPicoMultiSerialBegin',
                            default: 'serial [NO] begin baudrate [VALUE]',
                            description: 'arduinoRaspberryPiPico2 multi serial begin'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            NO: {
                                type: ArgumentType.NUMBER,
                                menu: 'serialNo',
                                defaultValue: SerialNo.USB
                            },
                            VALUE: {
                                type: ArgumentType.STRING,
                                menu: 'baudrate',
                                defaultValue: Buadrate.B9600
                            }
                        },
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {
                        opcode: 'multiSerialPrint',
                        text: formatMessage({
                            id: 'arduinoRaspberryPiPico2.serial.multiSerialPrint',
                            default: 'serial [NO] print [VALUE] [EOL]',
                            description: 'arduinoRaspberryPiPico2 multi serial print'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            NO: {
                                type: ArgumentType.NUMBER,
                                menu: 'serialNo',
                                defaultValue: SerialNo.USB
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
                        opcode: 'multiSerialAvailable',
                        text: formatMessage({
                            id: 'arduinoRaspberryPiPico2.serial.multiSerialAvailable',
                            default: 'serial [NO] available data length',
                            description: 'arduinoRaspberryPiPico2 multi serial available data length'
                        }),
                        arguments: {
                            NO: {
                                type: ArgumentType.NUMBER,
                                menu: 'serialNo',
                                defaultValue: SerialNo.USB
                            }
                        },
                        blockType: BlockType.REPORTER,
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {
                        opcode: 'multiSerialReadAByte',
                        text: formatMessage({
                            id: 'arduinoRaspberryPiPico2.serial.multiSerialReadAByte',
                            default: 'serial [NO] read a byte',
                            description: 'arduinoRaspberryPiPico2 multi serial read a byte'
                        }),
                        arguments: {
                            NO: {
                                type: ArgumentType.NUMBER,
                                menu: 'serialNo',
                                defaultValue: SerialNo.USB
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
            },
            {
                id: 'data',
                name: formatMessage({
                    id: 'arduinoRaspberryPiPico2.category.data',
                    default: 'Data',
                    description: 'The name of the arduino RaspberryPiPico device data category'
                }),
                color1: '#CF63CF',
                color2: '#C94FC9',
                color3: '#BD42BD',
                blocks: [
                    {
                        opcode: 'dataMap',
                        text: formatMessage({
                            id: 'arduinoRaspberryPiPico2.data.dataMap',
                            default: 'map [DATA] from ([ARG0], [ARG1]) to ([ARG2], [ARG3])',
                            description: 'arduinoRaspberryPiPico2 data map'
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
                            id: 'arduinoRaspberryPiPico2.data.dataConstrain',
                            default: 'constrain [DATA] between ([ARG0], [ARG1])',
                            description: 'arduinoRaspberryPiPico2 data constrain'
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
                            id: 'arduinoRaspberryPiPico2.data.dataConvert',
                            default: 'convert [DATA] to [TYPE]',
                            description: 'arduinoRaspberryPiPico2 data convert'
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
                            id: 'arduinoRaspberryPiPico2.data.dataConvertASCIICharacter',
                            default: 'convert [DATA] to ASCII character',
                            description: 'arduinoRaspberryPiPico2 data convert to ASCII character'
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
                            id: 'arduinoRaspberryPiPico2.data.dataConvertASCIINumber',
                            default: 'convert [DATA] to ASCII nubmer',
                            description: 'arduinoRaspberryPiPico2 data convert to ASCII nubmer'
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

module.exports = OpenBlockArduinoRaspberryPiPico2Device;

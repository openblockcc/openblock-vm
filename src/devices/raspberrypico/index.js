const formatMessage = require('format-message');

const ArgumentType = require('../../extension-support/argument-type');
const BlockType = require('../../extension-support/block-type');
const RaspberryPicoPeripheral = require('../arduinoCommon/raspberrypico-peripheral');

/**
* The list of USB device filters.
* @readonly
*/
const PNPID_LIST = [
    // RaspberryPico
    'USB\\VID_2E8A&PID_0005'
];

/**
* Configuration of serialport
* @readonly
*/
const SERIAL_CONFIG = {
    baudRate: 115200,
    dataBits: 8,
    stopBits: 1,
    hupcl:true
};

/**
 * Configuration of flash.
 * @readonly
 */

const menuIconURI = "data:image/svg+xml;base64,PD94bWwgdmVyc2lvbj0iMS4wIiBlbmNvZGluZz0iVVRGLTgiIHN0YW5kYWxvbmU9Im5vIj8+CjwhRE9DVFlQRSBzdmcgUFVCTElDICItLy9XM0MvL0RURCBTVkcgMS4xLy9FTiIgImh0dHA6Ly93d3cudzMub3JnL0dyYXBoaWNzL1NWRy8xLjEvRFREL3N2ZzExLmR0ZCI+CjxzdmcgdmVyc2lvbj0iMS4xIiBpZD0iTGF5ZXJfMSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiB4bWxuczp4bGluaz0iaHR0cDovL3d3dy53My5vcmcvMTk5OS94bGluayIgeD0iMHB4IiB5PSIwcHgiIHdpZHRoPSIyMHB4IiBoZWlnaHQ9IjI1cHgiIHZpZXdCb3g9IjAgMCAyMCAyNSIgZW5hYmxlLWJhY2tncm91bmQ9Im5ldyAwIDAgMjAgMjUiIHhtbDpzcGFjZT0icHJlc2VydmUiPiAgPGltYWdlIGlkPSJpbWFnZTAiIHdpZHRoPSIyMCIgaGVpZ2h0PSIyNSIgeD0iMCIgeT0iMCIKICAgIGhyZWY9ImRhdGE6aW1hZ2UvcG5nO2Jhc2U2NCxpVkJPUncwS0dnb0FBQUFOU1VoRVVnQUFBQlFBQUFBWkNBSUFBQUMrZFptRUFBQUFCR2RCVFVFQUFMR1BDL3hoQlFBQUFDQmpTRkpOCkFBQjZKZ0FBZ0lRQUFQb0FBQUNBNkFBQWRUQUFBT3BnQUFBNm1BQUFGM0NjdWxFOEFBQUFCbUpMUjBRQS93RC9BUCtndmFlVEFBQUEKQjNSSlRVVUg1UXdkQmhRbGpQazljZ0FBQlV4SlJFRlVPTXR0azJsc0ZHVWN4di92Kzg1TVovYVl2ZHR1dTZYVVF0dEZLRWVKd2NKeQpsRWFnUlV5OEFrcmtFQTJCaVBoSkloRkpGQWlZbUdqQUNMRkZMYWp4U0FnR09Sb09pOUNDUWJOeVZicHR5VUxiM2UzdWRuZDJ1ek03Ck96T3ZIMEQwZzcrUHovOTU4aVQvNUVHYXBnV0R3WjlPSE8rK2RpazhPbUNBWmhOYzlaTm1ybDI5dm5GdUl3QmM3THA0K0VqN3pmNWcKT3A5a01GdnBxWDZ5WWU3VExTdW0xVTlEZ3dPRGF6YTlEUDZVcDhacWNmT0VKVXBHVGQ3TlptL2c3ZXQyYVFWdFYvdDIyd3prcXJMeQpJcWRyeG5oY2lZY3lwTTk5K0pNT0pwZVR4M1ZwK3NJU2s3MUlUcW1Ba0hPQzFWVWxwdXV5SDMrNXI1RFhKcjBvT2lwRWFsQWxyUUxnCkVyL2Q1alBkdmpNbXl3b3VMeS96Mm4xU0pBY0lNVHpKUnVYdzFkSHNxR0l2c3d5ckF5UHFvS1BDT2g1WHdsZmowb2pNOGdRaGxJbksKWGx1RnQ4eUxiWGJiVTRHV2NFK0NHcFRsaVh1eWFQT1pCeTlGKzg2TmpONlJFcUZNNkh4a29Dc3FlZ1ZQamNnS2hGSTY5TnZZa2tDcgoxV3JCQUxCeTVjb0tkZXBmWjRjb0JZU1JZNEs1b3NGOXBhMXY1UHBZdERmZGZmQk8rVXluczhxQ01LSVVRbDBqWlRsL0lEQmZsbVZFCktRV0FVS2gveCs1MytvemZLK2E0cVdiMG5ob09uUjRXRFJZQTBxaFEzVnpxYi9FUkh0Ky9tcXdqczF1YVZpU1RTYmZIelFBQUFGUlcKVHFpcmZEejQ0Vm5yR2FRVzlGeHliTE9sOW9raUR3SzRsazk4MzNtM2NKVndSYXlVbFNhLzdiZlpiSWxFUXBLa2g4MzdEeHc0dm4zZgpKdTR4SitIYk1uMEVvWTFpN1lNVFFlaG9kakNteXhzc05Ta2ozNmFGRzdhc1hyWjBtY2xrWWdBZ2tVejg4RVhIUzhockpkeW5tZDRmCmMrRnF4bEl0VzJ0Wkd3WVUwWE4vcXNrcitiZ0c5RFZyelF0YXlYZWR2MnpkdXRWaGR6QUFFSW5HMHZjalNYQWVUUVh6Vk45bW04b2oKY2xtSkhaQjZNYUFBWC9LOGVlSno1c3B6U21Sditub3JYNTY5bDR6RjR3NjdBK2ZrM05jZEhZbGtjbmY2QmdYWVlaK3hYUEExQzk0dApvcitZQ0RiTXJiRlV6K0tjc3pqWEZ0SHZ3Znl1OVBWWVBQWk54eEZGVVlnbzJxNTgxTDVScUw1VlNLKzFUcXBtckRwUUEwQkF6RjA5CksyRFNMSGdOQUFyQUFDNGhRa2pMdkdXcHU5RFRQV3JHSkRFUWZrVXRydUxFWUNHNWlDKzFZUGJCL3lsQXB6d1NNK1FtM3NzZzlFanMKSzBpTGhiSmlnLzJxdHdkbmg2SWxyR21jYWtOYUxxb3JERUlZRUFQb3BwcnF5WThHMWJFTFNnUUJJb0FZaEllMFhMK1d5VkhOemZEcAoreEZHS0hYSHMwb2VqSDR0MDVicHd3QXVVdFJYa0s3azQ2MG1Id0gwcXhLTDZjb016cEV5Q2gzai9YY0swajB0Unlqd3BTNjBhL2Z1Cmkzc1BFbFhyTFVoUlhUWUFUSmpVczg0M3hMcFNJZ0JBMGxEM1M3ZlBLeEVEYUQzbm1NTGFzMFlod1JxdDc3NkpOMng0TlRyWnpWTjgKd0RWbmo3TmhJbU1aMDlWV2s4L0xDRHBRSGFnVGM4K2FLMW1FV3dYZkIvYVptOFE2eG9CeGYvbjZkZXR4c2FlNGFlR2lFc3piQ1RlYgpjMjJ6VDYxbXJSekNsTUlqR0VCemlqeXZXMnRFekZGS1hhU291YW5KNVhSaUFBZ3NtSCtYVmZPR2JnQ2R6SWp6K0pKT2VWaW1PZ09JCkFhU0MwYVZFRzRzOGRzd1pRQXZVaUhCR1lQNThBTUFBc0NBUU1NK2JmaW9UTmlobEVLNGs1cFB5MFB1cDRBbjUvaWw1ZUUvcStyRmMKMk1lWUNVSkE0WFQybnJCZ3hyekdSZ0FnTzNmdTVIbCtXc09zWTZFL2V2cHZqZVN6NTlWbzNNamJNR2NBU0xRZ1VUVnRGQlNxWnpYMQp0QjVMTjAzZHNXOVBtYmNNQUlEK1F5cWQvdXpRb1pyYVdnRFliS250OGk2OTdHM3A5clowZTF2ZXM5VVRnQ2wrLytmdGJaSWtQWW84Cm5PUWpCZ1lHamh3OWVybnQyNlZwb2E3SWdRQjY4NmxPUjc1eHc4cFZxMVpWVlZYOTEveHY4Mzg1Y2ZMbjVZdWE1em9yQXE0Snp5eGUKY3VyTTZmKzEvUTBzb3J3WkVQelo0Z0FBQUNWMFJWaDBaR0YwWlRwamNtVmhkR1VBTWpBeU1TMHhNaTB5T1ZRd05qb3lNRG96TnlzdwpNRG93TUxDQi91MEFBQUFsZEVWWWRHUmhkR1U2Ylc5a2FXWjVBREl3TWpFdE1USXRNamxVTURZNk1qQTZNemNyTURBNk1EREIzRVpSCkFBQUFBRWxGVGtTdVFtQ0MiIC8+Cjwvc3ZnPgo=";

const DIVECE_OPT = {
    type: 'raspberrypico'
};

const LedState = {
    On: 'on',
    Off: 'off'
};

const Key = {
    A: 'a',
    B: 'b'
};

const Mode = {
  input: 'Pin.IN',
  input_pullup: 'Pin.IN, Pin.PULL_UP',
  output: 'Pin.OUT'
};

const Camera = {
	monocular: 'reset',
	binacular: 'binacular_reset'
};

const colorMode = {
	CONTRAST: 'contrast',
    BRIGHTNESS: 'brightness',
	GAINVALUE: 'auto_gain',
    SATURATION: 'saturation'
};

const CAMDetails  = {
	WIDTH: 'width',
	HEIGHT: 'height',
	GAINVALUE: 'get_gain_db',
	FRAMECACHE: 'get_fb',
	ID: 'id'
};


const time = {
	SEC: 'time',
	MSEC: 'ticks_ms',
	USEC: 'ticks_us'
};

const Pins = {
    P25: '25',
    P0: '0',
    P1: '1',
    P2: '2',
    P3: '3',
    P4: '4',
    P5: '5',
    P6: '6',
    P7: '7',
    P8: '8',
    P9: '9',
    P10: '10',
    P11: '11',
    P12: '12',
    P13: '13',
    P14: '14',
    P15: '15',
    P16: '16',
    P17: '17',
    P18: '18',
    P19: '19',
    P20: '20',
    P21: '21',
    P22: '22',
    P26: '26',
    P27: '27',
    P28: '28'
};

const analogPins = {
    A0: '26',
    A1: '27',
    A2: '28'
};

const Level = {
    High: '1',
    Low: '0'
};


/**
 * Manage communication with a RaspberryPico  peripheral over a OpenBlock Link client socket.
 */
class RaspberryPico extends RaspberryPicoPeripheral{
    /**
     * Construct a RaspberryPico communication object.
     * @param {Runtime} runtime - the OpenBlock runtime
     * @param {string} deviceId - the id of the deivce
     * @param {string} originalDeviceId - the original id of the peripheral, like xxx_arduinoUno
     */
    constructor (runtime, deviceId, originalDeviceId) {
        super(runtime, deviceId, originalDeviceId, PNPID_LIST, SERIAL_CONFIG, DIVECE_OPT);
    }
}

/**
 * OpenBlock blocks to interact with a RaspberryPico  peripheral.
 */
class OpenBlockRaspberryPicoDevice {
    /**
     * @return {string} - the ID of this device.
     */
    static get DEVICE_ID () {
        return 'raspberrypico';
    }

    get LEDSTATE_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'raspberrypico.ledState.on',
                    default: 'on',
                    description: 'label for led state on'
                }),
                value: LedState.On
            },
            {
                text: formatMessage({
                    id: 'raspberrypico.ledState.off',
                    default: 'off',
                    description: 'label for led state off'
                }),
                value: LedState.Off
            }
        ];
    }

    get PINS_MENU () {
        return [
            {
                text: 'Inbuilt LED',
                value: Pins.P25
            },
            {
                text: '0',
                value: Pins.P0
            },
            {
                text: '1',
                value: Pins.P1
            },
            {
                text: '2',
                value: Pins.P2
            },
            {
                text: '3',
                value: Pins.P3
            },
            {
                text: '4',
                value: Pins.P4
            },
            {
                text: '5',
                value: Pins.P5
            },
            {
                text: '6',
                value: Pins.P6
            },
            {
                text: '7',
                value: Pins.P7
            },
            {
                text: '8',
                value: Pins.P8
            },
            {
                text: '9',
                value: Pins.P9
            },
            {
                text: '10',
                value: Pins.P10
            },
            {
                text: '11',
                value: Pins.P11
            },
            {
                text: '12',
                value: Pins.P12
            },
            {
                text: '13',
                value: Pins.P13
            },
            {
                text: '14',
                value: Pins.P14
            },
            {
                text: '15',
                value: Pins.P15
            },
            {
                text: '16',
                value: Pins.P16
            },
            {
                text: '17',
                value: Pins.P17
            },
            {
                text: '18',
                value: Pins.P18
            },
            {
                text: '19',
                value: Pins.P19
            },
            {
                text: '20',
                value: Pins.P20
            },
            {
                text: '21',
                value: Pins.P21
            },
            {
                text: '22',
                value: Pins.P22
            },
            {
                text: '26_A0',
                value: Pins.P26
            },
            {
                text: '27_A1',
                value: Pins.P27
            },
            {
                text: '28_A2',
                value: Pins.P28
            }
        ];
    }

    get MODE_MENU () {
      return [
        {
          text: 'input',
          value: Mode.input
        },
        {
          text: 'input-pullup',
          value: Mode.input_pullup
        },
        {
          text: 'output',
          value: Mode.output
        }
      ];
    }

    get LEVEL_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'raspberrypico.levelMenu.high',
                    default: 'high',
                    description: 'label for high level'
                }),
                value: Level.High
            },
            {
                text: formatMessage({
                    id: 'raspberrypico.levelMenu.low',
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
                text: '26_A0',
                value: analogPins.A0
            },
            {
                text: '27_A1',
                value: analogPins.A1
            },
            {
                text: '28_A2',
                value: analogPins.A2
            }
        ];
    }

    /**
     * Construct a set of RaspberryPico blocks.
     * @param {Runtime} runtime - the OpenBlock runtime.
     */
    constructor (runtime) {
        /**
         * The OpenBlock runtime.
         * @type {Runtime}
         */
        this.runtime = runtime;

        // Create a new RaspberryPico peripheral instance
        this._peripheral = new RaspberryPico(this.runtime, OpenBlockRaspberryPicoDevice.DEVICE_ID);
    }



    /**
     * @returns {Array.<object>} metadata for this extension and its blocks.
     */
    getInfo () {
        return [
		{
			id: 'RaspberryPico',
			name: formatMessage({
				id: 'raspberrypico.category.raspberrypico',
				default: 'RPi Pico',
				description: 'The name of the raspberrypico device'
			}),
			menuIconURI: menuIconURI,
			color1: '#FF009D',
			color2: '#E6008E',
			color3: '#CC007E',
			blocks: [
				{
					opcode: 'whenraspberrypicobegin',
					text: formatMessage({
						id: 'raspberrypico.raspberrypico.header',
						default: 'when RPi Pico begins',
						description: 'raspberrypico header'
					}),
					blockType: BlockType.HAT
				}
				],
				menus: {}
		},
		{
            id: 'pin',
            name: formatMessage({
                id: 'raspberrypico.category.pins',
                default: 'Pins',
                description: 'The name of the raspberrypico device pin category'
            }),
            color1: '#4C97FF',
            color2: '#3373CC',
            color3: '#3373CC',

            blocks: [
              {
                  opcode: 'setPinMode',
                  text: formatMessage({
                      id: 'raspberrypico.pins.setPinMode',
                      default: 'set pin [PIN] mode [MODE]',
                      description: 'raspberrypico set pin mode'
                  }),
                  blockType: BlockType.COMMAND,
                  arguments: {
                      PIN: {
                          type: ArgumentType.STRING,
                          menu: 'pins',
                          defaultValue: Pins.P25
                      },
                      MODE: {
                          type: ArgumentType.STRING,
                          menu: 'mode',
                          defaultValue: Mode.input
                      }
                  }
              },
                {
                    opcode: 'setDigitalOutput',
                    text: formatMessage({
                        id: 'raspberrypico.pins.setDigitalOutput',
                        default: 'set digital pin [PIN] out [LEVEL]',
                        description: 'raspberrypico set digital pin out'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        PIN: {
                            type: ArgumentType.STRING,
                            menu: 'pins',
                            defaultValue: Pins.P25
                        },
                        LEVEL: {
                            type: ArgumentType.STRING,
                            menu: 'level',
                            defaultValue: Level.High
                        }
                    }
                },
                {
                    opcode: 'setDigitalToggle',
                    text: formatMessage({
                        id: 'raspberrypico.pins.setDigitalToggle',
                        default: 'set toggle digital pin [PIN]',
                        description: 'raspberrypico set digital pin toggle'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        PIN: {
                            type: ArgumentType.STRING,
                            menu: 'pins',
                            defaultValue: Pins.P25
                        }
                    }
                },
                {
                    opcode: 'setPwmOutput',
                    text: formatMessage({
                        id: 'raspberrypico.pins.setPwmOutput',
                        default: 'set pwm pin [PIN] output [OUT] freq [FREQ]',
                        description: 'raspberrypico set pwm pin out'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        PIN: {
                            type: ArgumentType.STRING,
                            menu: 'pins',
                            defaultValue: Pins.P25
                        },
                        OUT: {
                            type: ArgumentType.NUMBER,
                            defaultValue: '65025'
                        },
                        FREQ: {
                            type: ArgumentType.NUMBER,
                            defaultValue: '1000'
                        }
                    }
                },
                {
                    opcode: 'readDigitalPin',
                    text: formatMessage({
                        id: 'raspberrypico.pins.readDigitalPin',
                        default: 'read digital pin [PIN]',
                        description: 'raspberrypico read digital pin'
                    }),
                    blockType: BlockType.BOOLEAN,
                    arguments: {
                        PIN: {
                            type: ArgumentType.STRING,
                            menu: 'pins',
                            defaultValue: Pins.P25
                        }
                    }
                },
                {
                    opcode: 'readAnalogPin',
                    text: formatMessage({
                        id: 'raspberrypico.pins.readAnalogPin',
                        default: 'read analog pin [PIN]',
                        description: 'raspberrypico read analog pin'
                    }),
                    blockType: BlockType.REPORTER,
                    arguments: {
                        PIN: {
                            type: ArgumentType.STRING,
                            menu: 'analogPins',
                            defaultValue: analogPins.A0
                        }
                    }
                }
            ],
            menus: {
                pins: {
                    items: this.PINS_MENU
                },
                level: {
                    items: this.LEVEL_MENU
                },
                mode:{
                    items: this.MODE_MENU
                },
                analogPins: {
                    items: this.ANALOG_PINS_MENU
                }
            }
        },
        {
            id: 'console',
            name: formatMessage({
                id: 'raspberrypico.category.console',
                default: 'Console',
                description: 'The name of the raspberrypico device console category'
            }),
            color1: '#FF3399',
            color2: '#FF3399',
            color3: '#FF3399',

            blocks: [
                {
                    opcode: 'consolePrint',
                    text: formatMessage({
                        id: 'raspberrypico.console.consolePrint',
                        default: 'print [TEXT]',
                        description: 'raspberrypico console print'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        TEXT: {
                            type: ArgumentType.STRING,
                            defaultValue: 'Hello OpenBlock'
                        }
                    }
                }
            ],
            menus: { }
        }
        ];
    }
}

module.exports = OpenBlockRaspberryPicoDevice;

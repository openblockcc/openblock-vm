const OpenBlockArduinoLeonardoDevice = require('../arduinoLeonardo/arduinoLeonardo');

const ArduinoPeripheral = require('../common/arduino-peripheral');

/**
 * The list of USB device filters.
 * @readonly
 */
const PNPID_LIST = [
    // https://github.com/arduino/Arduino/blob/1.8.0/hardware/arduino/avr/boards.txt#L268-L275
    'USB\\VID_2341&PID_0036',
    'USB\\VID_2341&PID_8036',
    'USB\\VID_2A03&PID_0036',
    'USB\\VID_2A03&PID_8036',
    // https://github.com/movuino/Firmware/blob/ec6edb735454f53afee635cb31fdf29ea5890ca9/sketchbook/hardware/sparkfun/avr/boards.txt#L109-L112
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
 * Configuration of build and flash. Used by arduino-cli.
 * @readonly
 */
const DIVECE_OPT = {
    type: 'arduino',
    fqbn: 'SparkFun:avr:makeymakey',
    // The time to wait for USB re-recognition after uploading is completed. Darwin and linux will take more time to
    // rerecognize device, therefore, this parameter should be tested and confirmed under Linux or Darwin system.
    postUploadDelay: 3000
};

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

/**
  * Manage communication with a Arduino MakeyMakey peripheral over a OpenBlock Link client socket.
  */
class MakeyMakey extends ArduinoPeripheral{
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
   * OpenBlock blocks to interact with a Arduino MakeyMakey peripheral.
   */
class OpenBlockMakeyMakeyDevice extends OpenBlockArduinoLeonardoDevice{

    /**
       * @return {string} - the ID of this extension.
       */
    get DEVICE_ID () {
        return 'makeyMakey';
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

    /**
      * Construct a set of Arduino blocks.
      * @param {Runtime} runtime - the OpenBlock runtime.
      * @param {string} originalDeviceId - the original id of the peripheral, like xxx_arduinoUno
      */
    constructor (runtime, originalDeviceId) {
        super(runtime, originalDeviceId);

        // Create a new Arduino makeymakey peripheral instance
        this._peripheral = new MakeyMakey(this.runtime, this.DEVICE_ID, originalDeviceId);
    }
}

module.exports = OpenBlockMakeyMakeyDevice;

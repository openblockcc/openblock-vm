const JSONRPC = require('../util/jsonrpc');

class Serialport extends JSONRPC {

    /**
     * A serialport peripheral socket object.  It handles connecting, over web sockets, to
     * serialport peripherals, and reading and writing data to them.
     * @param {Runtime} runtime - the Runtime for sending/receiving GUI update events.
     * @param {string} deviceId - the id of the device using this socket.
     * @param {object} peripheralOptions - the list of options for peripheral discovery.
     * @param {object} connectCallback - a callback for connection.
     * @param {object} resetCallback - a callback for resetting extension state.
     */
    constructor (runtime, deviceId, peripheralOptions, connectCallback = null, resetCallback = null) {
        super();

        this._socket = runtime.getScratchLinkSocket('SERIALPORT');
        this._socket.setOnOpen(this.requestPeripheral.bind(this));
        this._socket.setOnClose(this.handleDisconnectError.bind(this));
        this._socket.setOnError(this._handleRequestError.bind(this));
        this._socket.setHandleMessage(this._handleMessage.bind(this));

        this._sendMessage = this._socket.sendMessage.bind(this._socket);

        this._availablePeripherals = {};
        this._connectCallback = connectCallback;
        this._connected = false;
        this._onMessage = null;
        this._resetCallback = resetCallback;
        this._discoverTimeoutID = null;
        this._deviceId = deviceId;
        this._peripheralOptions = peripheralOptions;
        this._runtime = runtime;

        this._socket.open();
    }

    /**
     * Request connection to the peripheral.
     * If the web socket is not yet open, request when the socket promise resolves.
     */
    requestPeripheral () {
        this._availablePeripherals = {};
        if (this._discoverTimeoutID) {
            window.clearTimeout(this._discoverTimeoutID);
        }
        this._discoverTimeoutID = window.setTimeout(this._handleDiscoverTimeout.bind(this), 15000);
        this.sendRemoteRequest('discover', this._peripheralOptions)
            .catch(e => {
                this._handleRequestError(e);
            });
    }

    /**
     * Try connecting to the input peripheral id, and then call the connect
     * callback if connection is successful.
     * @param {number} id - the id of the peripheral to connect to
     * @param {object} config - communacation configuration of peripheral
     */
    connectPeripheral (id, config) {
        this.sendRemoteRequest('connect', {peripheralId: id, peripheralConfig: config})
            .then(() => {
                this._connected = true;
                this._runtime.emit(this._runtime.constructor.PERIPHERAL_CONNECTED);
                if (this._connectCallback) {
                    this._connectCallback();
                }
            })
            .catch(e => {
                this._handleRequestError(e);
            });
    }

    /**
     * Close the websocket.
     */
    disconnect () {
        if (this._connected) {
            this._connected = false;
        }

        if (this._socket.isOpen()) {
            this._socket.close();
        }

        if (this._discoverTimeoutID) {
            window.clearTimeout(this._discoverTimeoutID);
        }

        // Sets connection status icon to orange
        this._runtime.emit(this._runtime.constructor.PERIPHERAL_DISCONNECTED);
    }

    /**
     * @return {bool} whether the peripheral is connected.
     */
    isConnected () {
        return this._connected;
    }

    setBaudrate (baudRate) {
        if (baudRate) {
            const params = {baudRate};
            return this.sendRemoteRequest('updateBaudrate', params)
                .catch(e => {
                    this.handleDisconnectError(e);
                });
        }
    }

    /**
     * Set serialport start read.
     * @param {object} onMessage - callback for characteristic change notifications.
     * @return {Promise} - a promise from the remote read request.
     */
    read (onMessage = null) {
        if (onMessage) {
            this._onMessage = onMessage;
        }
        return this.sendRemoteRequest('read')
            .catch(e => {
                this.handleDisconnectError(e);
            });
    }

    /**
     * Write data to the serialport.
     * @param {string} message - the message to send.
     * @param {string} encoding - the message encoding type.
     * @return {Promise} - a promise from the remote send request.
     */
    write (message, encoding = null) {
        const params = {message};
        if (encoding) {
            params.encoding = encoding;
        }
        return this.sendRemoteRequest('write', params)
            .catch(e => {
                this.handleDisconnectError(e);
            });
    }

    /**
     * Upload code to the peripheral.
     * @param {string} message - the code to upload.
     * @param {object} config - the configuration of upload process.
     * @param {string} encoding - the message encoding type.
     * @return {Promise} - a promise from the remote send request.
     */
    upload (message, config, encoding = null) {
        const params = {message, config};
        if (encoding) {
            params.encoding = encoding;
        }
        params.library = this._runtime.getCurrentDeviceExtensionLibrary();
        return this.sendRemoteRequest('upload', params)
            .catch(e => {
                this.handleDisconnectError(e);
            });
    }

    /**
     * Upload realtime firmware to the peripheral.
     * @param {object} config - the configuration of upload process.
     * @return {Promise} - a promise from the remote send request.
     */
    uploadFirmware (config) {
        return this.sendRemoteRequest('uploadFirmware', config)
            .catch(e => {
                this.handleDisconnectError(e);
            });
    }

    /**
     * abort the uploading process.
     * @param {object} config - the configuration of upload process.
     * @return {Promise} - a promise from the remote send request.
     */
    abort (config) {
        const params = {config};
        return this.sendRemoteRequest('abort', params)
            .catch(e => {
                this.handleDisconnectError(e);
            });
    }

    /**
     * Handle a received call from the socket.
     * @param {string} method - a received method label.
     * @param {object} params - a received list of parameters.
     * @return {object} - optional return value.
     */
    didReceiveCall (method, params) {
        switch (method) {
        case 'didDiscoverPeripheral':
            this._availablePeripherals[params.peripheralId] = params;
            this._runtime.emit(
                this._runtime.constructor.PERIPHERAL_LIST_UPDATE,
                this._availablePeripherals
            );
            if (this._discoverTimeoutID) {
                window.clearTimeout(this._discoverTimeoutID);
            }
            break;
        case 'peripheralUnplug':
            this.handleDisconnectError();
            break;
        case 'onMessage':
            if (this._onMessage) {
                this._onMessage(params.message);
            }
            break;
        case 'uploadStdout':
            this._runtime.emit(
                this._runtime.constructor.PERIPHERAL_UPLOAD_STDOUT, {
                    message: params.message
                });
            break;
        case 'uploadError':
            this._runtime.emit(
                this._runtime.constructor.PERIPHERAL_UPLOAD_ERROR, {
                    message: params.message
                });
            break;
        case 'uploadSuccess':
            this._runtime.emit(
                this._runtime.constructor.PERIPHERAL_UPLOAD_SUCCESS);
            break;
        case 'ping':
            return 42;
        }
    }

    /**
     * Handle an error resulting from losing connection to a peripheral.
     *
     * This could be due to:
     * - battery depletion
     * - going out of bluetooth range
     * - being powered down
     *
     * Disconnect the socket, and if the extension using this socket has a
     * reset callback, call it. Finally, emit an error to the runtime.
     */
    handleDisconnectError (/* e */) {
        if (!this._connected) return;

        this.disconnect();

        if (this._resetCallback) {
            this._resetCallback();
        }

        this._runtime.emit(this._runtime.constructor.PERIPHERAL_CONNECTION_LOST_ERROR, {
            message: `Scratch lost connection to`,
            deviceId: this._deviceId
        });
    }

    /**
     * Handle an error resulting from losing connection to a peripheral realtime protocal.
     *
     * This could be due to:
     * - peripheral didn't running the realtime protocal like firmata.
     * - peripheral is not running.
     * @param {string} e - error message.
     */
    handleRealtimeDisconnectError (e) {
        this._runtime.emit(this._runtime.constructor.PERIPHERAL_REALTIME_CONNECTION_LOST_ERROR, {
            message: e,
            deviceId: this._deviceId
        });
    }

    handleRealtimeConnectSucess () {
        this._runtime.emit(this._runtime.constructor.PERIPHERAL_REALTIME_CONNECT_SUCCESS, {deviceId: this._deviceId});
    }

    _handleRequestError (/* e */) {
        this._runtime.emit(this._runtime.constructor.PERIPHERAL_REQUEST_ERROR, {
            message: `Scratch lost connection to`,
            deviceId: this._deviceId
        });
    }

    _handleDiscoverTimeout () {
        if (this._discoverTimeoutID) {
            window.clearTimeout(this._discoverTimeoutID);
        }
        this._runtime.emit(this._runtime.constructor.PERIPHERAL_SCAN_TIMEOUT);
    }
}

module.exports = Serialport;

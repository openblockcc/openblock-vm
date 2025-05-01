const fetch = require('node-fetch');
const loadjs = require('loadjs');
const formatMessage = require('format-message');
const validUrl = require('valid-url');

const dispatch = require('../dispatch/central-dispatch');
const log = require('../util/log');
const maybeFormatMessage = require('../util/maybe-format-message');

const BlockType = require('./block-type');

// Local resources server address
const localResourcesServerUrl = 'http://127.0.0.1:20112/';

// These extensions are currently built into the VM repository but should not be loaded at startup.
// TODO: move these out into a separate repository?
// TODO: change extension spec so that library info, including extension ID, can be collected through static methods

const builtinExtensions = {
    // This is an example that isn't loaded with the other core blocks,
    // but serves as a reference for loading core blocks as extensions.
    coreExample: () => require('../blocks/scratch3_core_example'),
    // These are the non-core built-in extensions.
    pen: () => require('../extensions/scratch3_pen'),
    music: () => require('../extensions/scratch3_music'),
    text2speech: () => require('../extensions/scratch3_text2speech'),
    translate: () => require('../extensions/scratch3_translate'),
    videoSensing: () => require('../extensions/scratch3_video_sensing'),
    makeymakey: () => require('../extensions/scratch3_makeymakey')
};

const builtinDevices = {
    // Arduino Uno
    arduinoUno: () => require('../devices/arduinoUno/arduinoUno'),
    arduinoNano: () => require('../devices/arduinoUno/arduinoNano'),
    arduinoUnoUltra: () => require('../devices/arduinoUno/arduinoUnoUltra'),
    arduinoUnoSE: () => require('../devices/arduinoUno/arduinoUnoSE'),
    // Arduino Leonardo
    arduinoLeonardo: () => require('../devices/arduinoLeonardo/arduinoLeonardo'),
    makeyMakey: () => require('../devices/arduinoLeonardo/makeyMakey'),
    // Arduino Mega2560
    arduinoMega2560: () => require('../devices/arduinoMega2560/arduinoMega2560'),
    // Arduino Uno R4 Minima
    arduinoUnoR4Minima: () => require('../devices/arduinoUnoR4Minima/arduinoUnoR4Minima'),
    // Arduino Uno R4 WiFi
    arduinoUnoR4Wifi: () => require('../devices/arduinoUnoR4Wifi/arduinoUnoR4Wifi'),
    // Esp32
    arduinoEsp32: () => require('../devices/arduinoEsp32/arduinoEsp32'),
    // Esp32-S3
    arduinoEsp32S3: () => require('../devices/arduinoEsp32S3/arduinoEsp32S3'),
    // Esp8266
    arduinoEsp8266: () => require('../devices/arduinoEsp8266/arduinoEsp8266'),
    arduinoEsp8266NodeMCU: () => require('../devices/arduinoEsp8266/arduinoEsp8266NodeMCU'),
    // K210
    arduinoK210: () => require('../devices/arduinoK210/arduinoK210'),
    arduinoK210MaixDock: () => require('../devices/arduinoK210/arduinoK210MaixDock'),
    arduinoK210Maixduino: () => require('../devices/arduinoK210/arduinoK210Maixduino'),
    // Raspberry Pi Pico
    arduinoRaspberryPiPico: () => require('../devices/arduinoRaspberryPiPico/arduinoRaspberryPiPico'),
    // Raspberry Pi Pico W
    arduinoRaspberryPiPicoW: () => require('../devices/arduinoRaspberryPiPicoW/arduinoRaspberryPiPicoW'),
    // Raspberry Pi Pico 2
    arduinoRaspberryPiPico2: () => require('../devices/arduinoRaspberryPiPico2/arduinoRaspberryPiPico2'),
    // Raspberry Pi Pico 2W
    arduinoRaspberryPiPico2W: () => require('../devices/arduinoRaspberryPiPico2W/arduinoRaspberryPiPico2W'),
    // Microbit
    microbit: () => require('../devices/microbit/microbit'),
    microbitV2: () => require('../devices/microbit/microbitV2')

    // TODO: transform these to device extension.
    // wedo2: () => require('../extensions/scratch3_wedo2'),
    // ev3: () => require('../extensions/scratch3_ev3'),
    // boost: () => require('../extensions/scratch3_boost'),
    // gdxfor: () => require('../extensions/scratch3_gdx_for')
};

/**
 * @typedef {object} ArgumentInfo - Information about an extension block argument
 * @property {ArgumentType} type - the type of value this argument can take
 * @property {*|undefined} default - the default value of this argument (default: blank)
 */

/**
 * @typedef {object} ConvertedBlockInfo - Raw extension block data paired with processed data ready for scratch-blocks
 * @property {ExtensionBlockMetadata} info - the raw block info
 * @property {object} json - the scratch-blocks JSON definition for this block
 * @property {string} xml - the scratch-blocks XML definition for this block
 */

/**
 * @typedef {object} CategoryInfo - Information about a block category
 * @property {string} id - the unique ID of this category
 * @property {string} name - the human-readable name of this category
 * @property {string|undefined} blockIconURI - optional URI for the block icon image
 * @property {string} color1 - the primary color for this category, in '#rrggbb' format
 * @property {string} color2 - the secondary color for this category, in '#rrggbb' format
 * @property {string} color3 - the tertiary color for this category, in '#rrggbb' format
 * @property {Array.<ConvertedBlockInfo>} blocks - the blocks, separators, etc. in this category
 * @property {Array.<object>} menus - the menus provided by this category
 */

/**
 * @typedef {object} PendingExtensionWorker - Information about an extension worker still initializing
 * @property {string} extensionURL - the URL of the extension to be loaded by this worker
 * @property {Function} resolve - function to call on successful worker startup
 * @property {Function} reject - function to call on failed worker startup
 */

class ExtensionManager {
    constructor (runtime) {
        /**
         * The ID number to provide to the next extension worker.
         * @type {int}
         */
        this.nextExtensionWorker = 0;

        /**
         * FIFO queue of extensions which have been requested but not yet loaded in a worker,
         * along with promise resolution functions to call once the worker is ready or failed.
         *
         * @type {Array.<PendingExtensionWorker>}
         */
        this.pendingExtensions = [];

        /**
         * Map of worker ID to workers which have been allocated but have not yet finished initialization.
         * @type {Array.<PendingExtensionWorker>}
         */
        this.pendingWorkers = [];

        /**
         * Map of scratch extensions that can be loaded.
         * @type {Array.<Extensions>}
         */
        this._extensionsList = [];

        /**
         * Set of loaded extension URLs/IDs (equivalent for built-in extensions).
         * @type {Set.<string>}
         * @private
         */
        this._loadedExtensions = new Map();

        /**
         * Set of loaded device URLs/IDs (equivalent for built-in devices).
         * @type {Set.<string>}
         * @private
         */
        this._loadedDevice = new Map();

        /**
         * Map of device extensions that can be loaded.
         * @type {Array.<DeviceExtensions>}
         */
        this._deviceExtensionsList = [];

        /**
         * Keep a reference to the runtime so we can construct internal extension objects.
         * TODO: remove this in favor of extensions accessing the runtime as a service.
         * @type {Runtime}
         */
        this.runtime = runtime;

        dispatch.setService('extensions', this).catch(e => {
            log.error(`ExtensionManager was unable to register extension service: ${JSON.stringify(e)}`);
        });
    }

    /**
     * Check whether an extension is registered or is in the process of loading. This is intended to control loading or
     * adding extensions so it may return `true` before the extension is ready to be used. Use the promise returned by
     * `loadExtensionURL` if you need to wait until the extension is truly ready.
     * @param {string} extensionID - the ID of the extension.
     * @returns {boolean} - true if loaded, false otherwise.
     */
    isExtensionLoaded (extensionID) {
        return this._loadedExtensions.has(extensionID);
    }

    /**
     * Check whether an device is registered or is in the process of loading. This is intended to control loading or
     * adding device so it may return `true` before the device is ready to be used. Use the promise returned by
     * `loadDeviceURL` if you need to wait until the device is truly ready.
     * @param {string} deviceID - the ID of the device.
     * @returns {boolean} - true if loaded, false otherwise.
     */
    isDeviceLoaded (deviceID) {
        return this._loadedDevice.has(deviceID);
    }

    /**
     * Get extensions list from local server.
     * @param {string} extensions - raw extensions data
     * @returns {Promise} resolved extension list has been fetched or failure
     */
    getExtensionsList (extensions) {
        return new Promise(resolve => {
            const processedExtensions = extensions.map(extension => {
                if (this.isExtensionLoaded(extension.extensionId)) {
                    extension.isLoaded = true;
                } else {
                    extension.isLoaded = false;
                }
                return extension;
            });

            return resolve(processedExtensions);
        });
    }

    /**
     * Synchronously load an internal extension (core or non-core) by ID. This call will
     * fail if the provided id is not does not match an internal extension.
     * @param {string} extensionId - the ID of an internal extension
     */
    loadExtensionIdSync (extensionId) {
        if (!builtinExtensions.hasOwnProperty(extensionId)) {
            log.warn(`Could not find extension ${extensionId} in the built in extensions.`);
            return;
        }

        /** @TODO dupe handling for non-builtin extensions. See commit 670e51d33580e8a2e852b3b038bb3afc282f81b9 */
        if (this.isExtensionLoaded(extensionId)) {
            const message = `Rejecting attempt to load a second extension with ID ${extensionId}`;
            log.warn(message);
            return;
        }

        const extension = builtinExtensions[extensionId]();
        const extensionInstance = new extension(this.runtime);
        const serviceName = this._registerInternalExtension(extensionInstance);
        this._loadedExtensions.set(extensionId, serviceName);
        this.runtime.addScratchExtension(extensionId);
    }

    /**
     * Load an extension by URL or internal extension ID
     * @param {string} extensionURL - the URL for the extension to load OR the ID of an internal extension
     * @returns {Promise} resolved once the extension is loaded and initialized or rejected on failure
     */
    loadExtensionURL (extensionURL) {
        if (builtinExtensions.hasOwnProperty(extensionURL)) {
            /** @TODO dupe handling for non-builtin extensions. See commit 670e51d33580e8a2e852b3b038bb3afc282f81b9 */
            if (this.isExtensionLoaded(extensionURL)) {
                const message = `Rejecting attempt to load a second extension with ID ${extensionURL}`;
                log.warn(message);
                return Promise.resolve();
            }

            const extension = builtinExtensions[extensionURL]();
            const extensionInstance = new extension(this.runtime);
            const serviceName = this._registerInternalExtension(extensionInstance);
            this._loadedExtensions.set(extensionURL, serviceName);
            this.runtime.addScratchExtension(extensionURL);
            return Promise.resolve();
        }

        return new Promise((resolve, reject) => {
            // If we `require` this at the global level it breaks non-webpack targets, including tests
            const ExtensionWorker = require('worker-loader?name=extension-worker.js!./extension-worker');

            this.pendingExtensions.push({extensionURL, resolve, reject});
            dispatch.addWorker(new ExtensionWorker());
        });
    }

    /**
     * Unload an extension by URL or internal extension ID
     * @param {string} extensionURL - the URL for the extension to load OR the ID of an internal extension
     */
    unloadExtension (extensionURL) {
        this._loadedExtensions.delete(extensionURL);
        this.runtime.removeScratchExtension(extensionURL);
    }

    /**
     * Unload all extension
     */
    clearExtensions () {
        this._loadedExtensions.clear();
        this.runtime.clearScratchExtension();
    }

    /**
     * Get unbuild-in devices list from local server.
     * @returns {Promise} resolved devices list has been fetched or failure
     */
    getDeviceList () {
        return new Promise(resolve => {
            fetch(`${localResourcesServerUrl}devices/${formatMessage.setup().locale}.json`)
                .then(response => response.json())
                .then(devices => {
                    // filter unsupported distribution content
                    let filteredDevices = [];
                    let currentBases = 'none';

                    devices.forEach(dev => {
                        // Filter out devices that are not inherited but have multiple programming
                        // frameworks, and only keep devices with the Arduino framework
                        const deviceId = dev.deviceId;
                        if (!deviceId.startsWith('arduino') && !deviceId.startsWith('microPython')) {
                            currentBases = deviceId;
                            filteredDevices.push(dev);
                        } else if (deviceId.indexOf(currentBases.charAt(0).toUpperCase() +
                            currentBases.slice(1)) === -1) {
                            currentBases = deviceId;
                            filteredDevices.push(dev);
                        } else if (deviceId.startsWith('arduino')) {
                            filteredDevices.pop();
                            filteredDevices.push(dev);
                        }
                    });

                    filteredDevices = filteredDevices.filter(dev => {
                        // Filter out external non-inherited devices
                        if ((dev.deviceId.indexOf('_') === -1) && (!!dev.name)) {
                            return false;
                        }

                        // Filter out devices that are inherited but have multiple programming
                        // frameworks, and only keep devices with the Arduino framework
                        if ((dev.deviceId.indexOf('_') !== -1) && !!dev.typeList &&
                            (dev.deviceId.indexOf('arduino') === -1)) {
                            return false;
                        }
                        return true;
                    });

                    devices = filteredDevices.map(dev => {
                        dev.hide = false;
                        dev.iconURL = localResourcesServerUrl + dev.iconURL;
                        dev.connectionIconURL = localResourcesServerUrl + dev.connectionIconURL;
                        dev.connectionSmallIconURL = localResourcesServerUrl + dev.connectionSmallIconURL;
                        return dev;
                    });
                    return resolve(devices);
                }, err => {
                    log.warn(`Can not fetch data from local device server: ${err}`);
                    return resolve();
                });
        });
    }


    /**
     * Load an device by URL or internal device ID
     * @param {object} device - the device to be load
     * @returns {Promise} resolved once the device is loaded and initialized or rejected on failure
     */
    loadDeviceURL (device) {
        // if no deviceid return
        if (device.deviceId === 'null') {
            this.clearDevice();
            return Promise.resolve();
        }
        const {deviceId, type, pnpidList} = device;

        const realDeviceId = this.runtime.analysisRealDeviceId(deviceId);

        if (builtinDevices.hasOwnProperty(realDeviceId)) {
            if (this.isDeviceLoaded(deviceId)) {
                const message = `Rejecting attempt to load a device twice with ID ${deviceId}`;
                log.warn(message);
                return Promise.resolve();
            }

            // Try to disconnect the old device before change device.
            this.runtime.disconnectPeripheral(this.runtime.getDevice().deviceId);

            this.runtime.setDevice({deviceId: deviceId, type: type, pnpIdList: pnpidList});
            this.runtime.clearMonitor();
            const dev = builtinDevices[realDeviceId]();
            const deviceInstance = new dev(this.runtime, deviceId);
            const serviceName = this._registerInternalExtension(deviceInstance);
            this._loadedDevice.clear();

            this._loadedDevice.set(deviceId, serviceName);

            // Clear current extentions.
            this.clearExtensions();
            this.clearDeviceExtension();

            return Promise.resolve();
        }

        return Promise.reject(`Error while load device can not find device: ${deviceId}`);
    }

    /**
     * Clear curent device
     */
    clearDevice () {
        if (this.runtime.getDevice().deviceId) {
            this.runtime.disconnectPeripheral(this.runtime.getDevice().deviceId);

            const deviceId = this.runtime.getDevice().deviceId;

            this.runtime.clearDevice();
            this.runtime.clearMonitor();
            this._loadedDevice.clear();

            // Clear current extentions.
            this.clearExtensions();
            this.clearDeviceExtension();

            this.runtime.emit(this.runtime.constructor.SCRATCH_EXTENSION_REMOVED, {deviceId});
        }
    }

    /**
     * Get device extensions list from local server.
     * @returns {Promise} resolved extension list has been fetched or failure
     */
    getDeviceExtensionsList () {
        return new Promise(resolve => {
            fetch(`${localResourcesServerUrl}extensions/${formatMessage.setup().locale}.json`)
                .then(response => response.json())
                .then(extensions => {
                    // filter unsupported distribution content
                    let filteredExtensions = [];
                    filteredExtensions = extensions.filter(extension => {
                        // if the extension only has main.js but no blocks.js,
                        // the plugin should be blocked
                        if (!!extension.main && !extension.blocks) {
                            return false;
                        }
                        return true;
                    });

                    extensions = filteredExtensions.map(extension => {
                        extension.iconURL = localResourcesServerUrl + extension.iconURL;
                        if (this.isDeviceExtensionLoaded(extension.extensionId)) {
                            extension.isLoaded = true;
                        }
                        return extension;
                    });
                    this._deviceExtensionsList = extensions;
                    return resolve(this._deviceExtensionsList);
                }, err => {
                    log.warn(`Can not fetch data from local extension server: ${err}`);
                    return resolve();
                });
        });
    }

    /**
     * Check whether an device extension is loaded.
     * @param {string} deviceExtensionId - the ID of the device extension.
     * @returns {boolean} - true if loaded, false otherwise.
     */
    isDeviceExtensionLoaded (deviceExtensionId) {
        return this.runtime.isDeviceExtensionLoaded(deviceExtensionId);
    }

    /**
     * Load an device extension by device extension ID
     * @param {string} deviceExtensionId - the ID of an device extension
     * @returns {Promise} resolved once the device extension is loaded or rejected on failure
     */
    loadDeviceExtension (deviceExtensionId) {
        return new Promise((resolve, reject) => {
            const deviceExtension = this._deviceExtensionsList.find(ext => ext.extensionId === deviceExtensionId);
            if (typeof deviceExtension === 'undefined') {
                return reject(`Error while loadDeviceExtension device extension ` +
                    `can not find device extension: ${deviceExtensionId}`);
            }

            let registerUrls = [];

            registerUrls.push(deviceExtension.toolbox);
            registerUrls.push(deviceExtension.blocks);
            registerUrls.push(deviceExtension.translations);
            registerUrls.push(deviceExtension.generator);

            // Remove null values
            registerUrls = registerUrls.filter(url => url !== null && typeof url !== 'undefined' && url !== '');

            // If it is a local file, add the localhost address in front
            registerUrls = registerUrls.map(url => {
                if (!validUrl.isWebUri(url)) {
                    return localResourcesServerUrl + url;
                }
                return url;
            });

            // clear global register before load external extension.
            global.registerToolboxs = null;
            global.registerBlocks = null;
            global.registerGenerators = null;
            global.registerBlocksMessages = null;

            loadjs(registerUrls, {returnPromise: true})
                .then(() => {
                    const getToolboxXML = global.registerToolboxs;
                    this.runtime.addDeviceExtension(deviceExtensionId, getToolboxXML(), deviceExtension.library);

                    const deviceExtensionsRegister = {
                        defineBlocks: global.registerBlocks,
                        defineGenerators: global.registerGenerators,
                        defineMessages: global.registerBlocksMessages
                    };

                    this.runtime.emit(this.runtime.constructor.DEVICE_EXTENSION_ADDED, deviceExtensionsRegister);
                    return resolve();
                })
                .catch(err => reject(`Error while load device extension ` +
                    `${deviceExtension.extensionId}'s js file: ${err}`));
        });
    }

    /**
     * Unload an device extension by device extension ID
     * @param {string} deviceExtensionId - the ID of an device extension
     */
    unloadDeviceExtension (deviceExtensionId) {
        this.runtime.removeDeviceExtension(deviceExtensionId);
        this.runtime.emit(this.runtime.constructor.DEVICE_EXTENSION_REMOVED);
    }

    /**
     * Unload all device extensions
     */
    clearDeviceExtension () {
        const loadedDeviceExtensionId = this.runtime.getLoadedDeviceExtension();
        loadedDeviceExtensionId.forEach(id => {
            this.unloadDeviceExtension(id);
        });
    }

    /**
     * Get id of extension or device from service name
     * @param {string} serviceName - the name of service
     * @returns {object} the id of extension or device
     */
    getIdFromServiceName (serviceName) {
        if (serviceName) {
            let extensionId = null;
            let deviceId = null;
            // get deviceId or extensions Id
            if (serviceName.startsWith('device')) {
                deviceId = serviceName.split('_')[2];
            } else {
                extensionId = serviceName.split('_')[2];
            }
            return {extensionId, deviceId};
        }
        return null;
    }

    /**
     * Regenerate blockinfo for any loaded extensions
     * @returns {Promise} resolved once all the extensions have been reinitialized
     */
    refreshBlocks () {
        const allServiceName = Array.from(this._loadedExtensions.values())
            .concat(Array.from(this._loadedDevice.values()));
        const allPromises = allServiceName.map(serviceName =>
            dispatch.call(serviceName, 'getInfo')
                .then(info => {
                    info = this._prepareExtensionInfo(serviceName, info, this.getIdFromServiceName(serviceName));
                    dispatch.call('runtime', '_refreshExtensionPrimitives', info);
                })
                .catch(e => {
                    log.error(`Failed to refresh built-in extension primitives: ${JSON.stringify(e)}`);
                })
        );
        return Promise.all(allPromises);
    }

    allocateWorker () {
        const id = this.nextExtensionWorker++;
        const workerInfo = this.pendingExtensions.shift();
        this.pendingWorkers[id] = workerInfo;
        return [id, workerInfo.extensionURL];
    }

    /**
     * Synchronously collect extension metadata from the specified service and begin the extension registration process.
     * @param {string} serviceName - the name of the service hosting the extension.
     */
    registerExtensionServiceSync (serviceName) {
        const info = dispatch.callSync(serviceName, 'getInfo');
        this._registerExtensionInfo(serviceName, info);
    }

    /**
     * Collect extension metadata from the specified service and begin the extension registration process.
     * @param {string} serviceName - the name of the service hosting the extension.
     */
    registerExtensionService (serviceName) {
        dispatch.call(serviceName, 'getInfo').then(info => {
            this._registerExtensionInfo(serviceName, info);
        });
    }

    /**
     * Called by an extension worker to indicate that the worker has finished initialization.
     * @param {int} id - the worker ID.
     * @param {*?} e - the error encountered during initialization, if any.
     */
    onWorkerInit (id, e) {
        const workerInfo = this.pendingWorkers[id];
        delete this.pendingWorkers[id];
        if (e) {
            workerInfo.reject(e);
        } else {
            workerInfo.resolve(id);
        }
    }

    /**
     * Register an internal (non-Worker) extension object
     * @param {object} extensionObject - the extension object to register
     * @returns {string} The name of the registered extension service
     */
    _registerInternalExtension (extensionObject) {
        const extensionId = extensionObject.EXTENSION_ID;
        const fakeWorkerId = this.nextExtensionWorker++;
        let serviceName;
        if (extensionId) {
            serviceName = `extension_${fakeWorkerId}_${extensionId}`;
        } else {
            serviceName = `device_${fakeWorkerId}_${extensionObject.DEVICE_ID}`;
        }
        dispatch.setServiceSync(serviceName, extensionObject);
        dispatch.callSync('extensions', 'registerExtensionServiceSync', serviceName);
        return serviceName;
    }

    /**
     * Sanitize extension info then register its primitives with the VM.
     * @param {string} serviceName - the name of the service hosting the extension
     * @param {ExtensionInfo} extensionInfo - the extension's metadata
     * @private
     */
    _registerExtensionInfo (serviceName, extensionInfo) {
        extensionInfo = this._prepareExtensionInfo(serviceName, extensionInfo, this.getIdFromServiceName(serviceName));
        dispatch.call('runtime', '_registerExtensionPrimitives', extensionInfo, this.getIdFromServiceName(serviceName))
            .catch(e => {
                log.error(`Failed to register primitives for extension on service ${serviceName}:`, e);
            });
    }

    /**
     * Modify the provided text as necessary to ensure that it may be used as an attribute value in valid XML.
     * @param {string} text - the text to be sanitized
     * @returns {string} - the sanitized text
     * @private
     */
    _sanitizeID (text) {
        return text.toString().replace(/[<"&]/, '_');
    }

    /**
     * Apply minor cleanup and defaults for optional extension fields.
     * TODO: make the ID unique in cases where two copies of the same extension are loaded.
     * @param {string} serviceName - the name of the service hosting this extension block
     * @param {ExtensionInfo} extensionInfo - the extension info to be sanitized
     * @param {object} id - the id of oringal extensions or device.
     * @returns {ExtensionInfo} - a new extension info object with cleaned-up values
     * @private
     */
    _prepareExtensionInfo (serviceName, extensionInfo, id) {
        extensionInfo = Object.assign([], extensionInfo);
        extensionInfo.map(category => {
            if (!/^[a-z0-9]+$/i.test(category.id)) {
                throw new Error('Invalid category id');
            }
            if (id.deviceId) {
                category.id = `${this.runtime.getDevice().type}_${category.id}`;
            }
            category.name = category.name || category.id;
            category.blocks = category.blocks || [];
            category.targetTypes = category.targetTypes || [];
            category.blocks = category.blocks.reduce((results, blockInfo) => {
                try {
                    let result;
                    switch (blockInfo) {
                    case '---': // separator
                        result = '---';
                        break;
                    default: // an ExtensionBlockMetadata object
                        result = this._prepareBlockInfo(serviceName, blockInfo);
                        break;
                    }
                    results.push(result);
                } catch (e) {
                    // TODO: more meaningful error reporting
                    log.error(`Error processing block: ${e.message}, Block:\n${JSON.stringify(blockInfo)}`);
                }
                return results;
            }, []);
            category.menus = category.menus || {};
            category.menus = this._prepareMenuInfo(serviceName, category.menus);
            return category;
        });
        return extensionInfo;
    }

    /**
     * Prepare extension menus. e.g. setup binding for dynamic menu functions.
     * @param {string} serviceName - the name of the service hosting this extension block
     * @param {Array.<MenuInfo>} menus - the menu defined by the extension.
     * @returns {Array.<MenuInfo>} - a menuInfo object with all preprocessing done.
     * @private
     */
    _prepareMenuInfo (serviceName, menus) {
        const menuNames = Object.getOwnPropertyNames(menus);
        for (let i = 0; i < menuNames.length; i++) {
            const menuName = menuNames[i];
            let menuInfo = menus[menuName];

            // If the menu description is in short form (items only) then normalize it to general form: an object with
            // its items listed in an `items` property.
            if (!menuInfo.items) {
                menuInfo = {
                    items: menuInfo
                };
                menus[menuName] = menuInfo;
            }
            // If `items` is a string, it should be the name of a function in the extension object. Calling the
            // function should return an array of items to populate the menu when it is opened.
            if (typeof menuInfo.items === 'string') {
                const menuItemFunctionName = menuInfo.items;
                const serviceObject = dispatch.services[serviceName];
                // Bind the function here so we can pass a simple item generation function to Scratch Blocks later.
                menuInfo.items = this._getExtensionMenuItems.bind(this, serviceObject, menuItemFunctionName);
            }
        }
        return menus;
    }

    /**
     * Fetch the items for a particular extension menu, providing the target ID for context.
     * @param {object} extensionObject - the extension object providing the menu.
     * @param {string} menuItemFunctionName - the name of the menu function to call.
     * @returns {Array} menu items ready for scratch-blocks.
     * @private
     */
    _getExtensionMenuItems (extensionObject, menuItemFunctionName) {
        // Fetch the items appropriate for the target currently being edited. This assumes that menus only
        // collect items when opened by the user while editing a particular target.
        const editingTarget = this.runtime.getEditingTarget() || this.runtime.getTargetForStage();
        const editingTargetID = editingTarget ? editingTarget.id : null;
        const extensionMessageContext = this.runtime.makeMessageContextForTarget(editingTarget);

        // TODO: Fix this to use dispatch.call when extensions are running in workers.
        const menuFunc = extensionObject[menuItemFunctionName];
        const menuItems = menuFunc.call(extensionObject, editingTargetID).map(
            item => {
                item = maybeFormatMessage(item, extensionMessageContext);
                switch (typeof item) {
                case 'object':
                    return [
                        maybeFormatMessage(item.text, extensionMessageContext),
                        item.value
                    ];
                case 'string':
                    return [item, item];
                default:
                    return item;
                }
            });

        if (!menuItems || menuItems.length < 1) {
            throw new Error(`Extension menu returned no items: ${menuItemFunctionName}`);
        }
        return menuItems;
    }

    /**
     * Apply defaults for optional block fields.
     * @param {string} serviceName - the name of the service hosting this extension block
     * @param {ExtensionBlockMetadata} blockInfo - the block info from the extension
     * @returns {ExtensionBlockMetadata} - a new block info object which has values for all relevant optional fields.
     * @private
     */
    _prepareBlockInfo (serviceName, blockInfo) {
        blockInfo = Object.assign({}, {
            blockType: BlockType.COMMAND,
            terminal: false,
            blockAllThreads: false,
            arguments: {}
        }, blockInfo);
        blockInfo.opcode = blockInfo.opcode && this._sanitizeID(blockInfo.opcode);
        blockInfo.text = blockInfo.text || blockInfo.opcode;

        switch (blockInfo.blockType) {
        case BlockType.EVENT:
            if (blockInfo.func) {
                log.warn(`Ignoring function "${blockInfo.func}" for event block ${blockInfo.opcode}`);
            }
            break;
        case BlockType.BUTTON:
            if (blockInfo.opcode) {
                log.warn(`Ignoring opcode "${blockInfo.opcode}" for button with text: ${blockInfo.text}`);
            }
            break;
        default: {
            if (!blockInfo.opcode) {
                throw new Error('Missing opcode for block');
            }

            const funcName = blockInfo.func ? this._sanitizeID(blockInfo.func) : blockInfo.opcode;

            const getBlockInfo = blockInfo.isDynamic ?
                args => args && args.mutation && args.mutation.blockInfo :
                () => blockInfo;
            const callBlockFunc = (() => {
                if (dispatch._isRemoteService(serviceName)) {
                    return (args, util, realBlockInfo) =>
                        dispatch.call(serviceName, funcName, args, util, realBlockInfo);
                }

                // avoid promise latency if we can call direct
                const serviceObject = dispatch.services[serviceName];
                if (!serviceObject[funcName]) {
                    // The function might show up later as a dynamic property of the service object
                    log.warn(`Could not find extension block function called ${funcName}`);
                    return () => { };
                }
                return (args, util, realBlockInfo) =>
                    serviceObject[funcName](args, util, realBlockInfo);
            })();

            blockInfo.func = (args, util) => {
                const realBlockInfo = getBlockInfo(args);
                // TODO: filter args using the keys of realBlockInfo.arguments? maybe only if sandboxed?
                return callBlockFunc(args, util, realBlockInfo);
            };
            break;
        }
        }

        return blockInfo;
    }
}

module.exports = ExtensionManager;

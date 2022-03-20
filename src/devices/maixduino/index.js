const formatMessage = require('format-message');

const ArgumentType = require('../../extension-support/argument-type');
const BlockType = require('../../extension-support/block-type');
const MaixduinoPeripheral = require('../arduinoCommon/maixduino-peripheral');

/**
* The list of USB device filters.
* @readonly
*/
const PNPID_LIST = [
    // Maixduino
    'USB\\VID_0403&PID_6010'
];

/**
* Configuration of serialport
* @readonly
*/
const SERIAL_CONFIG = {
    baudRate: 115200,
    dataBits: 8,
    stopBits: 1,
    hupcl:false
};

/**
 * Configuration of flash.
 * @readonly
 */

const menuIconURI = "data:image/svg+xml;base64,PD94bWwgdmVyc2lvbj0iMS4wIiBlbmNvZGluZz0iVVRGLTgiIHN0YW5kYWxvbmU9Im5vIj8+CjwhRE9DVFlQRSBzdmcgUFVCTElDICItLy9XM0MvL0RURCBTVkcgMS4xLy9FTiIgImh0dHA6Ly93d3cudzMub3JnL0dyYXBoaWNzL1NWRy8xLjEvRFREL3N2ZzExLmR0ZCI+CjxzdmcgdmVyc2lvbj0iMS4xIiBpZD0iTGF5ZXJfMSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiB4bWxuczp4bGluaz0iaHR0cDovL3d3dy53My5vcmcvMTk5OS94bGluayIgeD0iMHB4IiB5PSIwcHgiIHdpZHRoPSIyMHB4IiBoZWlnaHQ9IjIwcHgiIHZpZXdCb3g9IjAgMCAyMCAyMCIgZW5hYmxlLWJhY2tncm91bmQ9Im5ldyAwIDAgMjAgMjAiIHhtbDpzcGFjZT0icHJlc2VydmUiPiAgPGltYWdlIGlkPSJpbWFnZTAiIHdpZHRoPSIyMCIgaGVpZ2h0PSIyMCIgeD0iMCIgeT0iMCIKICAgIGhyZWY9ImRhdGE6aW1hZ2UvcG5nO2Jhc2U2NCxpVkJPUncwS0dnb0FBQUFOU1VoRVVnQUFBQlFBQUFBVUNBTUFBQUM2ViswL0FBQUFCR2RCVFVFQUFMR1BDL3hoQlFBQUFDQmpTRkpOCkFBQjZKUUFBZ0lNQUFQbi9BQUNBNlFBQWRUQUFBT3BnQUFBNm1BQUFGMitTWDhWR0FBQUN1RkJNVkVYLy8vL3ZwcVhoWFYzbGNIRGsKYjNEbGNuTHBob1g5OVBIMHhjWDIwYy8yejg3MzB0RDQyZGp3cjYzNTR1RDg4L0gxenNya2FtelNEUlRURVJmU0JnM2FOanJmVDFMUQpBQVRVRXhuVERoVGhXMS8xeGNUNjNkNzU0dC8vL3YzcWpvdjB4c1B5dmJyeXVyanl1N254c3EvMjBNNzc3ZXJtZlhybmVYamtiVzNrCmJtN3JsSlAvLy83ODhmSGVUbEhTQ3hEVUVoblZHUi9WRngzVUZ4M1ZGeDdWR0I3VUZSdlREeFRhUFQvNTNOemdVbFhXSUNUWk1EWG4KZm4vNzZPWHFqby9URGhMWUxERGZVbFBXSVNQWEt5M1hLaXpZTEMzY1FrTFdJU2ZTRFJUNDJkblJEZy81NE4zWUtpL1VGUnZWRngzZgpWRmIxeXNyZVVGTFFEUS81M052V0lTWFVGaHpWRngzNTNkenp4TVBkUjBuaFhGN2hYbUhoWG1EaFhWL2laV2ZWR2lEVkl5WGdYVjdYCkp5dnZxcWZrYzNQUkJncnZyYXJ0b2FETkFBRFNDUS9VRmh6VER4WFdJeWpVR0IzVUZobnRucHpxbHBQVUd4L1RFeGZ5dTdicGpJclgKSmlyYU16ZlNDUkRVRlJ2VERoWE5BQURwaG9qc25KclNEQkRlVGxIbmdvRFZHaC9hTlRyV0pDWFVFeG5hTlRqYlBEL2FOamphTmpuYQpOenJhTnpuU0d4dmxkbmZlVGsvVEVoZmNSVWJWRlJ2Vkd4L2NSVWJWSENIZVRVM2FPVHJaTXpUWk5UYlhLeXpnV1ZuZFNVdlNEQkxjClBVTFdIaVQ0M2QzWU1UUFNEUkxVRkJyVUV4clVGQnJVRmh6VkpDWDB3c1BSQXdySUFBRFFBQVhRQVFmU0RSSFZGUnpSQUFmUUFBVFMKREJMU0NRN1JBd25URGhYVUZSdlVFeG5LQUFEU0J3N1BBZ1ROQUFEVERoVFVGQnJWRkJ2U0NRL1FCUWpPQUFIU0JnN1FCQWZTQlF6UQpBQWZMQUFEUUJBalREUlRVRWhqUkJBblBBQURPQUFEVUVSalVGeHpSQnd2VUZodlRDeExVRmh6VkZ4N1FBQWJSQkF6VUZ4M1ZGeDNWCkZoelBBQUhSQWduUkJBdlNDaEhUQ2hIUkFRblFBQVBUREJQTUFBRFVFeHJUQ3hIVER4YlNDeEhURHhYVEVCYlRFQlhSQmd6Ly8vL0EKT0REWkFBQUFwM1JTVGxNQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBUApzL2I1OHZMeTlmWHk4c0ludXZ6YmhnWmkvdVJYVVZOVFVsUG8vZ3J1R05mejg3STN0L0FPMS9MeEdndUhob2FHaG9QdzExalJEVmZ6CkhBWjllbnA1ZnZMcEZTTHQ4eDBONlY5YlhGeGdGeUxzbGhmT21xNzl4YjIvdjcrL3dGT3Y5NjMwOTY3NVJ6VTJOalU0cnZ6UTdTYkwKOHZuNyt2UFdSSlhqTmxjQUFBQUJZa3RIUkFDSUJSMUlBQUFBQ1hCSVdYTUFBQVhRQUFBRjBBRXRrQ0xZQUFBQUIzUkpUVVVINVFzVwpCd0V2UDNvRFhnQUFCeko2VkZoMFVtRjNJSEJ5YjJacGJHVWdkSGx3WlNCNGJYQUFBSGljN1YzYnJxdTZEbjMzVjZ4UGdEZ1grQnhtCm9XOWJPby9uOC9kd01pbVVhMmlYdG95VVZtb3BUV0lQMjhNT3VGTHAvLy84ai83OCtWTTc0enp4ZzUraENaV3ZQZnNmNzRJMWxUZmUKK2VCYlAzQnZ6UEQ4K2ZsNUdvUHpyYmR5eGdWMnR1Zks5cUd5akxHTmI4azJvUXVZNkRoMGRuRFc0eDBMTW1PU01memtvZXI0RVJydQpRdU14MGZjaXpOZW1rcy8rNFlmQThoMkpCR2hqL1ZQMDRDNTk4Um9lTlptV3dia2ZtV0ZmTTB6bEd0dTdpb3dvOXd6eEZEc3pzRGM5CjlLbVpHVXR3NEJibmFuYmNzdUdHclhuZ3JPRUs1N3g1NHQzaXRlYVdUQjhuZGIrdmxpdXpmQnFCbDQ0QjBrQVh6NTB6MWxvL0FhU0kKTUgwcElKdGc4YXk0QTZobmlBOHpCQXd5UTlRN0pQbnlqUG9ZdkJxODlwUVdNWDNnQUMrSlhVSURjTENVZkQvcHN0UUk2c0I1Y0kzeApyZGlPNE5Lbkg2RGk3d0FZM2tBMlRDMGF3a3pkQ3VydmMrRTE4Y1VhVk5Sb21MeUpZN2pFOTREYlFPZEtBTUlKbFhnMWlhSXpXVmlLCk42eFhRV2ZqSDVQbFNBWkNBNGVKVHFKSDdBUnRrc1I2a25nbWtCQ3JEVllGQkpoT2pxNEJmYzJtY2ZvYTlibUJ4YjYyQitzZ21qNDEKTWhiczUwSTNqUDBtbFlPMUx2Z1VERWRDNlVpcVpJdUFjTU9ZcHpNeEFKL1JXMGdkc254eWpRUnUxVzI2ZjZXWFJkajZMUkpOV3RISwpGcitUcmhxZkVISU9IOXRqZVV0eDQ1eEpISzJKK2FKbmRDL29LY1Iwc0o3RW1uR3k2Tk0vZUhoTGJoVXR3MUVTbmVWWDhwSlVoNEdtCmNUR0h5MVJrRmpZdFA2SnpYcUpvUzFhMmhzN2E5TTREUmNrR1NDVlR5aXU4NmF3Y0kxKzZtRFZSRjh3UFJpR2o0M08xMUNVdFN2TlYKVHpUcG9XLzBhV2luT2FNaXROTEUvTXJ0Sk5udmFySmFsTEkwaWFYSkp5M3F0VGFpREgya3pjYkNCeHJGM05naHlDVUg5aktZQjREQgpFY3E1aTdsbzhFRkV5RWhhRHQyQnVVNS9uUlJXMEFOenhXS1NhZ1V0UitWUGgvdGQ3OUpvMUpqcE9pdTFqYU1GaHBnczRCTWUwZ2dqCndHUS9JaGhxeHA2RmF6QTBSQ3RqV3lOMFJVbU13U2VGb0VlRmR6am5VQ1lkam1FUGZHWXZyZ2g0dHp5anh5U2UxdktQZ3VGM29wQ0oKdzZ2cXlEemk0VzFURTRja0FGc1d3L2tnR1E1alpYeUEvdGhKb0hLUGtZMlZCVlFyUU9VSUV4cVdmUk0yWEpoYTR4UE9BOXdTMUVzMwptcFRMQWlXQjBxWnRJY1E4ZGhaNmVjZitlcWVHTGc0UVJHTXI1bWRPWjZDemwwMldISEdUQWhKYkdweUMraGdzNkVHSWVBUmdLeUM3CituaEhsNEFoVDBjdlk4SFF2cytnMWNSeGF3ZlBoSHA3K2EwWndyV0hiNmVObHV5NElRL0xqTVV4N3F3WDVYRmRNbllMSkNSTjVWck0KenJIaW9VUWg4bVpaRzA0UVN6THREVDZEdEJSSGNZazhxaDRHQTcxSHcya3d6T0craWFjanFwNVpMZ1J3MElraDJOQkhWVzFqTWZwUQpGOG1iT01JeXRYalExaVNKWEtKN3VTbTR1aWVndDAzQm9jeEo1R2JPZmxmZlBGK1hFb2ozZklKZ24vMTNDTEpMa1NPQ1lNazZia1R0CjNCejBQVGxTY0ZJT09lQ25LaFVqWkhDUVZjaUI3RjVISHNBbzRJWkY4bzhjYXJ5SjZiT1JBR1E1VThkS0liWE9IcE1qaWY4MElHZmsKT05wb2djaHhhMVBsa0NPcHNwdlk4c21SQXBXK0owZmlCbjFQamsyS3ZQbEdOQksvaEJoOEF5NFhsdDZialNDZmtkUnlPTGhiMTY1eQprUFpJZUpXRHRFZkNxeHlrSFZtWDk1STBiaVkvM2t1ZUY4aHJWcU05RWw2ODJKblgvbmNTWHZVZ0hSSGlDZ2ZwNUtvMW00TVphU1NQCmczUTA1QW9INmF3UXpnY2Y3U3RwdFFVKzJMbk9vSTUzUzhZdGM5b2V4OTNnTDZNNGJhaFdBNWZqTml2dFhJSnJ4dVRnSDNJVDhTV2cKWlIvclc4WHh4dEI0UElxbDQrbG53Rnc3enFOcmNwZlRwOW1VSlhkRFd3ZXJ3UzAyc2pQR1VlYnduVXY2OGQ1VFI5TlZOSXArdW1FdgpWOTNJUXpFenVYaGQveHErUFhxejBzNmxnS2h0a0l6d2NJMjE4ZWFCaEFHL0xHcVM5V1FHNVUwNUF5WjN0TVlGMmpPWks1RnZNMmhiCjVweGh5MXZneXhscExNM3Y0S2ZXaFhCdjJidzR6d1JaK1NqbitWOHNsRmNDYmdMdEdoamQwRExyMmoyZ1pWNnYzUU5hNW1iMEh0Q3UKVUVZOXRNLzVyeERhTjhsTUc3VE1HNXIzZ0hia21OdEJ5NCt5RzBEN2xESXFvZjBkL3V1QWR0Qmd1Uiswekx0Kzk0Q1dHMlczZ1BZWgpaWlJDK3h2ODF3SnR3cUpGb3k4VzJuUE1MYUhsUmRsTm9IMUNHYlhRdnVlL0htZ0h2ejY4SDdSdHg5d1VXazZVM1FiYWRjb29odll0Ci96VkJPL2loOS8yZ25mNGs1azdRenFQc1J0Q3VVa1kxdE8vNHJ3dGE1cTk5N2dIdDlMYzFkNEpXbXBsYW9aVm1wbEpvcFptcEZWcHAKWm1xRlZwcVpTcUdWWnFaV2FLV1pxUlJhYVdacWhWYWFtVnFobFdhbVVtaWxtYWtWV21sbUtvVldtcGxhb1pWbXBsWm9wWm1wRkZwcApabXFGVnBxWlNxR1ZacVpXYUtXWnFSVmFhV1lxaFZhYW1WcWhsV2FtVm1pbG1ha1VXbWxtYW9WV21wbEtvWlZtcGxab3BabXBGVnBwClppcUZWcHFaV3FHVlpxWlNhS1dacVJWYWFXWnFoVmFhbVVxaGxXYW1WbWlsbWFrVVdtbG1hb1ZXbXBsYW9aVm1wbEpvcFptcEZWcHAKWm1xRlZwcVpTcUdWWnVZNGRmTVBpK1ZQRk9LZmxJYVEvbXVZL2dWcXMrUXlDaDJoVWdBQUFBRnZjazVVQWMraWQ1b0FBQUdHU1VSQgpWQmpUWTlEVjAxOXVZR2hrYkdKaWFtcHFZbUprWm01aHlXQzFZcVgxcXRWcjFxNWJ2d0VJTm03YXZHV3JEY08yN2JaMjlnNDdISjJjClhWeGMzZHc5ZHU3YXZZZkJjNjhYQThNK2J4OUdKbVptRmxZMjMvMjdkdmt4K0I4SUNBd0tEbUhuNE9UaTR1YmhEVDI0YTBzWVEvanUKUTRlUFJFUkdSY2ZFeHNiRnh5WWNCUWthSFR1ZXlNZWZkUGdFQ0p3OGNlbzBSUEJNY29wQTZzb3R1ODd1UEhkK0Z4Q0FCUzljVEV2UAp5TXpLenNuTnUzUVpKcmpyeXJyOEFrRWhZUkhSd3FLVlYyR0N1NjVjS3k0UkU1ZVFsQ28xdjM0WkpyanJ5c1d5OG9yS3F1cWEycm9iCmwyR0N1NjVzdTNsK3k2NlQ5ZElOamJjZ2dwdDMzYjZ6OGlJUUhHNlNrVzIrQVJFODF0TGExdDdSMGRuVjNTUFhlMjB6UlBCdW43eUMKb3BLU3NvcXFXdis5K3lEQkNWdDIzWjJvcnFHcHBjV2tyVFBwOFAzTnU3YjRNVHc0dGV2aDVDbFRwMDJmUG1QbXJKTkFzVjFyWmpQTQoyYnA3MTdGSGo1ODhmYnJsMlFtZ3k3ZGN2VGVYWWQ3OEJXWmhDeGN0WHJ4NDBlSWxZV0ZoWnVaTGx3RUFLWHZPRldRT25tMEFBQUFsCmRFVllkR1JoZEdVNlkzSmxZWFJsQURJd01qRXRNVEV0TWpKVU1EYzZNREU2TkRjck1EQTZNREFwOFZHYUFBQUFKWFJGV0hSa1lYUmwKT20xdlpHbG1lUUF5TURJeExURXhMVEl5VkRBM09qQXhPalEzS3pBd09qQXdXS3pwSmdBQUFCTjBSVmgwWkdNNlptOXliV0YwQUdsdApZV2RsTDNCdVovKzVHejRBQUFBVmRFVllkSEJvYjNSdmMyaHZjRHBEYjJ4dmNrMXZaR1VBTTFZQ3MwQUFBQUFkZEVWWWRIUnBabVk2CldGSmxjMjlzZFhScGIyNEFPVFl3TURBd0x6RXdNREF3ZElmYzNBQUFBQjEwUlZoMGRHbG1aanBaVW1WemIyeDFkR2x2YmdBNU5qQXcKTURBdk1UQXdNREQxb3JuN0FBQUFGSFJGV0hSNGJYQTZRMjlzYjNKVGNHRmpaUUEyTlRVek5UdFVUZklBQUFBb2RFVllkSGh0Y0RwRApjbVZoZEdWRVlYUmxBREl3TWpFdE1URXRNakpVTVRFNk5UTTZOVGNyTURVNk16QjQyWHAwQUFBQU0zUkZXSFI0YlhBNlEzSmxZWFJ2CmNsUnZiMndBUVdSdlltVWdVR2h2ZEc5emFHOXdJRU5ESURJd01UVXVOU0FvVjJsdVpHOTNjeW52Smk5UEFBQUFLblJGV0hSNGJYQTYKVFdWMFlXUmhkR0ZFWVhSbEFESXdNakV0TVRFdE1qSlVNVEk2TXpBNk1qZ3JNRFU2TXpDeVJuR05BQUFBS0hSRldIUjRiWEE2VFc5awphV1o1UkdGMFpRQXlNREl4TFRFeExUSXlWREV5T2pNd09qSTRLekExT2pNd2p1SWlNd0FBQUJaMFJWaDBlRzF3T2xCcGVHVnNXRVJwCmJXVnVjMmx2YmdBeU1Lb2d4MjRBQUFBV2RFVllkSGh0Y0RwUWFYaGxiRmxFYVcxbGJuTnBiMjRBTWpCM3RoN3JBQUFBUzNSRldIUjQKYlhCTlRUcEViMk4xYldWdWRFbEVBR0ZrYjJKbE9tUnZZMmxrT25Cb2IzUnZjMmh2Y0Rwa04yWmpaVEpsTnkwMFlqWXhMVEV4WldNdApPVGxtWXkxa09ESmhZemszWVRnMll6SnhTRnliQUFBQVBYUkZXSFI0YlhCTlRUcEpibk4wWVc1alpVbEVBSGh0Y0M1cGFXUTZabUV3CllqRm1OekV0TmpsbU9DMDFNelJqTFRnMVpqVXRaVGczWlRWallUSTNZMlE1eXpESjVRQUFBRVYwUlZoMGVHMXdUVTA2VDNKcFoybHUKWVd4RWIyTjFiV1Z1ZEVsRUFIaHRjQzVrYVdRNllXRTFaVEZsWkRVdE0yRTFaUzFtTURSbExUaGtZell0TURkaFpXWmpZekJsWm1KaApuTDYwVUFBQUFBQkpSVTVFcmtKZ2dnPT0iIC8+Cjwvc3ZnPgo=";

const DIVECE_OPT = {
    type: 'maixduino'
};

const LedState = {
    On: 'on',
    Off: 'off'
};

const Key = {
    A: 'a',
    B: 'b'
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
    P13: '13'
};

const analogPins = {
    A0: 'A0',
    A1: 'A1',
    A2: 'A2',
    A3: 'A3',
    A4: 'A4',
    A5: 'A5'
};

const timerPins = {
	T0: '0',
	T1: '1',
	T2: '2',
	T3: '3',
	T4: '4',
	T5: '5',
	T6: '6',
	T7: '7',
	T8: '8',
	T9: '9',
	T10: '10',
    T11: '11'
};


const Angle = {
	a0: '0',
	a1: '1',
	a2: '2',
	a3: '3',
};


const Buttons = {
	A: 'a',
	B: 'b',
	AB: 'ab'
};

const Level = {
    High: '1',
    Low: '0'
};

const Invert = {
	on: '1',
	off: '0'
};

const dim ={
	height: 'height',
	width: 'width'
};

const camColor = {
	colour: 'RGB565',
	gray: 'GRAYSCALE'
};

const camSize = {
	normal: 'QVGA',
	medium: 'QQVGA',
	small: 'QQQVGA',
	large: 'VGA'
};

const eyes = {
	left: '0',
	right: '1'
};

const Wait = {
	WAIT: 'True',
	BACKGROUND: 'False'
};

const collection = {
	AUTOMATE: 'enable',
	START: 'collect'
};

/**
 * Manage communication with a Maixduino  peripheral over a OpenBlock Link client socket.
 */
class Maixduino extends MaixduinoPeripheral{
    /**
     * Construct a Maixduino communication object.
     * @param {Runtime} runtime - the OpenBlock runtime
     * @param {string} deviceId - the id of the deivce
     * @param {string} originalDeviceId - the original id of the peripheral, like xxx_arduinoUno
     */
    constructor (runtime, deviceId, originalDeviceId) {
        super(runtime, deviceId, originalDeviceId, PNPID_LIST, SERIAL_CONFIG, DIVECE_OPT);
    }
}

/**
 * OpenBlock blocks to interact with a Maixduino  peripheral.
 */
class OpenBlockMaixduinoDevice {
    /**
     * @return {string} - the ID of this device.
     */
    static get DEVICE_ID () {
        return 'maixduino';
    }

    get LEDSTATE_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'maixduino.ledState.on',
                    default: 'on',
                    description: 'label for led state on'
                }),
                value: LedState.On
            },
            {
                text: formatMessage({
                    id: 'maixduino.ledState.off',
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
            }
        ];
    }

    get LEVEL_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'maixduino.levelMenu.high',
                    default: 'high',
                    description: 'label for high level'
                }),
                value: Level.High
            },
            {
                text: formatMessage({
                    id: 'maixduino.levelMenu.low',
                    default: 'low',
                    description: 'label for low level'
                }),
                value: Level.Low
            }
        ];
    }

	get INVERT_MENU () {
		return [
			{
				text: 'ON',
				value: Invert.on
			},
			{
				text: 'OFF',
				value: Invert.off
			}
		];
	}


    get ANALOG_PINS_MENU () {
        return [
            {
                text: 'A0',
                value: analogPins.A40
            },
            {
                text: 'A1',
                value: analogPins.A41
            },
            {
                text: 'A2',
                value: analogPins.A42
            },
            {
                text: 'A3',
                value: analogPins.A43
            },
            {
                text: 'A4',
                value: analogPins.A4
            },
            {
                text: 'A5',
                value: analogPins.A5
            }
        ];
    }

	get TIMER_PINS_MENU () {
		return [
		   {
                text: 'T0',
                value: timerPins.T0
            },
			{
                text: 'T1',
                value: timerPins.T1
            },
			{
                text: 'T2',
                value: timerPins.T2
            },
			{
                text: 'T3',
                value: timerPins.T3
            },
			{
                text: 'T4',
                value: timerPins.T4
            },
			{
                text: 'T5',
                value: timerPins.T5
            },
			{
                text: 'T6',
                value: timerPins.T6
            },
			{
                text: 'T7',
                value: timerPins.T7
            },
			{
                text: 'T8',
                value: timerPins.T8
            },
			{
                text: 'T9',
                value: timerPins.T9
            },
			{
                text: 'T10',
                value: timerPins.T10
            },
			{
                text: 'T11',
                value: timerPins.T11
            }
		];
	}

	get ANGLE_MENU () {
		return [
			{
				text: '0°',
				value: Angle.a0
			},
			{
				text: '90',
				value: Angle.a1
			},
			{
				text: '180°',
				value: Angle.a2
			},
			{
				text: '270°',
				value: Angle.a3
			}
		];
	}

	get DIMENSION_MENU () {
		return [
			{
				text: 'height',
				value: dim.height
			},
			{
				text: 'width',
				value: dim.width
			}
		];
	}

	get CAMERA_MENU () {
		return [
			{
				text: 'monocular',
				value: Camera.monocular
			},
			{
				text: 'binacular',
				value: Camera.binacular
			}
		];
	}

	get CAMCOLOR_MENU () {
		return [
			{
				text: 'colour',
				value: camColor.colour
			},
			{
				text: 'gray',
				value: camColor.gray
			}
		];
	}

	get CAMSIZE_MENU () {
		return [
			{
				text: 'QVGA',
				value: camColor.normal
			},
			{
				text: 'QQVGA',
				value: camColor.medium
			},
			{
				text: 'QQQVGA',
				value: camColor.small
			},
			{
				text: 'VGA',
				value: camColor.large
			}
		];
	}

	get EYE_MENU () {
		return [
			{
				text: '0',
				value: eyes.left
			},
			{
				text: '1',
				value: eyes.right
			}
		];
	}

	get COLORMODE_MENU () {
		return [
			{
				text: 'contrast',
				value: colorMode.CONTRAST
			},
			{
				text: 'brightness',
				value: colorMode.BRIGHTNESS
			},
			{
				text: 'gain value',
				value: colorMode.GAINVALUE
			},
			{
				text: 'saturation',
				value: colorMode.SATURATION
			}
		];
	}

	get CAMDETAILS_MENU () {
		return [
			{
				text: 'resolution width',
				value: CAMDetails.WIDTH
			},
			{
				text: 'resolution height',
				value: CAMDetails.HEIGHT
			},
			{
				text: 'gain value',
				value: CAMDetails.GAINVALUE
			},
			{
				text: 'frame image cache',
				value: CAMDetails.FRAMECACHE
			},
			{
				text: 'hardware ID',
				value: CAMDetails.ID
			}
		];
	}

	get WAIT_MENU () {
		return [
			{
				text: 'wait',
				value: Wait.WAIT
			},
			{
				text: 'background',
				value: Wait.BACKGROUND
			}
		];
	}

	get TIME_MENU () {
		return [
			{
				text: 'seconds',
				value: time.SEC
			},
			{
				text: 'milliseconds',
				value: time.MSEC
			},
			{
				text: 'microseconds',
				value: time.USEC
			}
		];
	}

	get COLLECTION_MENU () {
		return [
			{
				text: 'Automatic',
				value: collection.AUTOMATE
			},
			{
				text: 'Start',
				value: collection.START
			}
		];
	}




    /**
     * Construct a set of Maixduino blocks.
     * @param {Runtime} runtime - the OpenBlock runtime.
     */
    constructor (runtime) {
        /**
         * The OpenBlock runtime.
         * @type {Runtime}
         */
        this.runtime = runtime;

        // Create a new Maixduino peripheral instance
        this._peripheral = new Maixduino(this.runtime, OpenBlockMaixduinoDevice.DEVICE_ID);
    }



    /**
     * @returns {Array.<object>} metadata for this extension and its blocks.
     */
    getInfo () {
        return [
		{
			id: 'Maixduino',
			name: formatMessage({
				id: 'maixduino.category.maixduino',
				default: 'Maixduino',
				description: 'The name of the maixduino device'
			}),
			menuIconURI: menuIconURI,
              color1: '#FFBF00',
              color2: '#E6AC00',
              color3: '#CC9900',
			blocks: [
				{
					opcode: 'whenmaixduinobegin',
					text: formatMessage({
						id: 'maixduino.maixduino.header',
						default: 'when Maixduino begins',
						description: 'maixduino header'
					}),
					blockType: BlockType.HAT
				}
				],
				menus: {}
		},
		{
            id: 'pin',
            name: formatMessage({
                id: 'maixduino.category.pins',
                default: 'Pins',
                description: 'The name of the maixduino device pin category'
            }),
            color1: '#4C97FF',
            color2: '#3373CC',
            color3: '#3373CC',

            blocks: [
                {
                    opcode: 'setDigitalOutput',
                    text: formatMessage({
                        id: 'maixduino.pins.setDigitalOutput',
                        default: 'set digital pin [PIN] out [LEVEL]',
                        description: 'maixduino set digital pin out'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        PIN: {
                            type: ArgumentType.STRING,
                            menu: 'pins',
                            defaultValue: Pins.P2
                        },
                        LEVEL: {
                            type: ArgumentType.STRING,
                            menu: 'level',
                            defaultValue: Level.High
                        }
                    }
                },
                {
                    opcode: 'initializePwm',
                    text: formatMessage({
                        id: 'maixduino.pins.initializePwm',
                        default: 'initialize pwm pin [PIN] Freq [FREQ] hz timer [TIMER]',
                        description: 'maixduino initialize Pwm pin'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        PIN: {
                            type: ArgumentType.STRING,
                            menu: 'pins',
                            defaultValue: Pins.P2
                        },
                        FREQ: {
                            type: ArgumentType.NUMBER,
                            defaultValue: '500000'
                        },
						TIMER: {
                            type: ArgumentType.STRING,
                            menu: 'timerPins',
                            defaultValue: timerPins.T0
                        }
                    }
                },
				{
                    opcode: 'setPwmOutput',
                    text: formatMessage({
                        id: 'maixduino.pins.setPwmOutput',
                        default: 'set pwm pin [PIN] output duty cycle [OUT]',
                        description: 'maixduino set pwm pin out'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        PIN: {
                            type: ArgumentType.STRING,
                            menu: 'pins',
                            defaultValue: Pins.P2
                        },
                        OUT: {
                            type: ArgumentType.UINT8_NUMBER,
                            defaultValue: '50'
                        }
                    }
                },
                {
                    opcode: 'readDigitalPin',
                    text: formatMessage({
                        id: 'maixduino.pins.readDigitalPin',
                        default: 'read digital pin [PIN]',
                        description: 'maixduino read digital pin'
                    }),
                    blockType: BlockType.BOOLEAN,
                    arguments: {
                        PIN: {
                            type: ArgumentType.STRING,
                            menu: 'pins',
                            defaultValue: Pins.P2
                        }
                    }
                },
                {
                    opcode: 'readAnalogPin',
                    text: formatMessage({
                        id: 'maixduino.pins.readAnalogPin',
                        default: 'read analog pin [PIN]',
                        description: 'maixduino read analog pin'
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
                analogPins: {
                    items: this.ANALOG_PINS_MENU
                },
                timerPins: {
                    items: this.TIMER_PINS_MENU
                }
            }
        },
        {
            id: 'display',
            name: formatMessage({
                id: 'maixduino.category.display',
                default: 'Display',
                description: 'The name of the maixduino device display category'
            }),
            color1: '#9966FF',
            color2: '#774DCB',
            color3: '#774DCB',
            blocks: [
				{
                    opcode: 'initializeDisplay',
                    text: formatMessage({
                        id: 'maixduino.display.initializeDisplay',
					default: 'Display initialize freq [FREQ] Hz and Background [COLOR] invert [INVERT] ',
                        description: 'maixduino initialize Display'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        FREQ: {
                            type: ArgumentType.NUMBER,
                            defaultValue: '15000000'
                        },
						COLOR: {
                            type: ArgumentType.COLOR
                        },
						INVERT: {
                            type: ArgumentType.STRING,
                            menu: 'Invert',
                            defaultValue: Invert.off
                        }
                    }
                },
				{
                    opcode: 'displayWords',
                    text: formatMessage({
                        id: 'maixduino.display.displayWords',
					default: 'Display Text[TEXT] x(0-320):[X_VALUE] y(0-240):[Y_VALUE] in [T_COLOR] background [COLOR] ',
                        description: 'maixduino Display words'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        TEXT: {
                            type: ArgumentType.STRING,
                            defaultValue: 'Hello OpenBlock'
                        },
						X_VALUE: {
							type: ArgumentType.NUMBER,
							defaultValue: 0
						},
						Y_VALUE: {
							type: ArgumentType.NUMBER,
							defaultValue: 0
						},
						T_COLOR: {
                            type: ArgumentType.COLOR
                        },
						COLOR: {
                            type: ArgumentType.COLOR
                        }
                    }
                },
				{
                    opcode: 'displayImagePath',
                    text: formatMessage({
                        id: 'maixduino.display.displayImagePath',
					default: 'Display image path [PATH]',
                        description: 'maixduino Display image path'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        PATH: {
                            type: ArgumentType.STRING,
                            defaultValue: '/sd/pic.bmp'
                        }
                    }
                },
				{
                    opcode: 'displayImage',
                    text: formatMessage({
                        id: 'maixduino.display.displayImage',
					default: 'Display image [IMAGE]',
                        description: 'maixduino Display image'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        IMAGE: {
                            type: ArgumentType.STRING,
                            defaultValue: 'image.image()'
                        }
                    }
                },
				{
                    opcode: 'displayClear',
                    text: formatMessage({
                        id: 'maixduino.display.displayClear',
					default: 'Clear display with [COLOR] background',
                        description: 'maixduino Display clear'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        COLOR: {
                            type: ArgumentType.COLOR
                        }
                    }
                },
				{
                    opcode: 'displayRotate',
                    text: formatMessage({
                        id: 'maixduino.display.displayRotate',
					default: 'Display rotate [ANGLE]',
                        description: 'maixduino Display rotate'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        ANGLE: {
                            type: ArgumentType.STRING,
                            menu: 'Angle',
                            defaultValue: Angle.r0
                        }
                    }
                },
				{
                    opcode: 'displayMirror',
                    text: formatMessage({
                        id: 'maixduino.display.displayMirror',
					default: 'Screen mirror is [VALUE]',
                        description: 'maixduino Display mirror'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        VALUE: {
                            type: ArgumentType.STRING,
                            menu: 'Invert',
                            defaultValue: Invert.on
                        }
                    }
                },
                {
                    opcode: 'displayResolution',
                    text: formatMessage({
                        id: 'maixduino.pins.displayResolution',
                        default: 'Display resolution [DIM]',
                        description: 'maixduino display resolution'
                    }),
                    blockType: BlockType.REPORTER,
                    arguments: {
                        DIM: {
                            type: ArgumentType.STRING,
                            menu: 'dimension',
                            defaultValue: dim.height
                        }
                    }
                },
                {
                    opcode: 'displayRGB888',
                    text: formatMessage({
                        id: 'maixduino.pins.displayRGB888',
                        default: 'RGB8888 red[RED] green[GREEN] blue[BLUE]',
                        description: 'maixduino display RGB888'
                    }),
                    blockType: BlockType.REPORTER,
                    arguments: {
                        RED: {
                            type: ArgumentType.UINT8_NUMBER,
                            defaultValue: 255
                        },
						GREEN: {
                            type: ArgumentType.UINT8_NUMBER,
                            defaultValue: 255
                        },
						BLUE: {
                            type: ArgumentType.UINT8_NUMBER,
                            defaultValue: 255
                        }
                    }
                },
                {
                    opcode: 'displayColor',
                    text: formatMessage({
                        id: 'maixduino.pins.displayColor',
                        default: 'Colour [COLOR]',
                        description: 'maixduino display color'
                    }),
                    blockType: BlockType.REPORTER,
                    arguments: {
                        COLOR: {
                            type: ArgumentType.COLOR
                        }
                    }
                }
            ],
            menus: {
                Invert: {
                    items: this.INVERT_MENU
                },
				Angle: {
					items: this.ANGLE_MENU
				},
				dimension: {
					items: this.DIMENSION_MENU
				}
            }
        },
		{
            id: 'camera',
            name: formatMessage({
                id: 'maixduino.category.camera',
                default: 'Camera',
                description: 'The name of the maixduino device camera category'
            }),
            color1: '#666699',
            color2: '#5c5c8a',
            color3: '#52527a',

            blocks: [
                {
                    opcode: 'cameraInit',
                    text: formatMessage({
                        id: 'maixduino.camera.cameraInit',
                        default: 'Camera initialize: [CAMERA] frame format: [CAMCOLOR] size: [CAMSIZE]',
                        description: 'maixduino camera initialize'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        CAMERA: {
                            type: ArgumentType.STRING,
							menu: 'Camera',
                            defaultValue: Camera.monocular
                        },
						CAMCOLOR: {
                            type: ArgumentType.STRING,
							menu: 'camColor',
                            defaultValue: camColor.colour
                        },
						CAMSIZE: {
                            type: ArgumentType.STRING,
							menu: 'camSize',
                            defaultValue: camSize.normal
                        }
					}
				},
				{
                    opcode: 'cameraInitSkipScreenshot',
                    text: formatMessage({
                        id: 'maixduino.camera.cameraInitSkipScreenshot',
                        default: 'Camera initialize: skip frame: [SKIP] Screenshot [VALUE]',
                        description: 'maixduino camera initialize skip screenshot'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
						SKIP: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 10
                        },
						VALUE: {
                            type: ArgumentType.STRING,
							menu: 'Invert',
                            defaultValue: Invert.on
                        }
					}
				},
				{
                    opcode: 'getImage',
                    text: formatMessage({
                        id: 'maixduino.camera.getImage',
                        default: 'Camera get image',
                        description: 'maixduino camera get image'
                    }),
                    blockType: BlockType.REPORTER,
                    disableMonitor: true,
                    arguments: {}
                },
				{
				opcode: 'chooseCameraEye',
                    text: formatMessage({
                        id: 'maixduino.camera.chooseCameraEye',
                        default: 'Double eye camera choose eye [EYE]',
                        description: 'maixduino camera choose camera eye'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        EYE: {
                            type: ArgumentType.STRING,
							menu: 'eye',
                            defaultValue: eyes.left
                        }
					}
				},
				{
				opcode: 'cameraMirrorHorizontal',
                    text: formatMessage({
                        id: 'maixduino.camera.mirrorHorizontal',
                        default: 'Camera Mirror-Horizontal is  [VALUE]',
                        description: 'maixduino camera mirror horizontal'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        VALUE: {
                            type: ArgumentType.STRING,
							menu: 'Invert',
                            defaultValue: Invert.on
                        }
					}
				},
				{
				opcode: 'cameraMirrorVertical',
                    text: formatMessage({
                        id: 'maixduino.camera.mirrorVertical',
                        default: 'Camera Mirror-Vertical is [VALUE]',
                        description: 'maixduino camera mirror vertical'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        VALUE: {
                            type: ArgumentType.STRING,
							menu: 'Invert',
                            defaultValue: Invert.on
                        }
					}
				},
				{
				opcode: 'cameraColorfulStrips',
                    text: formatMessage({
                        id: 'maixduino.camera.colorfulStrips',
                        default: 'Camera colourful strips is [VALUE]',
                        description: 'maixduino camera colorful strips'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        VALUE: {
                            type: ArgumentType.STRING,
							menu: 'Invert',
                            defaultValue: Invert.on
                        }
					}
				},
				{
				opcode: 'cameraSetColorMode',
                    text: formatMessage({
                        id: 'maixduino.camera.setColorMode',
                        default: 'Camera set [COLORMODE] as [VALUE]',
                        description: 'maixduino camera set color mode'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
						COLORMODE: {
							type: ArgumentType.STRING,
							menu: 'colormodes',
							defaultValue: colorMode.SATURATION
						},
                        VALUE: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 0
                        }
					}
				},
				{
				opcode: 'cameraSetWindow',
                    text: formatMessage({
                        id: 'maixduino.camera.setWindow',
                        default: 'Camera set window width  [WIDTH] height [HEIGHT]',
                        description: 'maixduino camera set window'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        WIDTH: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 320
                        },
                        HEIGHT: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 240
                        }
					}
				},
				{
				opcode: 'cameraGetDetails',
                    text: formatMessage({
                        id: 'maixduino.camera.getDetails',
                        default: 'Camera get [DETAILS]',
                        description: 'maixduino camera get details'
                    }),
                    blockType: BlockType.REPORTER,
                    arguments: {
                        DETAILS: {
                            type: ArgumentType.STRING,
							menu: 'camDetails',
                            defaultValue: CAMDetails.ID
                        }
					}
				}
            ],
            menus: {
				Camera: {
					items: this.CAMERA_MENU
				},
				camColor: {
					items: this.CAMCOLOR_MENU
				},
				camSize: {
					items: this.CAMSIZE_MENU
				},
				Invert: {
					items: this.INVERT_MENU
				},
				eye: {
					items: this.EYE_MENU
				},
				colormodes: {
					items: this.COLORMODE_MENU
				},
				camDetails: {
					items: this.CAMDETAILS_MENU
				}
			}
        },
		{
            id: 'audio',
            name: formatMessage({
                id: 'maixduino.category.audio',
                default: 'Audio',
                description: 'The name of the maixduino device audio category'
            }),
            color1: '#00BFFF',
            color2: '#00ACE6',
            color3: '#0099CC',

            blocks: [
                {
                    opcode: 'audioPlay',
                    text: formatMessage({
                        id: 'maixduino.audio.audioPlay',
					default: 'Audio play [LOC] [WAIT]',
                        description: 'maixduino audio play'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        LOC: {
                            type: ArgumentType.STRING,
                            defaultValue: '/sd/MaixDuino.wav'
                        },
						WAIT: {
							type:ArgumentType.STRING,
							menu: 'wait',
						defaultValue: Wait.WAIT
						}
                    }
                },
				{
                    opcode: 'audioVolume',
                    text: formatMessage({
                        id: 'maixduino.audio.audioVolume',
					default: 'Audio volume [VOLUME] %',
                        description: 'maixduino audio volume'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        VOLUME: {
								type: ArgumentType.NUMBER,
								defaultValue: 50
						}
                    }
                },
				{
                    opcode: 'audioStatus',
                    text: formatMessage({
                        id: 'maixduino.audio.audioStatus',
                        default: 'Audio get play status',
                        description: 'maixduino audio status'
                    }),
                    blockType: BlockType.REPORTER,
                    disableMonitor: true,
                    arguments: {}
                },
				{
                    opcode: 'audioRecord',
                    text: formatMessage({
                        id: 'maixduino.audio.audioRecord',
					default: 'Audio record [LOC] for [VALUE] sec and [WAIT]',
                        description: 'maixduino audio record'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
						LOC: {
                            type: ArgumentType.STRING,
                            defaultValue: '/sd/MaixDuino.wav'
                        },
                        VALUE: {
								type: ArgumentType.NUMBER,
								defaultValue: 5
						},
						WAIT: {
							type:ArgumentType.STRING,
							menu: 'wait',
						defaultValue: Wait.WAIT
						}
                    }
                }
            ],
            menus: {
				wait: {
					items: this.WAIT_MENU
				}
			}
        },
		{
            id: 'video',
            name: formatMessage({
                id: 'maixduino.category.video',
                default: 'Video',
                description: 'The name of the maixduino device video category'
            }),
            color1: '#00CC58',
            color2: '#00B34D',
            color3: '#009942',

            blocks: [
                {
                    opcode: 'videoPlay',
                    text: formatMessage({
                        id: 'maixduino.video.videoPlay',
					default: 'Video play [LOC] [WAIT]',
                        description: 'maixduino video play'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        LOC: {
                            type: ArgumentType.STRING,
                            defaultValue: '/sd/MaixDuino.wav'
                        },
						WAIT: {
							type:ArgumentType.STRING,
							menu: 'wait',
						defaultValue: Wait.WAIT
						}
                    }
                },
				{
                    opcode: 'videoVolume',
                    text: formatMessage({
                        id: 'maixduino.video.videoVolume',
					default: 'Video volume [VOLUME] %',
                        description: 'maixduino video volume'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        VOLUME: {
								type: ArgumentType.NUMBER,
								defaultValue: 50
						}
                    }
                },
				{
                    opcode: 'videoStatus',
                    text: formatMessage({
                        id: 'maixduino.video.videoStatus',
                        default: 'Video get play status',
                        description: 'maixduino video status'
                    }),
                    blockType: BlockType.REPORTER,
                    disableMonitor: true,
                    arguments: {}
                },
				{
                    opcode: 'videoRecord',
                    text: formatMessage({
                        id: 'maixduino.video.videoRecord',
					default: 'Video record [LOC] for [VALUE] sec and [WAIT]',
                        description: 'maixduino video record'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
						LOC: {
                            type: ArgumentType.STRING,
                            defaultValue: '/sd/MaixDuino.wav'
                        },
                        VALUE: {
								type: ArgumentType.NUMBER,
								defaultValue: 5
						},
						WAIT: {
							type:ArgumentType.STRING,
							menu: 'wait',
						defaultValue: Wait.WAIT
						}
                    }
                }
            ],
            menus: {
				wait: {
					items: this.WAIT_MENU
				}
			}
        },
		{
            id: 'system',
            name: formatMessage({
                id: 'maixduino.category.system',
                default: 'System',
                description: 'The name of the maixduino device system resource'
            }),
            color1: '#95B300',
            color2: '#7F9900',
            color3: '#6A8000',

            blocks: [
                {
                    opcode: 'getExecutiveTime',
                    text: formatMessage({
                        id: 'maixduino.system.getExecutiveTime',
                        default: 'Return execute time difference, begin: [BEGIN] end: [END]',
                        description: 'maixduino system get executive time'
                    }),
                    blockType: BlockType.REPORTER,
                    arguments: {
                        BEGIN: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 0
                        },
                        END: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 0
                        }
                    }
                },
				{
                    opcode: 'executeTime',
                    text: formatMessage({
                        id: 'maixduino.system.executeTime',
                        default: 'Execute time [TIME]',
                        description: 'maixduino system execute time'
                    }),
                    blockType: BlockType.REPORTER,
                    arguments: {
                        TIME: {
                            type: ArgumentType.STRING,
							menu: 'exeTime',
                            defaultValue: time.SEC
                        }
                    }
                },
				{
                    opcode: 'garbageCollection',
                    text: formatMessage({
                        id: 'maixduino.system.garbageCollection',
                        default: 'set [COLLECTION] garbage collection',
                        description: 'maixduino system garbage collection'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        COLLECTION: {
                            type: ArgumentType.STRING,
							menu: 'collect',
                            defaultValue: collection.AUTOMATE
                        }
                    }
                },
				{
                    opcode: 'reset',
                    text: formatMessage({
                        id: 'maixduino.system.reset',
                        default: 'System Reset',
                        description: 'maixduino system reset'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {}
                }
            ],
            menus: {
				exeTime: {
					items: this.TIME_MENU
				},
				collect: {
					items: this.COLLECTION_MENU
				}
			}
        },
        {
            id: 'console',
            name: formatMessage({
                id: 'maixduino.category.console',
                default: 'Console',
                description: 'The name of the maixduino device console category'
            }),
            color1: '#FF3399',
            color2: '#FF3399',
            color3: '#FF3399',

            blocks: [
                {
                    opcode: 'consolePrint',
                    text: formatMessage({
                        id: 'maixduino.console.consolePrint',
                        default: 'print [TEXT]',
                        description: 'maixduino console print'
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

module.exports = OpenBlockMaixduinoDevice;

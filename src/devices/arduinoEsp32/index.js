const formatMessage = require('format-message');

const ArgumentType = require('../../extension-support/argument-type');
const BlockType = require('../../extension-support/block-type');
const ProgramModeType = require('../../extension-support/program-mode-type');

const EspPeripheral = require('../arduinoCommon/esp-peripheral');

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
    hupcl:false
};

/**
 * Configuration for arduino-cli.
 * @readonly
 */
const DIVECE_OPT = {
    type: 'arduino',
    fqbn: 'esp32:esp32:esp32'
};

const menuIconURI = "data:image/svg+xml;base64,PD94bWwgdmVyc2lvbj0iMS4wIiBlbmNvZGluZz0iVVRGLTgiIHN0YW5kYWxvbmU9Im5vIj8+CjwhRE9DVFlQRSBzdmcgUFVCTElDICItLy9XM0MvL0RURCBTVkcgMS4xLy9FTiIgImh0dHA6Ly93d3cudzMub3JnL0dyYXBoaWNzL1NWRy8xLjEvRFREL3N2ZzExLmR0ZCI+CjxzdmcgdmVyc2lvbj0iMS4xIiBpZD0iTGF5ZXJfMSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiB4bWxuczp4bGluaz0iaHR0cDovL3d3dy53My5vcmcvMTk5OS94bGluayIgeD0iMHB4IiB5PSIwcHgiIHdpZHRoPSIyMHB4IiBoZWlnaHQ9IjIwcHgiIHZpZXdCb3g9IjAgMCAyMCAyMCIgZW5hYmxlLWJhY2tncm91bmQ9Im5ldyAwIDAgMjAgMjAiIHhtbDpzcGFjZT0icHJlc2VydmUiPiAgPGltYWdlIGlkPSJpbWFnZTAiIHdpZHRoPSIyMCIgaGVpZ2h0PSIyMCIgeD0iMCIgeT0iMCIKICAgIGhyZWY9ImRhdGE6aW1hZ2UvcG5nO2Jhc2U2NCxpVkJPUncwS0dnb0FBQUFOU1VoRVVnQUFBQlFBQUFBVUNBTUFBQUM2ViswL0FBQUFCR2RCVFVFQUFMR1BDL3hoQlFBQUFDQmpTRkpOCkFBQjZKUUFBZ0lNQUFQbi9BQUNBNlFBQWRUQUFBT3BnQUFBNm1BQUFGMitTWDhWR0FBQUMzMUJNVkVYLy8vLy8rdnIvOS9qLzF0Zi8KOGZILy92Ny8zK0QvMGRILzF0Yi9sWmovTXpmL0tDei9NRFQvS1MzL01EVC9pSXIvLy8vL201My9oNG4vTlRuL0xqTC9LQ3ovVlZqLwowdFQvdkw3L2JuRC9LaTcvTWpiL2lJdi9URS8vdHJqL3k4ei9scGovYkc3L2NITC8xTlgvME5IL1AwUC9wS2Iva1pQL1EwZi9wYWYvCkxUTC9kWGovLy8vL1BELy9XRnYvcks3L05Eai9XMTcvVVZUL2pvLy8rUGovU2szL1ZGai8zK0QvblovL2NuWC9kM24vNXViL3ZyLy8KbFpmL1kyWC9QMFAvcEtiL1RsTC9lSHYvZVh2L0xUTC9qWkQvb0tML3VMci9rNWIvMGRMLzM5Ly9LeS8vU1UzL3E2My94Y2IvT3ozLwpiM0gvNmVuLzhmSC9yYTcvdmIvL2FtMy8zTjMvSUNUL1RsTC9pWXovdjcvL2RYai9NamIveXN2L3g4ai8rdnIvYzNiL1RFLy9NemYvCkxqTC9xYXIvVmxuL21adi95TW4vdExiLzV1Zi9hV3Yvakk3L3NMSC9vcVQvZjRML0ppdi9aMnIvOC9QL2hZZi9kM24vdExYL1RWSC8KNStqL1MwLy9zTEgvSUNUL3A2bi9oSWYvZjRMLzM5Ly9WbHIvN096L3pNMy9VbGIvMU5YL2pwRC91cnovMTlmLzUrZi9TVXovWFdELwoxTlgvVUZUL1ltYi9TazMvSlNuL0hTTC9wYWYveXN2L0d4My9RMGIvNCtQL1Yxdi8zTnovV1YzL2xKZi9LeTcvS1MzL2pwRC9tNXovCmlvei9xNjMvZ0lQL2Y0TC9tcHovbEpmL3U3ei9UbEwvS3kvL01UWC9WMXYvbFpmL2UzNy9nb1QvbEpiL2Rubi9pWXovZW56L0xERC8KanBEL1RWRC9VbFgvZTMzL2dvWC9MREgvS2k3L0xERC9KU3IvS2k3L0x6UC9NVFgvS3kvL0t5Ny9kWGYvK3ZyL0FBRC9JQ1QvTkRqLwpNRFQvTGpML0xUSC9MelAvSkNuL0h5VC9NamIvTVRYL0t5Ly9KeXYvSFNIL0RoUC9EeFQvS1MzL016Zi9KU24vSlNyL0l5Zi9BQUgvCkhDRC9FUmIvQ3hEL0VoYi9IeVAvS1M3L0pDai9EUkwvUEVEL0lpZi9NemIvSVNYL05Ubi9BQVgvSWliL0hTTC9DZy8vS0N6L0NRNy8KTEREL0hpUC9LQzMvQ3cvL0dSMy9HUjcvSmlyL0hpTC9FeGYvUEQvL0doNy9GUm4vLy84TkRCUy9BQUFBdjNSU1RsTUFBQUFBQUFBQQpBQUNHNC9MeTh1ZVRBb1djOWZyNnpENU9zZmoybk41WEsyK3FxQ3c2NG51bXBXcnpxUWpwMG5Ib3hjR0JETjdXTVlhNW55SXZjN25oCmF1Q3VzL1NUZkZXSk55bjVuVVZUMHFNWEdHcFd2eDcrbUlFK3BQdEdTUWV6eFByMGRPQjRBMTRneFpSa2dxUDR0Qktnc21QZkROMXUKL0ZxRXJpVFhBUnpTSzVoeEhBM254Z3JXbk9MOC9IbFkrT2tIMFE3VmxmajVpM2FIWXBTYWYzaFp6ZmY0eDRPWmxYbVdrYVg3bFBMdApxcG5vOHZmKytmUHk4dXFsQzRtaEZqb0FBQUFCWWt0SFJBQ0lCUjFJQUFBQUNYQklXWE1BQUNjUUFBQW5FQUdVYVZFWkFBQUFCM1JKClRVVUg1UXNXQnh3a1Y4UzJ5Z0FBQnVaNlZGaDBVbUYzSUhCeWIyWnBiR1VnZEhsd1pTQjRiWEFBQUhpYzdWMUpqdXM0RE4zekZQOEkKanFqQlBrN0t3KzREdmV6ajk2T1VlSjRTRnhvMG9BcmdPSTQ0UEk2eVdVRG8zNy8vMEo4L2Z4NjJMQTF4elYwb1ErRWZudjJQZDhHYQp3aHZ2ZlBDVmI3a3hwdTErZm40NlkzQzk4bGF1dU1ET05sellKaFNXc2JiMEZka3lQQU1JSFllbmJaMzFlQWREWmhBWnd4MjN4WlByClVQSXpsQjZFdmhGaC9tRUsrZXhyM3dhVzcwZ2tRQnZyTzlHRG4rbUxmbm5VWkdDRGF6OUNZWHNLVTdqU05xNGdJOHAxSVY1aVoxcjIKcG9FK0QyWUdDdzVjNGRxREhWZHN1R1JyYWx3MVhPQ2FOeDNlTFk0UHJzZzBrZWo1T2xvdXpQeGxCRjQ2QjBnRFhUdy9uYkhXK2dFZwpSWVRwU3dGWkJvdFh3VStBNmtMOE0yM0FJdE5HdlVPU0w2K29qOEhSNE5oUVltS2F3QUZlRXJ1RUV1QmdLZmwrMEdXdUVkU0I4K0FhCjR5dXhIY0dsblcraDRtc0JERzhnRzZZV0RXR201d0xxNnpYem12aGlDU3BxMUE3ZXhEbGM0aHZBTGFGeklRRGhoRUs4bWtUUnNTd1IKWlJzRTRLNDQycGNIaHMwZ2RVOW84aG9IYTEzd3lTN2ZnU1JjUU9JRVdCNkxPbWVpTDhUWUJnUk8yQ09YNEUzeDRaNWVORmdETW5rbApsb3JJdEI3aVNOYmg2RURuSkpWRVRHQjZxZlk0WjRsb2lBTFozaUhpZ0NXZVJhL1RhWXNzeUtjaHNPcitoUTRXWWV2WGttZ1FUQXZKCkw2Smx5TzJMSXhqWjRXTzFMMjh1N2swemlLTmxZdmJwR1dNYTZTbUo2UkF6WWgvamhHbm5hMjRueGEyZ2hRbFI2Q3ozeFV0S0hSYWEKMHNVYUxxU29MR3dxcm1OSTlxSm9UZFpwRFoyMTZaMWJpcElOa0VxbGxDTkMyRm1jRzlSTGgrTDZqRVgxQjZ0UTBWRkZpN2t1aVNtTgp1UjVvOG1vSVBxMS9ESFNpREYzUVpzS1l4cHdQTlpJOGc5TkROZFZHbEtHdnRaa3hwU05OWW0xOFdta01ISUcwc2J5QW5OdTBBbUZhCm9XV2ptejhSMVJidkREMFk3K2dwaktMRndZdW1KWm9rUWd1ZHhhQnA0Q3F1TWI0RGd0Z3VLaTkwMkVSTWxONlR2NjgzUlVJSlJRNTkKbVVxRWt4MUJYQkYzSTJ2MUNFMU1OSExSN3VqcjBpaHhGQWd1T1FHTUdXZm9xMUpGQU04S0ROa0M0UnFnNHhPdWc3cW1HYWhWM1U0NApvNU9XWGFWTkZlVFU2NHg2NzlnRUxuWi8rRVFNRFUxRmUwUzI1d0piS3ZFRWZJZXRnZmdKM29tUXNQc1N2K0NUUkJpK2piRHR3am1kClpQK2hQaWVBeWRZdnVoa2NRN1ZPMGhPKzkwVUloL0JZcmwwVS80RUV0YmdhZGlteVhZVTRzSG0zMDdndDdSc3FwWTY2VlhCWGRSc2EKdkd5Q09MWUxiQ0phaE42bzV2bGt5dFhGc25aTEhPM0tPMHpWSVJob0dnMWZCd1BUa2Z5emxxTXhsQkNRaEU0czhYbExvTGxwQjJhZgplWkZlNU5LS2NRWTJEL0VndW1oSTBUM3ZxRnNObFU1MTFMSEdHeUlwbkRCb1pEYUs5eldLWlJ4OWxTQ1MvYitTSUlDR1NHVkk4OThuCnh3emFOOGt4Ym1QMGJSL3IyOWhMUEYxUGptbEFYa2lPUmRKK214eUptcTRuUjhvTnVwNGNLVGMyZC80N1lTZ2FpVnRDakwwV2UyMkkKcEwwbG4rUWdiU1hoMnVLdG5oWWplOUYrMFRXUG1pYlg4ZDRPa1IzdjRacTQ5V01KZUp3S25IZ3pCV0RDZkxsMHMzbEhmOUowd3lUUApTUUFXeEsvdHB1eGJPV3E5c3IwYXI2YjM4ajFwTS9KSzRzZEpyVUk0QUU4ZGJ5RndhMnh3WjRXYk1peEFMc1JFclhDVVZEYnhTYzJXClBoT0c5T2E0b2RINzNqSHVnWER1WHM5UCtpcjFFaFNmamV3dVBCTGd5bFJ3eFVhMVBGSVpTU2lHVjN6STgvNVVEemVxQTRNM1BaMlcKWEUwSjU0THBVUElHZzlVZDIwVHlDcG1ENVJEdnRvK3BsZFcwdG53akxkNTM0Nk5BeEU0blBjRjA5SGJnL3JKajVvVEtWd1Vwc2JVcgpyWTE1SnlIQXZURk5zdHBBc2s1QkE4bVJ6QkdEYWltU0RtU3VrQ3lmQkpxNDgxOTlHRGdubVpTMytDZ3pQY09WV3BhZXc5SHdHUGRzCkcxcC9uU3IrL3lPamsrM29IdEFPYjdQdUJPM2tIdkllMEQ1SkdmWFF2czkvaGRDdUZETnQwS1pZTkdoMGdkR2VZMjRIN1h5VTNRRGEKdHltakV0cnY1TDhPYUdNc09qUzZ3R2piTVRlRWRqYktiZ0h0dTVSUkN1MDM4bDhMdEFHTEZvMHVNRG81WUxrSHRITlJkaE5vMzZTTQpXbWpYODE4UHREY1dQUnBkWUxUdW1KdENPeE5sdDRIMmVjb29oblkxL3pWQjIvbkh5dnRCVzUydjNSWGFjWlRkQ05xbkthTWEyclg4CjF3Vk5zT2pTNkFLanczK0p1Uk8wb3lpN0ZiVFBVa1k1dER6TVZBb3REek8xUXN2RFRLM1E4akJUS2JROHpOUUtMUTh6bFVMTHcweXQKMFBJd1V5dTBQTXhVQ2kwUE03VkN5OE5NcGREeU1GTXJ0RHpNMUFvdER6T1ZRc3ZEVEszUThqQlRLYlE4ek5RS0xROHp0VUxMdzB5bAowUEl3VXl1MFBNelVDaTBQTTVWQ3k4Tk1yZER5TUZNcHREek0xQW90RHpPMVFzdkRUS1hROGpCVEs3UTh6RlFLTFE4enRVTEx3MHl0CjBQSXdVeW0wUE16VUNpMFBNNVZDeThOTXJkRHlNRk1ydER6TVZBb3REek8xUXN2RFRLM1E4akJUS2JROHpOUUtMUTh6WDJZWWZybVYKUmovZEtqK2VFSCtqTW9UMHE2djBIMXpSUUNiN1IyR0NBQUFBQVc5eVRsUUJ6NkozbWdBQUFaMUpSRUZVR05OallPRGs0dVpCQXJ4OAovQUlNZ3ZzUEhEd0VBb2VQSEFWUkI0L3RGMklRUG43aTBOR1RwMDZmUEhUbTdMbnpGdzRkT25sUmhFSDA0TUZMQnk2Zk9YSkZURnhDCjh1cTFneWNQWFpCaTREbDVVVnBHVms1ZVFWRkptVUZGOVN6UUxCNEduaFBYMWRRMUdCa1lHRFMxdEJsMGRHOWNBQW5lMU5QZmIyQm8KWk16QVlHSnF4bUIrN1JaSThLeUZwWlcxamEyZHZRTURnNk1UZy9OMW9LRExiVmMzQmlabUJnYjNZeDRNRExjOHZlNGM1bUh3dnV2agp5K0RuSHhESUVIUXZtQ0VrTkN6OEZFZ3dJcEtCSlNvNkpwWWg3bjQ4UTBKaTBoRWVodVFIS2FscDZRd01HUTh6R2JLeUdYSWVnWjEwCkp6ZHZmMzVCSVVOUk1VTkpLV3ZaMFdNZ3djZmxGWlZWMVRXc3RYWDFEWTBNVFUvT2d4My90TG1sdGEyZGpiMmprNk9ydTZmM0pFancKNUxPKy9na1RKekZNbnNJNjlmbTA2VE5lQUFWbkhyenc0T3FzMlhQbXpyczhmOEhDUll0ZkhycXdoR0hwN1pNWFhyMWV0bnpGeWxXcgoxNng5OC9iaHlmUHJHTmJ2ZjNmaDVJbjNINDV1MlBqeDArZkRKeTdjMnIrSlFXRHpscTJnYU5pMi9kaU9uYnQ0ZUhidjJic1BBR21tCnVIRGpHeEhOQUFBQUpYUkZXSFJrWVhSbE9tTnlaV0YwWlFBeU1ESXhMVEV4TFRJeVZEQTNPakk0T2pNMkt6QXdPakF3SVRyQlBBQUEKQUNWMFJWaDBaR0YwWlRwdGIyUnBabmtBTWpBeU1TMHhNUzB5TWxRd056b3lPRG96Tmlzd01Eb3dNRkJuZVlBQUFBQVRkRVZZZEdSagpPbVp2Y20xaGRBQnBiV0ZuWlM5d2JtZi91UnMrQUFBQUZYUkZXSFJ3YUc5MGIzTm9iM0E2UTI5c2IzSk5iMlJsQUROV0FyTkFBQUFBCkpuUkZXSFJ3YUc5MGIzTm9iM0E2U1VORFVISnZabWxzWlFCelVrZENJRWxGUXpZeE9UWTJMVEl1TVJ3dmJBc0FBQUFlZEVWWWRIUnAKWm1ZNldGSmxjMjlzZFhScGIyNEFNVEF3TURBd01DOHhNREF3TUZ2ZUZ2TUFBQUFlZEVWWWRIUnBabVk2V1ZKbGMyOXNkWFJwYjI0QQpNVEF3TURBd01DOHhNREF3TVA1Vmh2MEFBQUFRZEVWWWRIaHRjRHBEYjJ4dmNsTndZV05sQURFRkRzalJBQUFBS0hSRldIUjRiWEE2ClEzSmxZWFJsUkdGMFpRQXlNREl4TFRFeExUSXlWREV5T2pVNE9qRTJLekExT2pNd0g4N29Bd0FBQUROMFJWaDBlRzF3T2tOeVpXRjAKYjNKVWIyOXNBRUZrYjJKbElGQm9iM1J2YzJodmNDQkRReUF5TURFMUxqVWdLRmRwYm1SdmQzTXA3eVl2VHdBQUFDcDBSVmgwZUcxdwpPazFsZEdGa1lYUmhSR0YwWlFBeU1ESXhMVEV4TFRJeVZERXlPalU0T2pFMkt6QTFPak13bDVTSGhBQUFBQ2gwUlZoMGVHMXdPazF2ClpHbG1lVVJoZEdVQU1qQXlNUzB4TVMweU1sUXhNam8xT0RveE5pc3dOVG96TUtzdzFEb0FBQUFXZEVWWWRIaHRjRHBRYVhobGJGaEUKYVcxbGJuTnBiMjRBTWpDcUlNZHVBQUFBRm5SRldIUjRiWEE2VUdsNFpXeFpSR2x0Wlc1emFXOXVBREl3ZDdZZTZ3QUFBRXQwUlZoMAplRzF3VFUwNlJHOWpkVzFsYm5SSlJBQmhaRzlpWlRwa2IyTnBaRHB3YUc5MGIzTm9iM0E2WVRNMU1ESTROR1l0TkdJMk5TMHhNV1ZqCkxUazVabU10WkRneVlXTTVOMkU0Tm1NeXpXUXl1d0FBQUQxMFJWaDBlRzF3VFUwNlNXNXpkR0Z1WTJWSlJBQjRiWEF1YVdsa09tWTAKWkRNNE0yUTJMVGhoTjJFdE9EYzBOQzFpTmpFM0xUTTNZakprWmpsak9ESXpNT1o3Ky9RQUFBQkZkRVZZZEhodGNFMU5Pazl5YVdkcApibUZzUkc5amRXMWxiblJKUkFCNGJYQXVaR2xrT2pFNU0ySTVZMlpsTFRBMk1HUXRNelkwT1MwNE5qRmpMVFUzWmpreFpERTBPV0kyCk5NcUFJcllBQUFBQVNVVk9SSzVDWUlJPSIgLz4KPC9zdmc+Cg==";

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
    High: 'HIGH',
    Low: 'LOW'
};

const Channels = {
    CH0: '0',
    CH1: '1',
    CH2: '2',
    CH3: '3',
    CH4: '4',
    CH5: '5',
    CH6: '6',
    CH7: '7',
    CH8: '8',
    CH9: '9',
    CH10: '10',
    CH11: '11',
    CH12: '12',
    CH13: '13',
    CH14: '14',
    CH15: '15'
};

const SerialNo = {
    Serial0: '0',
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
    InputPullup: 'INPUT_PULLUP'
};

const InterrupMode = {
    Rising: 'RISING',
    Falling: 'FALLING',
    Change: 'CHANGE',
    Low: 'LOW',
    High: 'High'
};

const DataType = {
    Integer: 'INTEGER',
    Decimal: 'DECIMAL',
    String: 'STRING'
};

/**
 * Manage communication with a Arduino esp32 peripheral over a OpenBlock Link client socket.
 */
class ArduinoEsp32 extends EspPeripheral{
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
 * OpenBlock blocks to interact with a Arduino esp32 peripheral.
 */
class OpenBlockArduinoEsp32Device {
    /**
     * @return {string} - the ID of this extension.
     */
    static get DEVICE_ID () {
        return 'arduinoEsp32';
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
                    id: 'arduinoEsp32.modeMenu.input',
                    default: 'input',
                    description: 'label for input pin mode'
                }),
                value: Mode.Input
            },
            {
                text: formatMessage({
                    id: 'arduinoEsp32.modeMenu.output',
                    default: 'output',
                    description: 'label for output pin mode'
                }),
                value: Mode.Output
            },
            {
                text: formatMessage({
                    id: 'arduinoEsp32.modeMenu.inputPullup',
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

    get LEVEL_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'arduinoEsp32.levelMenu.high',
                    default: 'high',
                    description: 'label for high level'
                }),
                value: Level.High
            },
            {
                text: formatMessage({
                    id: 'arduinoEsp32.levelMenu.low',
                    default: 'low',
                    description: 'label for low level'
                }),
                value: Level.Low
            }
        ];
    }

    get LEDC_CHANNELS_MENU () {
        return [
            {
                text: 'CH0 (LT0)',
                value: Channels.CH0
            },
            {
                text: 'CH1 (LT0)',
                value: Channels.CH1
            },
            {
                text: 'CH2 (LT1)',
                value: Channels.CH2
            },
            {
                text: 'CH3 (LT1)',
                value: Channels.CH3
            },
            {
                text: 'CH4 (LT2)',
                value: Channels.CH4
            },
            {
                text: 'CH5 (LT2)',
                value: Channels.CH5
            },
            {
                text: 'CH6 (LT3)',
                value: Channels.CH6
            },
            {
                text: 'CH7 (LT3)',
                value: Channels.CH7
            },
            {
                text: 'CH8 (HT0)',
                value: Channels.CH8
            },
            {
                text: 'CH9 (HT0)',
                value: Channels.CH9
            },
            {
                text: 'CH10 (HT1)',
                value: Channels.CH10
            },
            {
                text: 'CH11 (HT1)',
                value: Channels.CH11
            },
            {
                text: 'CH12 (HT2)',
                value: Channels.CH12
            },
            {
                text: 'CH13 (HT2)',
                value: Channels.CH13
            },
            {
                text: 'CH14 (HT3)',
                value: Channels.CH14
            },
            {
                text: 'CH15 (HT3)',
                value: Channels.CH15
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
                    id: 'arduinoEsp32.InterrupModeMenu.risingEdge',
                    default: 'rising edge',
                    description: 'label for rising edge interrup'
                }),
                value: InterrupMode.Rising
            },
            {
                text: formatMessage({
                    id: 'arduinoEsp32.InterrupModeMenu.fallingEdge',
                    default: 'falling edge',
                    description: 'label for falling edge interrup'
                }),
                value: InterrupMode.Falling
            },
            {
                text: formatMessage({
                    id: 'arduinoEsp32.InterrupModeMenu.changeEdge',
                    default: 'change edge',
                    description: 'label for change edge interrup'
                }),
                value: InterrupMode.Change
            },
            {
                text: formatMessage({
                    id: 'arduinoEsp32.InterrupModeMenu.low',
                    default: 'low',
                    description: 'label for low interrup'
                }),
                value: InterrupMode.Low
            },
            {
                text: formatMessage({
                    id: 'arduinoEsp32.InterrupModeMenu.high',
                    default: 'high',
                    description: 'label for high interrup'
                }),
                value: InterrupMode.High
            }
        ];
    }

    get SERIAL_NO_MENU () {
        return [
            {
                text: '0',
                value: SerialNo.Serial0
            },
            // {
            //     text: '1',
            //     value: SerialNo.Serial1
            // },
            {
                text: '2',
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
                    id: 'arduinoEsp32.eolMenu.warp',
                    default: 'warp',
                    description: 'label for warp print'
                }),
                value: Eol.Warp
            },
            {
                text: formatMessage({
                    id: 'arduinoEsp32.eolMenu.noWarp',
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
                    id: 'arduinoEsp32.dataTypeMenu.integer',
                    default: 'integer',
                    description: 'label for integer'
                }),
                value: DataType.Integer
            },
            {
                text: formatMessage({
                    id: 'arduinoEsp32.dataTypeMenu.decimal',
                    default: 'decimal',
                    description: 'label for decimal number'
                }),
                value: DataType.Decimal
            },
            {
                text: formatMessage({
                    id: 'arduinoEsp32.dataTypeMenu.string',
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

        // Create a new Arduino esp32 peripheral instance
        this._peripheral = new ArduinoEsp32(this.runtime,
            OpenBlockArduinoEsp32Device.DEVICE_ID, originalDeviceId);
    }

    /**
     * @returns {Array.<object>} metadata for this extension and its blocks.
     */
    getInfo () {
        return [
			{
				id: 'ESP32',
				name: formatMessage({
                    id: 'arduinoEsp32.category.esp32',
                    default: 'ESP32',
                    description: 'The name of the ESP32 device'
                }),
				menuIconURI: menuIconURI,
                color1: '#FFBF00',
                color2: '#E6AC00',
                color3: '#CC9900',
				blocks: [
                    {
                        opcode: 'Esp32header',
                        text: formatMessage({
                            id: 'arduinoEsp32.esp32.header',
                            default: 'when ESP32 begins',
                            description: 'arduinoEsp32 header'
                        }),
                        blockType: BlockType.HAT,
						programMode: [ProgramModeType.UPLOAD]
                    }]
			},
            {
                id: 'pin',
                name: formatMessage({
                    id: 'esp32Arduino.category.pins',
                    default: 'Pins',
                    description: 'The name of the esp32 arduino device pin category'
                }),
                color1: '#4C97FF',
                color2: '#3373CC',
                color3: '#3373CC',

                blocks: [
                    {
                        opcode: 'setPinMode',
                        text: formatMessage({
                            id: 'esp32Arduino.pins.setPinMode',
                            default: 'set pin [PIN] mode [MODE]',
                            description: 'esp32Arduino set pin mode'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'outPins',
                                defaultValue: Pins.IO2
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
                            id: 'esp32Arduino.pins.setDigitalOutput',
                            default: 'set digital pin [PIN] out [LEVEL]',
                            description: 'esp32Arduino set digital pin out'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'outPins',
                                defaultValue: Pins.IO2
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
                            id: 'esp32Arduino.pins.esp32SetPwmOutput',
                            default: 'set pwm pin [PIN] use channel [CH] out [OUT]',
                            description: 'esp32Arduino set pwm pin out'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'outPins',
                                defaultValue: Pins.IO2
                            },
                            OUT: {
                                type: ArgumentType.NUMBER,
                                defaultValue: '0'
                            },
                            CH: {
                                type: ArgumentType.NUMBER,
                                menu: 'ledcChannels',
                                defaultValue: Channels.CH0
                            }
                        }
                    },
                    {

                        opcode: 'esp32SetDACOutput',
                        text: formatMessage({
                            id: 'esp32Arduino.pins.esp32SetDACOutput',
                            default: 'set dac pin [PIN] out [OUT]',
                            description: 'esp32Arduino set dac pin out'
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
                    '---',
                    {
                        opcode: 'readDigitalPin',
                        text: formatMessage({
                            id: 'arduinoEsp32.pins.readDigitalPin',
                            default: 'read digital pin [PIN]',
                            description: 'arduinoEsp32 read digital pin'
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
                        opcode: 'readAnalogPin',
                        text: formatMessage({
                            id: 'arduinoEsp32.pins.readAnalogPin',
                            default: 'read analog pin [PIN]',
                            description: 'arduinoEsp32 read analog pin'
                        }),
                        blockType: BlockType.REPORTER,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'analogPins',
                                defaultValue: Pins.IO2
                            }
                        }
                    },
                    {
                        opcode: 'esp32ReadTouchPin',
                        text: formatMessage({
                            id: 'arduinoEsp32.pins.esp32ReadTouchPin',
                            default: 'read touch pin [PIN]',
                            description: 'arduinoEsp32 read touch pin'
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
                            id: 'arduinoEsp32.pins.setServoOutput',
                            default: 'set servo pin [PIN] use channel [CH] out [OUT]',
                            description: 'arduinoEsp32 set servo pin out'
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
                            },
                            CH: {
                                type: ArgumentType.NUMBER,
                                menu: 'ledcChannels',
                                defaultValue: Channels.CH0
                            }
                        }
                    },
                    '---',
                    {

                        opcode: 'esp32AttachInterrupt',
                        text: formatMessage({
                            id: 'arduinoEsp32.pins.attachInterrupt',
                            default: 'attach interrupt pin [PIN] mode [MODE] executes',
                            description: 'arduinoEsp32 attach interrupt'
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
                    },
                    {

                        opcode: 'esp32DetachInterrupt',
                        text: formatMessage({
                            id: 'arduinoEsp32.pins.detachInterrupt',
                            default: 'detach interrupt pin [PIN]',
                            description: 'arduinoEsp32 detach interrupt'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'pins',
                                defaultValue: Pins.IO2
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
                    analogPins: {
                        items: this.ANALOG_PINS_MENU
                    },
                    level: {
                        acceptReporters: true,
                        items: this.LEVEL_MENU
                    },
                    ledcChannels: {
                        items: this.LEDC_CHANNELS_MENU
                    },
                    dacPins: {
                        items: this.DAC_PINS_MENU
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
                    id: 'arduinoEsp32.category.serial',
                    default: 'Serial',
                    description: 'The name of the arduino esp32 device serial category'
                }),
                color1: '#9966FF',
                color2: '#774DCB',
                color3: '#774DCB',

                blocks: [
                    {
                        opcode: 'multiSerialBegin',
                        text: formatMessage({
                            id: 'arduinoEsp32.serial.multiSerialBegin',
                            default: 'serial [NO] begin baudrate [VALUE]',
                            description: 'arduinoEsp32 multi serial begin'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            NO: {
                                type: ArgumentType.NUMBER,
                                menu: 'serialNo',
                                defaultValue: SerialNo.Serial0
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
                        opcode: 'multiSerialTimeout',
                        text: formatMessage({
                            id: 'arduinoEsp32.serial.multiSerialTimeout',
                            default: 'serial [NO] set timeout [ARG1]',
                            description: 'arduinoEsp32 multi serial begin'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            NO: {
                                type: ArgumentType.NUMBER,
                                menu: 'serialNo',
                                defaultValue: SerialNo.Serial0
                            },
                            ARG1: {
                                type: ArgumentType.NUMBER,
                                defaultValue: '100'
                            }
                        },
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {
                        opcode: 'multiSerialPrint',
                        text: formatMessage({
                            id: 'arduinoEsp32.serial.multiSerialPrint',
                            default: 'serial [NO] print [VALUE] [EOL]',
                            description: 'arduinoEsp32 multi serial print'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            NO: {
                                type: ArgumentType.NUMBER,
                                menu: 'serialNo',
                                defaultValue: SerialNo.Serial0
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
                            id: 'arduinoEsp32.serial.multiSerialAvailable',
                            default: 'serial [NO] available data length',
                            description: 'arduinoEsp32 multi serial available data length'
                        }),
                        arguments: {
                            NO: {
                                type: ArgumentType.NUMBER,
                                menu: 'serialNo',
                                defaultValue: SerialNo.Serial0
                            }
                        },
                        blockType: BlockType.REPORTER,
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {
                        opcode: 'multiSerialReadAByte',
                        text: formatMessage({
                            id: 'arduinoEsp32.serial.multiSerialReadAByte',
                            default: 'serial [NO] read a byte',
                            description: 'arduinoEsp32 multi serial read a byte'
                        }),
                        arguments: {
                            NO: {
                                type: ArgumentType.NUMBER,
                                menu: 'serialNo',
                                defaultValue: SerialNo.Serial0
                            }
                        },
                        blockType: BlockType.REPORTER,
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {
                        opcode: 'multiSerialReadInteger',
                        text: formatMessage({
                            id: 'arduinoEsp32.serial.multiSerialReadInteger',
                            default: 'serial [NO] read integer',
                            description: 'arduinoEsp32 multi serial read integer'
                        }),
                        arguments: {
                            NO: {
                                type: ArgumentType.NUMBER,
                                menu: 'serialNo',
                                defaultValue: SerialNo.Serial0
                            }
                        },
                        blockType: BlockType.REPORTER,
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {
                        opcode: 'multiSerialReadString',
                        text: formatMessage({
                            id: 'arduinoEsp32.serial.multiSerialReadString',
                            default: 'serial [NO] read String',
                            description: 'arduinoEsp32 multi serial read string'
                        }),
                        arguments: {
                            NO: {
                                type: ArgumentType.NUMBER,
                                menu: 'serialNo',
                                defaultValue: SerialNo.Serial0
                            }
                        },
                        blockType: BlockType.COMMAND,
                        programMode: [ProgramModeType.UPLOAD]
                    },
                    {
                        opcode: 'multiSerialCompareString',
                        text: formatMessage({
                            id: 'arduinoEsp32.serial.multiSerialCompareString',
                            default: 'serial [NO] Compare String [VALUE]',
                            description: 'arduinoEsp32 multi serial compare string'
                        }),
                        arguments: {
                            NO: {
                                type: ArgumentType.NUMBER,
                                menu: 'serialNo',
                                defaultValue: SerialNo.Serial0
                            },
                            VALUE: {
                                type: ArgumentType.STRING,
                                defaultValue: 'LEDON'
                            }
                        },
                        blockType: BlockType.BOOLEAN,
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
                id: 'sensor',
                name: formatMessage({
                    id: 'arduinoEsp32.category.sensor',
                    default: 'Sensor',
                    description: 'The name of the arduino esp32 device sensor category'
                }),
                color1: '#4CBFE6',
                color2: '#2E8EB8',
                color3: '#2E8EB8',

                blocks: [
                    {
                        opcode: 'esp32ReadHallSensor',
                        text: formatMessage({
                            id: 'arduinoEsp32.sensor.readHallSensor',
                            default: 'read hall sensor',
                            description: 'arduino esp32 read hall sensor'
                        }),
                        blockType: BlockType.REPORTER,
                        disableMonitor: true
                    },
                    '---',
                    {
                        opcode: 'runningTime',
                        text: formatMessage({
                            id: 'arduinoEsp32.sensor.runningTime',
                            default: 'running time (millis)',
                            description: 'arduino esp32 running time'
                        }),
                        blockType: BlockType.REPORTER,
                        disableMonitor: true
                    }
                ]
            },
            {
                id: 'data',
                name: formatMessage({
                    id: 'arduinoEsp32.category.data',
                    default: 'Data',
                    description: 'The name of the arduino esp32 device data category'
                }),
                color1: '#CF63CF',
                color2: '#C94FC9',
                color3: '#BD42BD',

                blocks: [
                    {
                        opcode: 'dataMap',
                        text: formatMessage({
                            id: 'arduinoEsp32.data.dataMap',
                            default: 'map [DATA] from ([ARG0], [ARG1]) to ([ARG2], [ARG3])',
                            description: 'arduinoEsp32 data map'
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
                            id: 'arduinoEsp32.data.dataConstrain',
                            default: 'constrain [DATA] between ([ARG0], [ARG1])',
                            description: 'arduinoEsp32 data constrain'
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
                            id: 'arduinoEsp32.data.dataConvert',
                            default: 'convert [DATA] to [TYPE]',
                            description: 'arduinoEsp32 data convert'
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
                            id: 'arduinoEsp32.data.dataConvertASCIICharacter',
                            default: 'convert [DATA] to ASCII character',
                            description: 'arduinoEsp32 data convert to ASCII character'
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
                            id: 'arduinoEsp32.data.dataConvertASCIINumber',
                            default: 'convert [DATA] to ASCII number',
                            description: 'arduinoEsp32 data convert to ASCII number'
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

module.exports = OpenBlockArduinoEsp32Device;

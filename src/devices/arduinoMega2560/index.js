const formatMessage = require('format-message');

const ArgumentType = require('../../extension-support/argument-type');
const BlockType = require('../../extension-support/block-type');
const ProgramModeType = require('../../extension-support/program-mode-type');

const ArduinoPeripheral = require('../arduinoCommon/arduino-peripheral');

/**
 * The list of USB device filters.
 * @readonly
 */
const PNPID_LIST = [
    // https://github.com/arduino/Arduino/blob/1.8.0/hardware/arduino/avr/boards.txt#L175-L186
    'USB\\VID_2341&PID_0010',
    'USB\\VID_2341&PID_0042',
    'USB\\VID_2A03&PID_0010',
    'USB\\VID_2A03&PID_0042',
    'USB\\VID_2341&PID_0210',
    'USB\\VID_2341&PID_0242',
    // For chinese clones that use CH340
    'USB\\VID_1A86&PID_7523'
];

/**
 * Configuration of serialport
 * @readonly
 */
const SERIAL_CONFIG = {
    baudRate: 57600,
    dataBits: 8,
    stopBits: 1,
    hupcl:true
};

/**
 * Configuration for arduino-cli.
 * @readonly
 */
const DIVECE_OPT = {
    type: 'arduino',
    fqbn: 'arduino:avr:mega:cpu=atmega2560',
    firmware: 'arduinoMega2560.standardFirmata.ino.hex'
};

const menuIconURI = "data:image/svg+xml;base64,PD94bWwgdmVyc2lvbj0iMS4wIiBlbmNvZGluZz0iVVRGLTgiIHN0YW5kYWxvbmU9Im5vIj8+CjwhRE9DVFlQRSBzdmcgUFVCTElDICItLy9XM0MvL0RURCBTVkcgMS4xLy9FTiIgImh0dHA6Ly93d3cudzMub3JnL0dyYXBoaWNzL1NWRy8xLjEvRFREL3N2ZzExLmR0ZCI+CjxzdmcgdmVyc2lvbj0iMS4xIiBpZD0iTGF5ZXJfMSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiB4bWxuczp4bGluaz0iaHR0cDovL3d3dy53My5vcmcvMTk5OS94bGluayIgeD0iMHB4IiB5PSIwcHgiIHdpZHRoPSIzM3B4IiBoZWlnaHQ9IjMzcHgiIHZpZXdCb3g9IjAgMCAzMyAzMyIgZW5hYmxlLWJhY2tncm91bmQ9Im5ldyAwIDAgMzMgMzMiIHhtbDpzcGFjZT0icHJlc2VydmUiPiAgPGltYWdlIGlkPSJpbWFnZTAiIHdpZHRoPSIzMyIgaGVpZ2h0PSIzMyIgeD0iMCIgeT0iMCIKICAgIGhyZWY9ImRhdGE6aW1hZ2UvcG5nO2Jhc2U2NCxpVkJPUncwS0dnb0FBQUFOU1VoRVVnQUFBQ0VBQUFBaENBWUFBQUJYNU1KdkFBQUFCR2RCVFVFQUFMR1BDL3hoQlFBQUFDQmpTRkpOCkFBQjZKUUFBZ0lNQUFQbi9BQUNBNlFBQWRUQUFBT3BnQUFBNm1BQUFGMitTWDhWR0FBQUFCbUpMUjBRQS93RC9BUCtndmFlVEFBQUEKQ1hCSVdYTUFBQVlPQUFBR0RnRXBzUlBTQUFBR0hIcFVXSFJTWVhjZ2NISnZabWxzWlNCMGVYQmxJSGh0Y0FBQWVKenRuVW1PNnpnTQpodmM4UlIzQkVUWFl4M0Y1MkQyZ2wzMzgvaW5IUTJ4NVNQTFFvQUZWZ0NUbFNDSS9UbExDQUtGLy8veERQejgvRDh2T0V6ZmNoeklVCi91SFovM29YckNtODhjNEhYL21PVzJPNi92ZjN0emNHMXl0djVZb0w3R3pMaFcxRFlSbGpTMStSTFVNZE1ORnhxRzNuck1jakZtVEcKSkdPNDU2Nm91UWtsMTZIMG1PaGJFZVlmcHBEL2ZlTzd3UElhaVFSb1kzMHZlbkE5dkRBTmo1ck15K0Rhcjh5dzB3eFR1TksycmlBagp5dlVoWG1Kbk92YW1oVDRQWnNZU0hMakN0UWM3cnRod3lkWTB1R3E0d0RWdmVqeGEzRCs0SXRQR1NmWHozbkpoMWpjamVNTnpRQnJvCjRybDJ4bHJyWjBDS2hNT0xBbGtHaTF2Qk5hRDZFUDlNRnpESWRGSHZNTWlYVzlUSDRON2d2cVZoRWRNR0R2Q1MyQ1dVZ0lPbDVQVloKbDdWR1VBZk9nMnVNcjhSMkJKZjJ2b09LendFd3ZJRnNtRm8waEpucURlcnp0dkthK0dJTEZUWHFabS9pT1Z6aVcrQ1cwTGtRUURpaApFSzhPb3VoTUZwYmloUFVLNkd4OE0xdU9aQ0EwY0pqb0pIckVUdEJta1BpWUpaNEpKTVJxaVZXQkFOUEpzL2RBcDlrMFR0OVNueHRZCjdHdGJaQjFFMDZkR3hvTHRVbWpDMkM5U09WanJnaCtDNFVnb0hVbVZhaEVRYmhqVE94TURzSS9lUXVtUTVRZlhTT0FXZGRMOUc3MHMKd3RhbmttaldpamEyZUU1NjEvaUVrSFA0dHpxV3R4WTN6cG5GMFRZeHAvU003a1Y2U21JNldFOWl6VGhadFBjTmR5L0ZyYUIxT0VxaApzendWTHlsMUdHaEtGMnU0VEVWbFlWTnhFNTB6aWFLVXJNc2FPbXVIUis0b1NqWWdsVW9wOS9DbXMzaHVVQzhaQlVlcXBqVy9HT1hpCmxXS3R5N0FvTFZjOTBhU0Z2dEdub1pybmpJclFqaWJWVU1WM05ka3NTcGMwaVZ1VEg3UjRiTFVSWmVnamJSSUxIMmdVYTJPTklKY2EKMk1wZzdnQ0RaOWpPWGF4Rm5ROGlRa2JTZXVnTzVyYjgxYkt4SWowd1Z5d21wVlpvT1NwL090enZlcGRHbzhaS1Yxdloyemhhb0l2RgpBajdoYmhoaEJFek9JelZzRnJ6QmRzN1JvZ0ZXQmhxS0RwS3U1Ukl2U1h3QkVVWjNDTUpXeklsa052RXNVT0I4Z3UwZXo5MXMrbGs4CmJlVWZCY056b2lRVGgyblhrWG0wVU56dUszNnVOMTFRUENGL3F6Y2xGSmRncUlhakgyUTE2WVhXSFBRWnlKYUROaUNYOU5rNmhEWmcKcU1YUmsxZ3dWR2tmVHA0Y1QyL3dPSVhIMGVEMUZNUjROWitsNUZBTmNWakdXUm8yd0hoNlhtMkJlOXRDVWhEVHRDWExXWTNqcm9adApDSG0rcU16d2lsaVM5OGFLdU4yZDlsWGVZb21kYktUemRMd1dESlNJNmxQNXlUSnloaElDS3BNVFN4eHZYTFM3Y3lVWDIvY2lIZWdpCnRSSFBzTXhEUEdoanJaYm9YdS83VWszcDQzMS9KWkwyWko1SHUrbVg4VTdIMlRRUGxuemFTeWNaUis4bUxPdzRIZ1RIU2hFWktDYkIKTTlSaGwyWSsvTDBPWEk3YjI0Nm1pYTRjRTljMzh2NW9JYUNZYi9FZDIvamZKSmlPRmpoRGM5VThqOTZWdkY1Z25FK1hKQ2YwZGJBYwpYR05qNmtTdlhSeStjMklaajlZMXpZY0V4OC9QSStSUTRaRjNqcC8xWUI2ZUhwMHNiRXNweUtJcVNMbzJyclEybm8wa0ZIaXlLUStuCkdKbEIxNmFjZ2NtQmZWeWdPcE81RWZreWc5SXlsMW0yZm9lL25qR01wZVVIRk1Nbk03THpyaitiT2E4R0YzYVJhN2YvWTZGcnUreE4KME42RDBZMTI4TDcvZm1qSmMvWmQwWTZqN0dabzc2U01lclRQODE4aDJqZkZUQnZhSzRzR2piNVk2TWd4dDBPN0htVTNRUHMwWlZTaQovWjM4MTRHMlpOR2gwUmNMWGZ4STR4NW9WNlBzRm1pZnBZeFN0TCtSLzFyUVpoWXRHbjJ4MEo1amJvbDJMY3B1Z3ZaSnlxaEYrejcvCjlhQWRmTG5pZm1ocHg5d1U3VXFVM1FidC9aUlJqUFp0L210Q08vZ2UyLzNRTHJaWDc0RjJIbVUzUW5zM1pWU2pmWmYvdXRBdWZybmkKSG1qSjc5YmNGUzAzTTdXaTVXYW1VclRjek5TS2xwdVpXdEZ5TTFNcFdtNW1ha1hMelV5bGFMbVpxUlV0TnpPMW91Vm1wbEswM016VQppcGFibVVyUmNqTlRLMXB1Wm1wRnk4MU1wV2k1bWFrVkxUY3psYUxsWnFaV3ROek0xSXFXbTVsSzBYSXpVeXRhYm1acVJjdk5US1ZvCnVabXBGUzAzTTVXaTVXYW1WclRjek5TS2xwdVpTdEZ5TTFNcldtNW1La1hMelV5dGFMbVpxUlV0TnpPVm91Vm1wbGEwM014VWlwYWIKbVZyUmNqTlRLMXB1WmlwRnk4MU1yV2k1bWFrVkxUY3psYUxsWnVZNE5mbDdqUElqQ3ZFMzJFSVlma3FSL2dPNmdGVkpGNDh2REFBQQpBQUZ2Y2s1VUFjK2lkNW9BQUFveFNVUkJWRmpEdFZocGtGeFZHVDEzZVh2M2RFOVBNajJUbnBCbHNwRVFDQkFDSVFFaU1ZYUFFVEdpCkpWYUppQllVSlZJdTVWSklYS3FRd3JKRXBCQmpLZTVsVUJRUUxaUXRoTEFrRlFqWlRDWWhrNWxoa3N4a2V1OSszVys5OS9xalp3WUkKb29YaTkrdTlkNnZ1UGQvNXRuTWZ1WHYzSGdBQVFjdnFVWXpqYmhPbElFQWppaEVwZ1NXWkRJNjdEUnh2TkZhVWcvQlNYNGh6SWlIbQpOS09vRTFJWkFFS0xzVEdEa1NNR1pidVNHdCtTdGUxdFNWMURNNG9nQUJTYlRXUk1FMU1jRzIyR0FTZ0ZOWDRteDc4d05RNHFxWE5VClFrbDNGMHVmUFZDdVhGY09naVZTS2tBQm5BSUdvYUFBbEZJb0JjRlVJY1FpUUYxSlFiNlYwdDI5M1k3MWl6bnAxRDAyWjNHeGljbEQKVHpWeUtoTzFLTVpZMDBjampqRFNhRzU0Zm5Uc2pub2N6OVVJUlZKallDQ0FVaUJRSUFxQUd0OWFxZkZuQ2FVQU40d1F4REV5cHRGLwo1dFNPVzIzT0g3QTRSOG95a1RxRkNmcEdSRklwT0p3amJXaDQ3c1Rvdlg4N052S2dKK1hjdEs0anFYR1FjYS8vblUwc081d2pZNXFvCmhXSHYxdUhqbS9mbUN6OUo2Qm9zemlGUDJZTk9NS0FBWkV3VFVpazhjTGovOFVPbHlrMDZaM0EwUHJuK1RreU5PNVhRZGRpYWhvRnEKN1ROL0h4amFJcVRpYWNPWVpKOFFBaXJIMFJ1TUlZaGpiTnAvOE1sRCtlSWEyN2FnRS9JVzFPL1VsRktnaENCam1SaXVWRmM5OHVxUgpMYzBvQW1jVVFpbElLVUZESVNHVWdzNFk3dGw3NEw3REovT3I5V1FDZ0hySDN2OG5NQ25Id1d2RjBzck5CL3J1ajRTRUY4ZW9SeEdvCmdzSVUyOFJ6eDBmVzdqMHhlaU1TRHVpN2V2eWJnZGlPamFQRjBuVzdSOGMrMEdZWUxhWUFvTkJza2o4ZUhmb2xPQWVsL3k4SUxXT0UKQXB6amhSTWpQeCt0MTFrVUM5QTJydUdaNFpFYnFyVjZGcFk1bWQ2RXRKSXJFQUwxS0VJdGpOQ01Zd2lsSnN2NVZDTm9KYU1YeDZpSApJZHd3UkNCRXErOFE4am9iaG82RzUyVmVPVGwyTXlVS2ZLanVZbnVoK0NWb1dnc0FJU0NFd0l0aU1CQ2tEZU9sYnR2YVkxQWFGSDEvClJqMElMM2FqT0dseENwMVFTS1ZBQ0lHUUVuNGN3NkRFN1RETlo2ZFkxbUFraEo1dk5zOHNldjR5Q1FtTGE2MUVWd3JnR2dacTlTL2sKa29rZjhQNWFiVm0rNGZYQzBERkJnZTgyd0Exait3WFRzbCt4R0h1MjB6UmhNSVo2R0tMaUIrbG1GTjIwdDFDOFBaQXgwb2FPV2hBQwpTbUYrSnIzUll2emVUdHNxdFdrYVFpRlFEaHo0c2Jodzc4bXg3K1k5YjRXajYxQUFOSTJoRm9UVGo5ZmNsWHlvN2w0T0tRQndnQkJJCno4ZjBWT292bDg2WXZqNXRHdWdyVjNDb1VybEVLWlhPV3RhdWhNYUhNN3IrSFpPeEovNVJLRHd4VW5kVEdjdHk1NlNTYTNvU2llMk0KRU5TQ01EZFFxU3lsaEZUYkRmMlp4Vk9udkdCeHZ2TEpvMGYvNkFiQmgyekxCQk1TVWF2ZHIrUEZJRHgzTXFoQmdMU2huL3paNmxYcgp1OXNTK1BxTE95L2ZPbnpzSGtyWmJBN2cxWElWV2N2OGhVYnBEY3U3c2pzWHRLZlcvbWJmd1VkWFQ4OTkyTmI0OXI1U21kV0RZTk5RCnRYWjlwZW5CMGpSd1NvWTB5ajUzOVlKNWY3NXM1bWtiN3R5eDg5aXhhaTJuR3daQUNKcHh2SlNXdzdBWGpMVkFSQkhXOTg2NlU5TVkKTmg5NjlheS9EQXo5MVkvRmJDOFc4SVNFRzBjWXJMbWZIS2pXWHRFcE1WZmxjanUrdU96Y3hldDdaejNiazBobysvT0ZsL2ZtQzlkNwpVWXhMcHZjOGJHdWFxQWZCak1lT0RqNXlJRjlZdXJpekV4ZjE5Tnd1NDdoVmdaU2lHY2U5VEZ5Ky9odUNFQWZqSDgrYTJuRUxDQzMrCi9PQ2hUY1ZhZmNHTVREby9MNTA2MG1sYnBXbU9mVklwSkl1K1B3MUtMVi9VMGI0NVplZzFpek8rdWUvd0k0ZUxwWXU2RTBsL2NVZjcKampXelpxNmxoUEJhRkMydU54dFdPUWlueDByOTluQ3BYQm9OZzF2RVJFSURra1pLNlJNMVJ4akRubUs1TkZDcklRUTVIVXFoTjVtOApiKzMwbmlYTHNsTVhYVEZ6eHVKdTIzNVpaeHc3VDQ2dC91dmcwTkptRk9PcG9XTkx0ZzJmdUp3eGhubnQ2VmV1bkR0blpTVUlzTFFyCnUzRnVPdlZUS0NCVWFuN0JhNklwNHFKT21aUnFvcVNoY1kzUU1KU3lWY05DWUc2cUxUT3JyUzJ2QTMxZ2RONitTdVV6dXdyRk5RcEsKTnlnVk9xRUxoUlJZbXUxOCt0S2UzRXNkbG9tcjU4L1pjN0JZZW16cjhMRjFmYVh5bVlPVjZ1TXJjdDN2ZTJTay8ydDV6L3NFR0lWRwpjSGhhSW9HaGFqMFRTa0hwZUIraVFFU1RuQlVoUktzN2hTR0VsT3R5Q1J0WDk4N2NxT2tHOG8xbWR5VGxjcWx3YmpPS2x1VTlMMEdBCmd4ZDBaYStZMFpZTWp6V2FXYUVRdmVlMDNBY3R6dmNXUGMrcGhzR2Eva3IxdDI0VWZidnErMTA2NTdoeVR1OXQ1MCtiaG5iRFdPczMKUEREU0dtQTZKVFhhWVJoSEVJdFdQRGpIMDhQSHZ1cFFobytmUG4vUFZiMHozMjlTT3FqUWFqQ2NVT1FjNTVlejA2bXpEYzc5WFdQRgo4Mi9iK3Z5K2gxNDljbkVsaU1JenBrNDVaMEdtL1g0QTJERXljazB0Q0xuTitmQ0crWE92V3B6dGZLa3ZYOERPa2RGYjZiZzJnVlF3Ckdlc25GLzcrVDk5ODRmaklOMkNacmNZYmhzZ2xFbzllTUszckE1MjJoYjV5QlZMSVZRU3FQWkx5NVM3TGZpMnBhVGpaYUp5M3IxQjgKWXFSZVQzVTVqdHViYmxzenJ6MjEzWThFaHFyMUhLYzRUMHBaQlNGYnpzNTI0a0MrZ0tjR0JoOVVsRzR3ZFIxQ1NnUnhqQzdidklPdAp1TzU2YjMrNSt1bUpNcVdhaHByYm1OL3Z1cGZwakIxaWhMeVdjNXpCbWNsa1h5eFYxUmNpVmZDOXp6OS9ZdlNCVUFnejZ6aW9oYUUrClhIYy83WEF1WWluMzJ4b2ZXOURSM21kemJUQ1NFaWRjZC9tK3NjTHY2bkY4bWFNYmtKQ1FVQkFLeUZybXJXVGppenZ3NHdPSCs4ZnEKN213WU9pZ2gwQmxGR0F0d1F0Q3U2eTlsVEdPM3paaFhEOFBlb3UrdjlLTzR6ZUVNR3FHUVVyWktUUWcwb3dnbW8yNjdZV3hMRy9xUgpNQlpteWZlWEZEenZ2RmhLMkpxR1dFb0lLUGhDd0dicytQeGtvb2ZQYWt2aXdzNk83ejFjTFAwSXBqRTU2U3lOZ1NpQ2FoUXVMUWZCClVxVUFuUUltWlVocUdnQTFxYm9tMUZOUzB4QkprVGpaYUs1N3JWNEhrUXFNVWhpTXcrUUtzVlNUMDFaSmhRNWIvMzVDWTZCdUZPTjkKcC9Wc1NxZmE4dkM4VnBWZ2ZLQ2lKZnNTbW9ZMlhZUE5PUmdoYjZzM0ZBQkdDQ3pPMGFiclNPZzZUTTVhc2tDOURzQVhFaGJYS2puVAorR0VrSktnaVFLZmp5SS9PbVhrdEl0R2k5LzhvYWlScGFZN1RMT05URHFVeFVRcFVTb1VSdDRGTGN0TWVPNnVuK3llb3UvaWZZUkQ4ClMzbE9BRFJpZ2FtRy9xdXN6aDl5b3doQ1NsQ2JNK2lNSUZZU1h6NzdqQnNXZEhVK0U5YnFMWEh6TGpKQUFOU0Z3QlJOZTNHQmJWeXIKbEFTQkFvTUNwYVFWeDBBS0dCckhMV2Vkc1hwaDU1U25tbzBtd3ZHRWV6ZEljWVZFMXRDM0xYU3NTeGp3SnBsSUZWNi9leFk4SDVSQQpYcmR3M25zWGRXUTIrVkdFU2hDQy9KZXNFQUNlbEFpa3hGUmR1MzlSd3I2WUVVUytQT1VHOXNZWFJnamNPRUlwQ0hGUmQ5ZU5INXMxCjR5TmRodEZmOUh6VW9uaGNzT0xmQXBwWUM2U0VLd1FjUmdjWE90WTEwMDNqZWw4cWhQS3RRcG0vZFJPQ1VFcVVnZ0NucDlyK2tMUE4KaDNZWHlqY2ZjZDFQRm9QZ1RESHVoVVlJekhHR2xBSWlLU0ZrSzg0RUJDbU43MjluN05kWm5kOU5nR0EwRE9FUThtYXYzdzdFRzcwcApoeUVnWmJ4c1NzZGRzeFAyWFFQMXhzcEtHSzV1eHVKY1Q0aDVmaHhub0JRSFFleHdYallvT2N5QlhSYWxUMmMwdGxVSFd2ODQvc05OCjVwODF0VFZQUUEzVFV3QUFBQ1YwUlZoMFpHRjBaVHBqY21WaGRHVUFNakF5TVMweE1TMHlNbFF3Tnpvd09Ub3pOeXN3TURvd01ERGoKR0hjQUFBQWxkRVZZZEdSaGRHVTZiVzlrYVdaNUFESXdNakV0TVRFdE1qSlVNRGM2TURrNk16Y3JNREE2TURCQnZxRExBQUFBRTNSRgpXSFJrWXpwbWIzSnRZWFFBYVcxaFoyVXZjRzVuLzdrYlBnQUFBQlYwUlZoMGNHaHZkRzl6YUc5d09rTnZiRzl5VFc5a1pRQXpWZ0t6ClFBQUFBQjUwUlZoMGRHbG1aanBZVW1WemIyeDFkR2x2YmdBeE1EQXdNREF3THpFd01EQXdXOTRXOHdBQUFCNTBSVmgwZEdsbVpqcFoKVW1WemIyeDFkR2x2YmdBeE1EQXdNREF3THpFd01EQXcvbFdHL1FBQUFCUjBSVmgwZUcxd09rTnZiRzl5VTNCaFkyVUFOalUxTXpVNwpWRTN5QUFBQUtIUkZXSFI0YlhBNlEzSmxZWFJsUkdGMFpRQXlNREl4TFRFeExUSXlWREV5T2pNMk9qUTBLekExT2pNd05zME52Z0FBCkFETjBSVmgwZUcxd09rTnlaV0YwYjNKVWIyOXNBRUZrYjJKbElGQm9iM1J2YzJodmNDQkRReUF5TURFMUxqVWdLRmRwYm1SdmQzTXAKN3lZdlR3QUFBQ3AwUlZoMGVHMXdPazFsZEdGa1lYUmhSR0YwWlFBeU1ESXhMVEV4TFRJeVZERXlPak01T2pFekt6QTFPak13d3RzSgpYZ0FBQUNoMFJWaDBlRzF3T2sxdlpHbG1lVVJoZEdVQU1qQXlNUzB4TVMweU1sUXhNam96T1RveE15c3dOVG96TVA1L1d1QUFBQUFXCmRFVllkSGh0Y0RwUWFYaGxiRmhFYVcxbGJuTnBiMjRBTXpNcU1xZVZBQUFBRm5SRldIUjRiWEE2VUdsNFpXeFpSR2x0Wlc1emFXOXUKQURNejk2UitFQUFBQUQxMFJWaDBlRzF3VFUwNlJHOWpkVzFsYm5SSlJBQjRiWEF1Wkdsa09qQTNZamN6TXprM0xUTTNOVEF0T0dJMApOeTFoTWpVMExXUmhaakpqTVRBNVkySmpOWTZSUDg4QUFBQTlkRVZZZEhodGNFMU5Pa2x1YzNSaGJtTmxTVVFBZUcxd0xtbHBaRG93Ck4ySTNNek01Tnkwek56VXdMVGhpTkRjdFlUSTFOQzFrWVdZeVl6RXdPV05pWXpYNHRxakZBQUFBUlhSRldIUjRiWEJOVFRwUGNtbG4KYVc1aGJFUnZZM1Z0Wlc1MFNVUUFlRzF3TG1ScFpEb3dOMkkzTXpNNU55MHpOelV3TFRoaU5EY3RZVEkxTkMxa1lXWXlZekV3T1dOaQpZeldDVEZFOEFBQUFBRWxGVGtTdVFtQ0MiIC8+Cjwvc3ZnPgo=";

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
    D14: '14',
    D15: '15',
    D16: '16',
    D17: '17',
    D18: '18',
    D19: '19',
    D20: '20',
    D21: '21',
    D22: '22',
    D23: '23',
    D24: '24',
    D25: '25',
    D26: '26',
    D27: '27',
    D28: '28',
    D29: '29',
    D30: '30',
    D31: '31',
    D32: '32',
    D33: '33',
    D34: '34',
    D35: '35',
    D36: '36',
    D37: '37',
    D38: '38',
    D39: '39',
    D40: '40',
    D41: '41',
    D42: '42',
    D43: '43',
    D44: '44',
    D45: '45',
    D46: '46',
    D47: '47',
    D48: '48',
    D49: '49',
    D50: '50',
    D51: '51',
    D52: '52',
    D53: '53',
    A0: 'A0',
    A1: 'A1',
    A2: 'A2',
    A3: 'A3',
    A4: 'A4',
    A5: 'A5',
    A6: 'A6',
    A7: 'A7',
    A8: 'A8',
    A9: 'A9',
    A10: 'A10',
    A11: 'A11',
    A12: 'A12',
    A13: 'A13',
    A14: 'A14',
    A15: 'A15'
};


const Level = {
    High: 'HIGH',
    Low: 'LOW'
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

const SerialNo = {
    Serial0: '0',
    Serial1: '1',
    Serial2: '2',
    Serial3: '3'
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
 * Manage communication with a Arduino Mega2560 peripheral over a OpenBlock Link client socket.
 */
class ArduinoMega2560 extends ArduinoPeripheral{
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
 * OpenBlock blocks to interact with a Arduino Mega2560 peripheral.
 */
class OpenBlockArduinoMega2560Device {
    /**
     * @return {string} - the ID of this extension.
     */
    static get DEVICE_ID () {
        return 'arduinoMega2560';
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
                text: '14',
                value: Pins.D14
            },
            {
                text: '15',
                value: Pins.D15
            },
            {
                text: '16',
                value: Pins.D16
            },
            {
                text: '17',
                value: Pins.D17
            },
            {
                text: '18',
                value: Pins.D18
            },
            {
                text: '19',
                value: Pins.D19
            },
            {
                text: '20',
                value: Pins.D20
            },
            {
                text: '21',
                value: Pins.D21
            },
            {
                text: '22',
                value: Pins.D22
            },
            {
                text: '23',
                value: Pins.D23
            },
            {
                text: '24',
                value: Pins.D24
            },
            {
                text: '25',
                value: Pins.D25
            },
            {
                text: '26',
                value: Pins.D26
            },
            {
                text: '27',
                value: Pins.D27
            },
            {
                text: '28',
                value: Pins.D28
            },
            {
                text: '29',
                value: Pins.D29
            },
            {
                text: '30',
                value: Pins.D30
            },
            {
                text: '31',
                value: Pins.D31
            },
            {
                text: '32',
                value: Pins.D32
            },
            {
                text: '33',
                value: Pins.D33
            },
            {
                text: '34',
                value: Pins.D34
            },
            {
                text: '35',
                value: Pins.D35
            },
            {
                text: '36',
                value: Pins.D36
            },
            {
                text: '37',
                value: Pins.D37
            },
            {
                text: '38',
                value: Pins.D38
            },
            {
                text: '39',
                value: Pins.D39
            },
            {
                text: '40',
                value: Pins.D40
            },
            {
                text: '41',
                value: Pins.D41
            },
            {
                text: '42',
                value: Pins.D42
            },
            {
                text: '43',
                value: Pins.D43
            },
            {
                text: '44',
                value: Pins.D44
            },
            {
                text: '45',
                value: Pins.D45
            },
            {
                text: '46',
                value: Pins.D46
            },
            {
                text: '47',
                value: Pins.D47
            },
            {
                text: '48',
                value: Pins.D48
            },
            {
                text: '49',
                value: Pins.D49
            },
            {
                text: '50',
                value: Pins.D50
            },
            {
                text: '51',
                value: Pins.D51
            },
            {
                text: '52',
                value: Pins.D52
            },
            {
                text: '53',
                value: Pins.D53
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
            },
            {
                text: 'A6',
                value: Pins.A6
            },
            {
                text: 'A7',
                value: Pins.A7
            },
            {
                text: 'A8',
                value: Pins.A8
            },
            {
                text: 'A9',
                value: Pins.A9
            },
            {
                text: 'A10',
                value: Pins.A10
            },
            {
                text: 'A11',
                value: Pins.A11
            },
            {
                text: 'A12',
                value: Pins.A12
            },
            {
                text: 'A13',
                value: Pins.A13
            },
            {
                text: 'A14',
                value: Pins.A14
            },
            {
                text: 'A15',
                value: Pins.A15
            }
        ];
    }

    get MODE_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'arduinoMega2560.modeMenu.input',
                    default: 'input',
                    description: 'label for input pin mode'
                }),
                value: Mode.Input
            },
            {
                text: formatMessage({
                    id: 'arduinoMega2560.modeMenu.output',
                    default: 'output',
                    description: 'label for output pin mode'
                }),
                value: Mode.Output
            },
            {
                text: formatMessage({
                    id: 'arduinoMega2560.modeMenu.inputPullup',
                    default: 'input-pullup',
                    description: 'label for input-pullup pin mode'
                }),
                value: Mode.InputPullup
            }
        ];
    }

    get DIGITAL_PINS_MENU () {
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
                text: '14',
                value: Pins.D14
            },
            {
                text: '15',
                value: Pins.D15
            },
            {
                text: '16',
                value: Pins.D16
            },
            {
                text: '17',
                value: Pins.D17
            },
            {
                text: '18',
                value: Pins.D18
            },
            {
                text: '19',
                value: Pins.D19
            },
            {
                text: '20',
                value: Pins.D20
            },
            {
                text: '21',
                value: Pins.D21
            },
            {
                text: '22',
                value: Pins.D22
            },
            {
                text: '23',
                value: Pins.D23
            },
            {
                text: '24',
                value: Pins.D24
            },
            {
                text: '25',
                value: Pins.D25
            },
            {
                text: '26',
                value: Pins.D26
            },
            {
                text: '27',
                value: Pins.D27
            },
            {
                text: '28',
                value: Pins.D28
            },
            {
                text: '29',
                value: Pins.D29
            },
            {
                text: '30',
                value: Pins.D30
            },
            {
                text: '31',
                value: Pins.D31
            },
            {
                text: '32',
                value: Pins.D32
            },
            {
                text: '33',
                value: Pins.D33
            },
            {
                text: '34',
                value: Pins.D34
            },
            {
                text: '35',
                value: Pins.D35
            },
            {
                text: '36',
                value: Pins.D36
            },
            {
                text: '37',
                value: Pins.D37
            },
            {
                text: '38',
                value: Pins.D38
            },
            {
                text: '39',
                value: Pins.D39
            },
            {
                text: '40',
                value: Pins.D40
            },
            {
                text: '41',
                value: Pins.D41
            },
            {
                text: '42',
                value: Pins.D42
            },
            {
                text: '43',
                value: Pins.D43
            },
            {
                text: '44',
                value: Pins.D44
            },
            {
                text: '45',
                value: Pins.D45
            },
            {
                text: '46',
                value: Pins.D46
            },
            {
                text: '47',
                value: Pins.D47
            },
            {
                text: '48',
                value: Pins.D48
            },
            {
                text: '49',
                value: Pins.D49
            },
            {
                text: '50',
                value: Pins.D50
            },
            {
                text: '51',
                value: Pins.D51
            },
            {
                text: '52',
                value: Pins.D52
            },
            {
                text: '53',
                value: Pins.D53
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
            },
            {
                text: 'A6',
                value: Pins.A6
            },
            {
                text: 'A7',
                value: Pins.A7
            },
            {
                text: 'A8',
                value: Pins.A8
            },
            {
                text: 'A9',
                value: Pins.A9
            },
            {
                text: 'A10',
                value: Pins.A10
            },
            {
                text: 'A11',
                value: Pins.A11
            },
            {
                text: 'A12',
                value: Pins.A12
            },
            {
                text: 'A13',
                value: Pins.A13
            },
            {
                text: 'A14',
                value: Pins.A14
            },
            {
                text: 'A15',
                value: Pins.A15
            }
        ];
    }

    get LEVEL_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'arduinoMega2560.levelMenu.high',
                    default: 'high',
                    description: 'label for high level'
                }),
                value: Level.High
            },
            {
                text: formatMessage({
                    id: 'arduinoMega2560.levelMenu.low',
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
                text: '44',
                value: Pins.D44
            },
            {
                text: '45',
                value: Pins.D45
            },
            {
                text: '46',
                value: Pins.D46
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
            },
            {
                text: '18',
                value: Pins.D18
            },
            {
                text: '19',
                value: Pins.D19
            },
            {
                text: '20',
                value: Pins.D20
            },
            {
                text: '21',
                value: Pins.D21
            }
        ];
    }

    get INTERRUP_MODE_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'arduinoUno.InterrupModeMenu.risingEdge',
                    default: 'rising edge',
                    description: 'label for rising edge interrup'
                }),
                value: InterrupMode.Rising
            },
            {
                text: formatMessage({
                    id: 'arduinoUno.InterrupModeMenu.fallingEdge',
                    default: 'falling edge',
                    description: 'label for falling edge interrup'
                }),
                value: InterrupMode.Falling
            },
            {
                text: formatMessage({
                    id: 'arduinoUno.InterrupModeMenu.changeEdge',
                    default: 'change edge',
                    description: 'label for change edge interrup'
                }),
                value: InterrupMode.Change
            },
            {
                text: formatMessage({
                    id: 'arduinoUno.InterrupModeMenu.low',
                    default: 'low',
                    description: 'label for low interrup'
                }),
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

    get SERIAL_NO_MENU () {
        return [
            {
                text: '0',
                value: SerialNo.Serial0
            },
            {
                text: '1',
                value: SerialNo.Serial1
            },
            {
                text: '2',
                value: SerialNo.Serial2
            },
            {
                text: '3',
                value: SerialNo.Serial3
            }
        ];
    }

    get EOL_MENU () {
        return [
            {
                text: formatMessage({
                    id: 'arduinoUno.eolMenu.warp',
                    default: 'warp',
                    description: 'label for warp print'
                }),
                value: Eol.Warp
            },
            {
                text: formatMessage({
                    id: 'arduinoUno.eolMenu.noWarp',
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
                    id: 'arduinoMega2560.dataTypeMenu.integer',
                    default: 'integer',
                    description: 'label for integer'
                }),
                value: DataType.Integer
            },
            {
                text: formatMessage({
                    id: 'arduinoMega2560.dataTypeMenu.decimal',
                    default: 'decimal',
                    description: 'label for decimal number'
                }),
                value: DataType.Decimal
            },
            {
                text: formatMessage({
                    id: 'arduinoMega2560.dataTypeMenu.string',
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

        // Create a new Arduino mega 2560 peripheral instance
        this._peripheral = new ArduinoMega2560(this.runtime,
            OpenBlockArduinoMega2560Device.DEVICE_ID, originalDeviceId);
    }

    /**
     * @returns {Array.<object>} metadata for this extension and its blocks.
     */
    getInfo () {
        return [
			{
				id: 'Mega2560',
				name: formatMessage({
                    id: 'arduinoMega2560.category.mega2560',
                    default: 'Mega2560',
                    description: 'The name of the arduino mega2560 device'
                }),
				menuIconURI: menuIconURI,
                color1: '#FFBF00',
                color2: '#E6AC00',
                color3: '#CC9900',
				blocks: [
                    {
                        opcode: 'Mega2560header',
                        text: formatMessage({
                            id: 'arduinoMega2560.mega2560.header',
                            default: 'when Arduino Mega2560 begins',
                            description: 'arduinoMega2560 header'
                        }),
                        blockType: BlockType.HAT,
						programMode: [ProgramModeType.UPLOAD]
                    }]
			},
            {
                id: 'pin',
                name: formatMessage({
                    id: 'arduinoMega2560.category.pins',
                    default: 'Pins',
                    description: 'The name of the arduino mega2560 device pin category'
                }),
                color1: '#4C97FF',
                color2: '#3373CC',
                color3: '#3373CC',

                blocks: [
                    {
                        opcode: 'setPinMode',
                        text: formatMessage({
                            id: 'arduinoMega2560.pins.setPinMode',
                            default: 'set pin [PIN] mode [MODE]',
                            description: 'arduinoMega2560 set pin mode'
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
                            id: 'arduinoMega2560.pins.setDigitalOutput',
                            default: 'set digital pin [PIN] out [LEVEL]',
                            description: 'arduinoMega2560 set digital pin out'
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
                            id: 'arduinoMega2560.pins.setPwmOutput',
                            default: 'set pwm pin [PIN] out [OUT]',
                            description: 'arduinoMega2560 set pwm pin out'
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
                            id: 'arduinoMega2560.pins.readDigitalPin',
                            default: 'read digital pin [PIN]',
                            description: 'arduinoMega2560 read digital pin'
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
                            id: 'arduinoMega2560.pins.readAnalogPin',
                            default: 'read analog pin [PIN]',
                            description: 'arduinoMega2560 read analog pin'
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
                            id: 'arduinoMega2560.pins.setServoOutput',
                            default: 'set servo pin [PIN] out [OUT]',
                            description: 'arduinoMega2560 set servo pin out'
                        }),
                        blockType: BlockType.COMMAND,
                        arguments: {
                            PIN: {
                                type: ArgumentType.STRING,
                                menu: 'pins',
                                defaultValue: Pins.D9
                            },
                            OUT: {
                                type: ArgumentType.ANGLE,
                                defaultValue: '90'
                            }
                        }
                    },
                    '---',
                    {

                        opcode: 'attachInterrupt',
                        text: formatMessage({
                            id: 'arduinoMega2560.pins.attachInterrupt',
                            default: 'attach interrupt pin [PIN] mode [MODE] executes',
                            description: 'arduinoMega2560 attach interrupt'
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
                            id: 'arduinoMega2560.pins.detachInterrupt',
                            default: 'detach interrupt pin [PIN]',
                            description: 'arduinoMega2560 detach interrupt'
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
                    id: 'arduinoMega2560.category.serial',
                    default: 'Serial',
                    description: 'The name of the arduino mega2560 device serial category'
                }),
                color1: '#9966FF',
                color2: '#774DCB',
                color3: '#774DCB',

                blocks: [
                    {
                        opcode: 'multiSerialBegin',
                        text: formatMessage({
                            id: 'arduinoMega2560.serial.multiSerialBegin',
                            default: 'serial [NO] begin baudrate [VALUE]',
                            description: 'arduinoMega2560 multi serial begin'
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
                                defaultValue: Buadrate.B9600
                            }
                        },
                        programMode: [ProgramModeType.UPLOAD]
                    },
					{
                        opcode: 'multiSerialTimeout',
                        text: formatMessage({
                            id: 'arduinoMega2560.serial.multiSerialTimeout',
                            default: 'serial [NO] set timeout [ARG1]',
                            description: 'arduinoMega2560 multi serial begin'
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
                            id: 'arduinoMega2560.serial.multiSerialPrint',
                            default: 'serial [NO] print [VALUE] [EOL]',
                            description: 'arduinoMega2560 multi serial print'
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
                            id: 'arduinoMega2560.serial.multiSerialAvailable',
                            default: 'serial [NO] available data length',
                            description: 'arduinoMega2560 multi serial available data length'
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
                            id: 'arduinoMega2560.serial.multiSerialReadAByte',
                            default: 'serial [NO] read a byte',
                            description: 'arduinoMega2560 multi serial read a byte'
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
                            id: 'arduinoMega2560.serial.multiSerialReadInteger',
                            default: 'serial [NO] read integer',
                            description: 'arduinoMega2560 multi serial read integer'
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
                            id: 'arduinoMega2560.serial.multiSerialReadString',
                            default: 'serial [NO] read String',
                            description: 'arduinoMega2560 multi serial read string'
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
                            id: 'arduinoMega2560.serial.multiSerialCompareString',
                            default: 'serial [NO] Compare String [VALUE]',
                            description: 'arduinoMega2560 multi serial compare string'
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
                id: 'data',
                name: formatMessage({
                    id: 'arduinoMega2560.category.data',
                    default: 'Data',
                    description: 'The name of the arduino mega2560 device data category'
                }),
                color1: '#CF63CF',
                color2: '#C94FC9',
                color3: '#BD42BD',
                blocks: [
                    {
                        opcode: 'dataMap',
                        text: formatMessage({
                            id: 'arduinoMega2560.data.dataMap',
                            default: 'map [DATA] from ([ARG0], [ARG1]) to ([ARG2], [ARG3])',
                            description: 'arduinoMega2560 data map'
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
                            id: 'arduinoMega2560.data.dataConstrain',
                            default: 'constrain [DATA] between ([ARG0], [ARG1])',
                            description: 'arduinoMega2560 data constrain'
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
                            id: 'arduinoMega2560.data.dataConvert',
                            default: 'convert [DATA] to [TYPE]',
                            description: 'arduinoMega2560 data convert'
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
                            id: 'arduinoMega2560.data.dataConvertASCIICharacter',
                            default: 'convert [DATA] to ASCII character',
                            description: 'arduinoMega2560 data convert to ASCII character'
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
                            id: 'arduinoMega2560.data.dataConvertASCIINumber',
                            default: 'convert [DATA] to ASCII number',
                            description: 'arduinoMega2560 data convert to ASCII number'
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

module.exports = OpenBlockArduinoMega2560Device;

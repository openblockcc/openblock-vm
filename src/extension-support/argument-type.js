/**
 * Block argument types
 * @enum {string}
 */
const ArgumentType = {
    /**
     * Numeric value with angle picker
     */
    ANGLE: 'angle',

    /**
     * Boolean value with hexagonal placeholder
     */
    BOOLEAN: 'Boolean',

    /**
     * Numeric value with color picker
     */
    COLOR: 'color',

    /**
     * Numeric value with angle picker, value from 0 ~ 180
     */
    HALF_ANGLE: 'half_angle',

    /**
     * Inline image on block (as part of the label)
     */
    IMAGE: 'image',

    /**
     * Numeric value with text field
     */
    NUMBER: 'number',

    /**
     * Intger numeric value with text field
     */
    INTEGER_NUMBER: 'intger_number',

    /**
     * Positive numeric value with text field
     */
    POSITIVE_NUMBER: 'positive_number',

    /**
     * Whole numeric value with text field
     */
    WHOLE_NUMBER: 'whole_number',

    /**
     * Numeric value with slider field. from -127 to 127 integer value
     */
    INT8_NUMBER: 'int8_number',

    /**
     * Numeric value with slider field. from 0 to 255 integer value
     */
    UINT8_NUMBER: 'uint8_number',

    /**
     * Numeric value with slider field. from -255 to 255 integer value
     */
    INT9_NUMBER: 'int9_number',

    /**
     * Numeric value with slider field. from 0 to 1023 integer value
     */
    UINT10_NUMBER: 'uint10_number',

    /**
     * Numeric value with slider field. from -1023 to 1023 integer value
     */
    INT11_NUMBER: 'int11_number',

    /**
     * Numeric value with slider field. from 0 to 65535 integer value
     */
    UINT16_NUMBER: 'uint16_number',

    /**
     * Numeric value with slider field. from 0 to 100 value integer value
     */
    INTOTO100_NUMBER: 'int0to100_number',

    /**
     * Numeric value with slider field. from 0 to 100 value
     */
    OTO100_NUMBER: '0to100_number',

    /**
     * Numeric value with slider field. from -100 to 100 value integer value
     */
    INTN100TO100_NUMBER: 'intn100to100_number',

    /**
     * Numeric value with slider field. from -100 to 100 value
     */
    N100TO100_NUMBER: 'n100to100_number',

    /**
     * String value with text field
     */
    STRING: 'string',

    /**
     * String value with matrix field
     */
    MATRIX: 'matrix',

    /**
     * String value with matrix field
     */
    MATRIX8X12: 'matrix8x12',

    /**
     * MIDI note number with note picker (piano) field
     */
    NOTE: 'note'

};

module.exports = ArgumentType;

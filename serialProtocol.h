/* 
 * carduinodroid serial Protocol
 * 
 * protocol version  : 2.0
 * author            : Till Max Schwikal
 * date              : 11.12.2015
 * url               : https://github.com/tuiSSE/carduinodroid-wiki/wiki/Serial-Protocol
 */

#define STARTBYTE              0x80
#define VERSION                2
#define SEND_LENGTH            7
#define RECEIVE_LENGTH         3

#define STATUS_LED_BIT         0
#define FRONT_LIGHT_BIT        1
#define RESET_ACC_CURRENT_BIT  4
#define FAILSAFE_STOP_BIT      5

#define NUM_START              0
#define NUM_VERSION_LENGTH     1
#define NUM_SPEED              2
#define NUM_STEER              3
#define NUM_STATUS             4
#define NUM_RECEIVE_CHECK      5

#define NUM_CURRENT            2
#define NUM_ACC_CURRENT        3
#define NUM_REL_ACC_CURRENT    4
#define NUM_VOLTAGE            5
#define NUM_TEMPERATURE        6
#define NUM_DISTANCE_FRONT     7
#define NUM_DISTANCE_BACK      8
#define NUM_SEND_CHECK         9

#define PARITY_BIT             7
#define PARITY_MSK             (1 << PARITY_BIT)
#define CHECK_MSK              0x01

#define SIGN_BIT               7
#define SIGN_MASK              (1 << SIGN_BIT)

#define BUFFER_LENGTH_OFFSET   3
#define SEND_BUFFER_LENGTH     (SEND_LENGTH + BUFFER_LENGTH_OFFSET)
#define RECEIVE_BUFFER_LENGTH  (RECEIVE_LENGTH + BUFFER_LENGTH_OFFSET)

#define VERSION_SHIFT          4
#define VERSION_MASK           0x70
#define LENGTH_SHIFT           0
#define LENGTH_MASK            0x0f

#define SEND_VERSION_LENGTH    (0x00 | ((VERSION << VERSION_SHIFT) &  VERSION_MASK) | ((SEND_LENGTH << LENGTH_SHIFT) & LENGTH_MASK))
#define RECEIVE_VERSION_LENGTH (0x00 | ((VERSION << VERSION_SHIFT) &  VERSION_MASK) | ((RECEIVE_LENGTH << LENGTH_SHIFT) & LENGTH_MASK))

#define STATUS_LED_MSK         (1 << STATUS_LED_BIT)
#define FRONT_LIGHT_MSK        (1 << FRONT_LIGHT_BIT)
#define RESET_ACC_CURRENT_MSK  (1 << RESET_ACC_CURRENT_BIT)
#define FAILSAFE_STOP_MSK      (1 << FAILSAFE_STOP_BIT)
#define PARITY_MSK             (1 << PARITY_BIT)

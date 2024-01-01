"""Send this code, run and watch the repl.
Then turn the wheel slowly to see the change"""
import board
import rp2pio
import array
import time
import busio
import adafruit_pioasm
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.GP21, board.GP20, frequency=800000)
bno = BNO08X_I2C(i2c)


program = """
; use the osr for count
; input pins c1 c2

    set y, 0            ; clear y
    mov osr, y          ; and clear osr
read:
    ; x will be the old value
    ; y the new values
    mov x, y            ; store old Y in x
    in null, 32         ; Clear ISR - using y
    in pins, 2          ; read two pins into y
    mov y, isr
    jmp x!=y, different ; Jump if its different
    jmp read            ; otherwise loop back to read

different:
    ; x has old value, y has new.
    ; extract the upper bit of X.
    in x, 31             ; get bit 31 - old p1 (remember which direction it came in)
    in null, 31         ; keep only 1 bit
    mov x, isr          ; put this back in x
    jmp !x, c1_old_zero

c1_old_not_zero:
    jmp pin, count_down
    jmp count_up

c1_old_zero:
    jmp pin, count_up
    ; fall through
count_up:
    ; for a clockwise move - we'll add 1 by inverting
    mov x, ~ osr        ; store inverted OSR on x
    jmp x--, fake       ; use jump to take off 1
fake:
    mov x, ~ x          ; invert back
    jmp send
count_down:
    ; for a clockwise move, just take one off
    mov x, osr          ; store osr in x
    jmp x--, send       ; dec and send
send:
    ; send x.
    mov isr, x          ; send it
    push noblock        ; put ISR into input FIFO
    mov osr, x          ; put X back in OSR
    jmp read            ; loop back
"""

assembled = adafruit_pioasm.assemble(program)

left_front_pins  = (board.GP14, board.GP15)
left_rear_pins   = (board.GP12, board.GP13) #
right_rear_pins  = (board.GP18, board.GP19) #
right_front_pins  = (board.GP16, board.GP17)

## set up a statemachine
left_front_enc = rp2pio.StateMachine(
    assembled, frequency=0, first_in_pin=left_front_pins[0], jmp_pin=left_front_pins[1], in_pin_count=2
)

left_rear_enc = rp2pio.StateMachine(
    assembled, frequency=0, first_in_pin=left_rear_pins[0], jmp_pin=left_rear_pins[1], in_pin_count=2
)

right_front_enc = rp2pio.StateMachine(
    assembled, frequency=0, first_in_pin=right_front_pins[0], jmp_pin=right_front_pins[1], in_pin_count=2
)

right_rear_enc = rp2pio.StateMachine(
    assembled, frequency=0, first_in_pin=right_rear_pins[0], jmp_pin=right_rear_pins[1], in_pin_count=2
)

buffer = array.array("i", [0])

left_front_data = 0
left_rear_data = 0
right_front_data = 0
right_rear_data = 0
          
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_GAME_ROTATION_VECTOR)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_STEP_COUNTER)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_STABILITY_CLASSIFIER)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACTIVITY_CLASSIFIER)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_SHAKE_DETECTOR)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_RAW_ACCELEROMETER)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_RAW_GYROSCOPE)
# bno.enable_feature(adafruit_bno08x.BNO_REPORT_RAW_MAGNETOMETER)


print("Rotation Vector Quaternion:")
while True:
#     (
#         linear_accel_x,
#         linear_accel_y,
#         linear_accel_z,
#     ) = bno.linear_acceleration  # pylint:disable=no-member
#     print(
#         "X: %0.6f  Y: %0.6f Z: %0.6f m/s^2"
#         % (linear_accel_x, linear_accel_y, linear_accel_z)
#     )
    
#     quat_i, quat_j, quat_k, quat_real = bno.quaternion
#     print(
#       "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real)
#     )
    
#     print("Geomagnetic Rotation Vector Quaternion:")
#     (
#         geo_quat_i,
#         geo_quat_j,
#         geo_quat_k,
#         geo_quat_real,
#     ) = bno.geomagnetic_quaternion  # pylint:disable=no-member
#     print(
#         "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f"
#         % (geo_quat_i, geo_quat_j, geo_quat_k, geo_quat_real)
#     )
    
        # read data from the fifo
    if left_front_enc.in_waiting:
        left_front_enc.readinto(buffer)
        left_front_data = buffer[0]
        print(left_front_data, left_rear_data, right_front_data, right_rear_data)
        
    if left_rear_enc.in_waiting:
        left_rear_enc.readinto(buffer)
        left_rear_data = buffer[0]
        print(left_front_data, left_rear_data, right_front_data, right_rear_data)
        
    if right_front_enc.in_waiting:
        right_front_enc.readinto(buffer)
        right_front_data = buffer[0]
        print(left_front_data, left_rear_data, right_front_data, right_rear_data)
        
    if right_rear_enc.in_waiting:
        right_rear_enc.readinto(buffer)
        right_rear_data = buffer[0]
        print(left_front_data, left_rear_data, right_front_data, right_rear_data)

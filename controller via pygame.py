import pygame
import time

# MAPPINGS 
# Axes
LX_AXIS = 0   # left stick X
LY_AXIS = 1   # left stick Y
RX_AXIS = 2   # right stick X
RY_AXIS = 3   # right stick Y

L2_AXIS = 4   # brake (L2 trigger)
R2_AXIS = 5   # throttle(R2 trigger)

# D-pad buttons
DPAD_UP_BTN    = 11
DPAD_DOWN_BTN  = 12
DPAD_LEFT_BTN  = 13
DPAD_RIGHT_BTN = 14

DPAD_HAT_INDEX = 0 


def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No controller detected.")
        return

    js = pygame.joystick.Joystick(0)
    js.init()

    print("Using:", js.get_name())
    print("Axes:", js.get_numaxes(), "Buttons:", js.get_numbuttons(), "Hats:", js.get_numhats())

    try:
        while True:
            pygame.event.pump()

            # STICKS 
            def axis(idx):
                return js.get_axis(idx) if js.get_numaxes() > idx else 0.0

            lx = axis(LX_AXIS)
            ly = axis(LY_AXIS)
            rx = axis(RX_AXIS)
            ry = axis(RY_AXIS)

            #  TRIGGERS (0..1) 
            def trigger_to_01(raw):
                # raw is -1 (released) to +1 (fully pressed)
                return max(0.0, min(1.0, (raw + 1.0) / 2.0))

            brake    = trigger_to_01(axis(L2_AXIS))  # L2
            throttle = trigger_to_01(axis(R2_AXIS))  # R2

            # FACE BUTTONS 
            # 0 = cross, 1 = circle, 2 = square, 3 = triangle
            cross    = js.get_button(0)
            circle   = js.get_button(1)
            square   = js.get_button(2)
            triangle = js.get_button(3)

            # D-PAD 
            dpad_x, dpad_y = 0, 0

            if DPAD_HAT_INDEX is not None and js.get_numhats() > DPAD_HAT_INDEX:
                hat_x, hat_y = js.get_hat(DPAD_HAT_INDEX)
                dpad_x, dpad_y = hat_x, hat_y

            # if hat didn't change or is always (0,0), we still want button-based D-pad
            # so we OR the button-based state on top
            if js.get_numbuttons() > max(DPAD_UP_BTN, DPAD_DOWN_BTN, DPAD_LEFT_BTN, DPAD_RIGHT_BTN):
                up    = js.get_button(DPAD_UP_BTN)
                down  = js.get_button(DPAD_DOWN_BTN)
                left  = js.get_button(DPAD_LEFT_BTN)
                right = js.get_button(DPAD_RIGHT_BTN)

                # cnvert buttons to -1/0/+1
                # (if hat works, this will override or reinforce it)
                if up and not down:
                    dpad_y = 1
                elif down and not up:
                    dpad_y = -1
                # if both or neither -> 0

                if right and not left:
                    dpad_x = 1
                elif left and not right:
                    dpad_x = -1
                # if both or neither -> 0

            # PRINT STATE 
            print(
                f"LX={lx:+.2f} LY={ly:+.2f}  "
                f"RX={rx:+.2f} RY={ry:+.2f}  "
                f"Throttle={throttle:.2f} Brake={brake:.2f}  "
                f"X={cross} O={circle} []={square} /\={triangle}  "
                f"DPAD=({dpad_x},{dpad_y})",
                end="\r",
                flush=True,
            )

            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()

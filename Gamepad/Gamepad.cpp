#include "Gamepad.hpp"

#define JOY_DEV "/dev/input/js0"

Gamepad::Gamepad()
    : joy_fd(-1), num_of_axis(0), num_of_buttons(0), mDeadBand(2000)
{
    joy_open = Open();
}

bool Gamepad::Open()
{
    if ((joy_fd = open(JOY_DEV, O_RDONLY)) < 0)
    {
        std::cerr << "[JOYSTICK] Console Device No Detected.\n";
        return 0;
    }
    else
    {
        ioctl(joy_fd, JSIOCGAXES, &num_of_axis);
        ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);
        ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);
        joy_button.resize(num_of_buttons, 0);
        joy_axis.resize(num_of_axis, 0);
        fcntl(joy_fd, F_SETFL, O_NONBLOCK);

        std::string joystickName(name_of_joystick);

        if (joystickName == "Microsoft Xbox Series S|X Controller" || joystickName == "Microsoft X-Box 360 pad" || joystickName == "Microsoft Xbox One X pad")
        {
            joy_type = XBOXCONTROLLER;
        }
        else if (joystickName == "DualSense Wireless Controller" || joystickName == "Wireless Controller")
        {
            joy_type = DUALSENSE;
        }
        else if (joystickName == "Sony Interactive Entertainment DualSense Wireless Controller")
        {
            joy_type = DUALSENSE_WIRE;
        }
        else if (joystickName == "Steam Deck")
        {
            joy_type = STEAMDECK;
        }
        else
        {
            std::cout << "[JOYSTICK] Unknown joystick: " << joystickName << std::endl;
            joy_type = UNDEFINED;
        }
        return 1;
    }
}

void Gamepad::Read()
{
    if (joy_open)
    {
        js_event js;
        read(joy_fd, &js, sizeof(js_event));
        switch (js.type & ~JS_EVENT_INIT)
        {
        case JS_EVENT_AXIS:
            if (((int)js.value > mDeadBand) || ((int)js.value < -mDeadBand))
            {
                int sign;
                if ((int)js.value > 0)
                {
                    sign = 1;
                }
                else
                {
                    sign = -1;
                }
                joy_axis[(int)js.number] = js.value - sign * mDeadBand;
            }
            else
            {
                joy_axis[(int)js.number] = 0;
            }
            break;
        case JS_EVENT_BUTTON:
            joy_button[(int)js.number] = js.value;
            break;
        }
        Transfer();
        usleep(100);

    }
}

void Gamepad::Transfer()
{
    switch (joy_type)
    {
    case XBOXCONTROLLER:
        mJoystickAxis[0] = joy_axis[0];                       // LeftStickX
        mJoystickAxis[1] = joy_axis[1];                       // LeftStickY
        mJoystickAxis[2] = joy_axis[2];                       // LeftTrigger
        mJoystickAxis[3] = joy_axis[3];                       // RightStickX
        mJoystickAxis[4] = joy_axis[4];                       // RightStickY
        mJoystickAxis[5] = joy_axis[5];                       // RightTrigger
        mJoystickAxis[6] = joy_axis[6];                       // DpadX
        mJoystickAxis[7] = joy_axis[7];                       // DpadY
        mJoystickButton[0] = joy_button[0];                   // FaceButtonA
        mJoystickButton[1] = joy_button[1];                   // FaceButtonB
        mJoystickButton[2] = joy_button[2];                   // FaceButtonY
        mJoystickButton[3] = joy_button[3];                   // FaceButtonX
        mJoystickButton[4] = joy_button[4];                   // LeftBumper
        mJoystickButton[5] = joy_button[5];                   // RightBumper
        mJoystickButton[6] = joy_button[6];                   // Back
        mJoystickButton[7] = joy_button[7];                   // Start
        mJoystickButton[8] = joy_button[8];                   // Guide
        mJoystickButton[9] = joy_button[9];                   // LeftStick
        mJoystickButton[10] = joy_button[10];                 // RightStick
        break;
    case STEAMDECK:
        mJoystickAxis[0] = joy_axis[0];                       // LeftStickX
        mJoystickAxis[1] = joy_axis[1];                       // LeftStickY
        mJoystickAxis[2] = joy_axis[9];                       // LeftTrigger
        mJoystickAxis[3] = joy_axis[2];                       // RightStickX
        mJoystickAxis[4] = joy_axis[3];                       // RightStickY
        mJoystickAxis[5] = joy_axis[8];                       // RightTrigger
        mJoystickAxis[6] = (joy_button[19] - joy_button[18]) * 30767;   // DpadX
        mJoystickAxis[7] = (joy_button[17] - joy_button[16]) * 30767;   // DpadY
        mJoystickButton[0] = joy_button[3];                   // FaceButtonA
        mJoystickButton[1] = joy_button[4];                   // FaceButtonB
        mJoystickButton[2] = joy_button[5];                   // FaceButtonY
        mJoystickButton[3] = joy_button[6];                   // FaceButtonX
        mJoystickButton[4] = joy_button[7];                   // LeftBumper
        mJoystickButton[5] = joy_button[8];                   // RightBumper
        mJoystickButton[6] = joy_button[11];                  // Back
        mJoystickButton[7] = joy_button[12];                  // Start
        mJoystickButton[8] = joy_button[8];;                // Guide
        mJoystickButton[9] = joy_button[14];;                 // LeftStick
        mJoystickButton[10] = joy_button[15];                 // RightStick
        break;
    case DUALSENSE:
        mJoystickAxis[0] = joy_axis[0];                       // LeftStickX
        mJoystickAxis[1] = joy_axis[1];                       // LeftStickY
        mJoystickAxis[2] = joy_axis[2];                       // LeftBack
        mJoystickAxis[3] = joy_axis[3];                       // RightStickX
        mJoystickAxis[4] = joy_axis[4];                       // RightStickY
        mJoystickAxis[5] = joy_axis[5];                       // RightTrigger
        mJoystickAxis[6] = joy_axis[6];                       // DpadX
        mJoystickAxis[7] = joy_axis[7];                       // DpadY
        mJoystickButton[0] = joy_button[0];                   // FaceButtonA
        mJoystickButton[1] = joy_button[1];                   // FaceButtonB
        mJoystickButton[2] = joy_button[3];                   // FaceButtonY
        mJoystickButton[3] = joy_button[2];                   // FaceButtonX
        mJoystickButton[4] = joy_button[4];                   // LeftBumper
        mJoystickButton[5] = joy_button[5];                   // RightBumper
        mJoystickButton[6] = joy_button[8];                   // Back
        mJoystickButton[7] = joy_button[9];                   // Start
        mJoystickButton[8] = joy_button[10];                  // Guide
        mJoystickButton[9] = joy_button[11];                  // LeftStick
        mJoystickButton[10] = joy_button[12];                 // RightStick
        break;
    case DUALSENSE_WIRE:
        mJoystickAxis[0] = joy_axis[0];                       // LeftStickX
        mJoystickAxis[1] = joy_axis[1];                       // LeftStickY
        mJoystickAxis[2] = joy_axis[2];                       // LeftBack
        mJoystickAxis[3] = joy_axis[3];                       // RightStickX
        mJoystickAxis[4] = joy_axis[4];                       // RightStickY
        mJoystickAxis[5] = joy_axis[5];                       // RightTrigger
        mJoystickAxis[6] = joy_axis[6];                       // DpadX
        mJoystickAxis[7] = joy_axis[7];                       // DpadY
        mJoystickButton[0] = joy_button[0];                   // FaceButtonA
        mJoystickButton[1] = joy_button[1];                   // FaceButtonB
        mJoystickButton[2] = joy_button[3];                   // FaceButtonY
        mJoystickButton[3] = joy_button[2];                   // FaceButtonX
        mJoystickButton[4] = joy_button[4];                   // LeftBumper
        mJoystickButton[5] = joy_button[5];                   // RightBumper
        mJoystickButton[6] = joy_button[8];                   // Back
        mJoystickButton[7] = joy_button[9];                   // Start
        mJoystickButton[8] = joy_button[10];                  // Guide
        mJoystickButton[9] = joy_button[11];                  // LeftStick
        mJoystickButton[10] = joy_button[12];                 // RightStick
        break;
    default:
        std::cout << "[JOYSTICK]Unknown Device. Please check\n";
        sleep(1);
    }
    mJoystickAxis[0] = setDeadBend(mJoystickAxis[0]);
    mJoystickAxis[1] = setDeadBend(mJoystickAxis[1]);
    mJoystickAxis[3] = setDeadBend(mJoystickAxis[3]);
    mJoystickAxis[4] = setDeadBend(mJoystickAxis[4]);
}

double Gamepad::setDeadBend(double joystick)
{
    double threshold = 500;
    double joyMax = 30767;
    double calc;
    if (joystick > threshold)
    {
        calc = (joystick - threshold) / (joyMax - threshold);
    }
    else if (joystick < -threshold)
    {
        calc = (joystick + threshold) / (joyMax - threshold);
    }
    else
    {
        calc = 0;
    }
    return calc * 30767;
}

void Gamepad::Close()
{
    close(joy_fd);
}

Gamepad::~Gamepad()
{
    Close();
}
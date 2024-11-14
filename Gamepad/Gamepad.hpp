#ifndef GAMEPAD_HPP
#define GAMEPAD_HPP

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

class Gamepad
{
public:
    static Gamepad* getInstance()
    {
        static Gamepad instance;
        return &instance;
    };
    Gamepad(const Gamepad&) = delete;
    Gamepad& operator=(const Gamepad&) = delete;
private:
    Gamepad();
    ~Gamepad();
    bool Open();
    double setDeadBend(double joystick);
    void Transfer();

    int joy_fd, num_of_axis, num_of_buttons;
    bool joy_open;
    int joy_type;
    int mDeadBand;

    std::vector<char> joy_button;
    std::vector<int> joy_axis;
public:
    void Close();
    void Read();
    char name_of_joystick[80];
    bool mJoystickButton[11];
    int mJoystickAxis[8];

private:
    enum JOYSTICK_TYPE
    {
        UNDEFINED = 0,
        XBOXCONTROLLER,
        DUALSENSE,
        DUALSENSE_WIRE,
        STEAMDECK
    };
};


#endif //GAMEPAD_HPP

import ddr
import numpy as np
import matplotlib.pyplot as plt


def test_DDR_xaxis_line_IK_and_back():
    bot = ddr.DDR(1, 1, 5)

    t = np.linspace(0, 20, 20001)
    t = t[1:]
    t_steps = np.linspace(20.0/20000, 20.0/20000, 20000)

    x = t * 1.0
    y = t * 0.0

    x_dot = np.linspace(1, 1, 20000)
    y_dot = t * 0.0

    x_dot_dot = t * 0
    y_dot_dot = t * 0

    phi_1, phi_2 = bot.IK(x_dot, y_dot, x_dot_dot, y_dot_dot)
    new_x, new_y, new_theta = bot.FK(phi_1, phi_2, t_steps)

    new_x = np.array(new_x)
    new_y = np.array(new_y)

    assert np.allclose(x, new_x) == True
    assert np.allclose(y, new_y) == True


def test_DDR_circle_IK_and_back():
    t = np.linspace(0, 40*np.pi, 10001)
    t = t[1:]
    t_steps = np.linspace(40.0*np.pi/10000, 40.0*np.pi/10000, 10000)

    x = 20*np.cos(t/20)
    y = 20*np.sin(t/20)

    x_dot = -np.sin(t/20)
    y_dot = np.cos(t/20)

    x_dot_dot = -np.cos(t/20)/20
    y_dot_dot = -np.sin(t/20)/20

#Actually run the kinematics
    bot = ddr.DDR(1, 1, 4, [20.0, 0, np.pi/2.0])

    phi_1, phi_2 = bot.IK(x_dot, y_dot, x_dot_dot, y_dot_dot)
    new_x, new_y, new_theta = bot.FK( phi_1, phi_2, t_steps )

    new_x = np.array(new_x)
    new_y = np.array(new_y)

    assert np.allclose(x, new_x, atol=0.01) == True
    assert np.allclose(y, new_y, atol=0.01) == True

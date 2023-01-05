// SPDX-License-Identifier: GPL-2.0
/**
 * nunchuk.c - Wii Nunchuck Joystick Driver
 * Copyright (c) 2022 Sebastian Popa <sebastian.popa@raptor-technologies.ro>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>

/* Nunchuk device */
#define NUNCHUCK_DEVICE             1
#define NUNCHUCK_START_BYTE         0x00

/* Nunchuk Registers */
#define NUNCHUK_REGISTER_SIZE       6u
#define NUNCHUK_JOYSTICK_X_REG      0x00
#define NUNCHUK_JOYSTICK_Y_REG      0x01
#define NUNCHUK_ACC_X_REG           0x02
#define NUNCHUK_ACC_Y_REG           0x03
#define NUNCHUK_ACC_Z_REG           0x04
#define NUNCHUK_BUTTON_PRESS_REG    0x05

/* Delays */
#define POLL_INTERVAL_MS             50u
#define CONSECUTIVE_WRITE_DELAY_US   1000u

/* Nunchuk device private structure*/
struct nunchuk_dev {
    struct i2c_client *i2c_client;
};

static bool is_zpressed(char reg_val)
{
    if (reg_val & (1 << 0))
        return false;

    return true;
}

static bool is_cpressed(char reg_val)
{
    if (reg_val & (1 << 1))
        return false;

    return true;
}

/**
 * Guaranteed to sleep between 10 and 20 ms
 * Such waiting time is needed to add time between the previous i2c 
 * operation and the next one
 */
void nunchuck_sleep(void)
{
    usleep_range(10000, 20000);
}

/**
 * Sends Start Byte (0x00) to start nunchuck device
 * returns 0 for success
 */
int nunchuck_start_measurement(struct i2c_client *client)
{
    int err;
    const char tx_buf[] = {NUNCHUCK_START_BYTE};
    
    err = i2c_master_send(client, tx_buf, ARRAY_SIZE(tx_buf));
    if (err < 0)
        return err;

    return 0;
}

int nunchuck_set_register(struct i2c_client *client, u8 reg, u8 val)
{
    int err;
    u8 tx_buff[2] = {reg, val};
    
    err = i2c_master_send(client, tx_buff, ARRAY_SIZE(tx_buff));
    if (err < 0)
        return err;

    return 0;
}

/**
 * Reads nunchuck registers: 
 * [0x00] : Joystick x
 * [0x01] : Joystick y
 * [0x02] : Accelerometer X
 * [0x03] : Accelerometer Y
 * [0x04] : Accelerometer Z
 * [0x05] : C-Z button press info
 */
static int nunchuck_read_registers(struct i2c_client *client, char *rx_buf, u32 buffer_size)
{
    int err;

    /* sleep required between previouus i2c operation and next one */
    nunchuck_sleep(); /* 10ms to 20ms */
    
    err = nunchuck_start_measurement(client);
    if (err)
        return err;

    /* sleep required between previouus i2c operation and next one */
    nunchuck_sleep(); /* 10ms to 20ms */

    err = i2c_master_recv(client, rx_buf, buffer_size);
    if (err < 0)
        return err;

    return 0;
}

/**
 * Nunchuck doesn't have interrupts to notify i2c master that its state has changed
 * This function is called to regulary poll its registers
 */
void nunchuk_poll(struct input_dev *input)
{
    int err;
    struct nunchuk_dev *nunchuk;
    char nunchuck_buff[NUNCHUK_REGISTER_SIZE];

    nunchuk = (struct nunchuk_dev *)input_get_drvdata(input);

    err = nunchuck_read_registers(nunchuk->i2c_client, nunchuck_buff, NUNCHUK_REGISTER_SIZE);
    if (err) {
        pr_err("Nunchuk reg read failed\n");
        return;
    }

    input_report_key(input, BTN_Z, is_zpressed(nunchuck_buff[NUNCHUK_BUTTON_PRESS_REG]));
    input_report_key(input, BTN_C, is_cpressed(nunchuck_buff[NUNCHUK_BUTTON_PRESS_REG]));
    input_report_abs(input, ABS_X, (int)nunchuck_buff[NUNCHUK_JOYSTICK_X_REG]);
    input_report_abs(input, ABS_Y, (int)nunchuck_buff[NUNCHUK_JOYSTICK_Y_REG]);
    input_sync(input);
}

/**
 * Initializes communication with Nunchuk device by sending a handshake signal over i2c.
 * Initializes register 0xf0 with value 0x55
 * Initializes register 0xfb with value 0x00
 * A delay of 1ms is required between consecutive i2c register reads.
 * 
 * returns 0 in case of success, negative value for error
 */
static int nunchuck_i2c_init(struct i2c_client *client)
{
    int err;

    err = nunchuck_set_register(client, 0xf0, 0x55);
    if (err) 
        return err;

    udelay(CONSECUTIVE_WRITE_DELAY_US);

    err = nunchuck_set_register(client, 0xfb, 0x00);
    if (err)
        return err;

    pr_info("Nunchuk init succesful\n");

    return 0;
}

static int nunchuk_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err;
    char nunchuck_buff[NUNCHUK_REGISTER_SIZE];
    struct input_dev *input;
    struct nunchuk_dev *nunchuk;

    /* Allocate memory for input device */
    input = devm_input_allocate_device(&client->dev);
    if (IS_ERR(input)) 
        return PTR_ERR(input);

    /* Alocate memory for nunchuck device structure */
    nunchuk = devm_kzalloc(&client->dev, sizeof(struct nunchuk_dev), GFP_KERNEL);
    if (!nunchuk)
        return -ENOMEM;
    
    /* Add i2c_client information to device */
    nunchuk->i2c_client = client;
    input_set_drvdata(input, nunchuk);

    /* Device init */
    input->name = "Wii Nunchuk";
    input->id.bustype = BUS_I2C;

    /* Nunchuk specific button init */
    set_bit(EV_KEY, input->evbit);
    set_bit(BTN_C, input->keybit);
    set_bit(BTN_Z, input->keybit);
    set_bit(ABS_X, input->absbit);
    set_bit(ABS_Y, input->absbit);
    input_set_abs_params(input, ABS_X, 30, 220, 4, 8);
    input_set_abs_params(input, ABS_Y, 40, 200, 4, 8);

    /* General joystick input device button init */
    set_bit(BTN_TL, input->keybit);
    set_bit(BTN_SELECT, input->keybit);
    set_bit(BTN_MODE, input->keybit);
    set_bit(BTN_START, input->keybit);
    set_bit(BTN_TR, input->keybit);
    set_bit(BTN_TL2, input->keybit);
    set_bit(BTN_B, input->keybit);
    set_bit(BTN_Y, input->keybit);
    set_bit(BTN_A, input->keybit);
    set_bit(BTN_X, input->keybit);
    set_bit(BTN_TR2, input->keybit);

    /* Setup polling */
    input_setup_polling(input, nunchuk_poll);
    input_set_poll_interval(input, POLL_INTERVAL_MS); /* 50 ms */

    /* Register input device */
    err = input_register_device(input);
    if (err) {
        pr_err("Input device registration failed\n");
        return err;
    }

    /* Init Nunchuck Device*/
    err = nunchuck_i2c_init(client); 
    if (err) {
        pr_err("Nunchuk i2c dev init failed\n");
        return -EIO;
    }

    /* Perform one register read to eliminate invalid data in registers */
    err = nunchuck_read_registers(client, nunchuck_buff, NUNCHUK_REGISTER_SIZE);
    if (err) {
        pr_err("Nunchuk device registers read failed\n");
        return err;
    }

    pr_info("Nunchuk device added\n");

    return 0;
}

static int nunchuk_i2c_remove(struct i2c_client *client)
{
    pr_info("Nunchuk device removed\n");
    return 0;
}

struct of_device_id nunchuk_of_match[] = {
    {.compatible = "nintendo,nunchuk", .data = (const void *)NUNCHUCK_DEVICE},
    { } /* NULL terminated */
};

MODULE_DEVICE_TABLE(of, nunchuk_of_match);

static struct i2c_driver nunchuck_i2c_driver = {
    .probe = nunchuk_i2c_probe,
    .remove = nunchuk_i2c_remove,
    .driver = {
        .name = "nunchuk",
        .of_match_table = nunchuk_of_match
    }
};

module_i2c_driver(nunchuck_i2c_driver);

/* Module Metadata*/
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Popa <sebastian.popa@raptor-technologies.ro>");
MODULE_DESCRIPTION("Nintendo Wii Nunchuk i2c input driver");

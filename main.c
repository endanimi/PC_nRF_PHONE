/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/settings/settings.h>

#include <dk_buttons_and_leds.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

#define NON_CONNECTABLE_ADV_IDX 0
#define CONNECTABLE_ADV_IDX     1

#define RUN_STATUS_LED          DK_LED1
#define CON_STATUS_LED          DK_LED2
#define RUN_LED_BLINK_INTERVAL  1000

#define NON_CONNECTABLE_DEVICE_NAME "AT Device"

static struct bt_le_ext_adv *ext_adv[CONFIG_BT_EXT_ADV_MAX_ADV_SET];
static const struct bt_le_adv_param *non_connectable_adv_param =
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_NAME,
			0x140, /* 200 ms */
			0x190, /* 250 ms */
			NULL);

static struct bt_data non_connectable_data[] = {
BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, "data")
};

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 32

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	while (uart_irq_rx_ready(uart_dev)) {

		uart_fifo_read(uart_dev, &c, 1);

		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}

static void adv_connected_cb(struct bt_le_ext_adv *adv,
			     struct bt_le_ext_adv_connected_info *info)
{
	printk("Advertiser[%d] %p connected conn %p\n", bt_le_ext_adv_get_index(adv),
		adv, info->conn);
}

static const struct bt_le_ext_adv_cb adv_cb = {
	.connected = adv_connected_cb
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	dk_set_led_on(CON_STATUS_LED);

	printk("Connected %s\n", addr);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	dk_set_led_off(CON_STATUS_LED);

	printk("Disconnected: %s (reason %u)\n", addr, reason);

}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static int advertising_set_create(struct bt_le_ext_adv **adv,
				  const struct bt_le_adv_param *param,
				  const struct bt_data *ad, size_t ad_len)
{
	int err;
	struct bt_le_ext_adv *adv_set;

	err = bt_le_ext_adv_create(param, &adv_cb,
				   adv);
	if (err) {
		return err;
	}

	adv_set = *adv;

	printk("Created adv: %p\n", adv_set);

	err = bt_le_ext_adv_set_data(adv_set, ad, ad_len,
				     NULL, 0);
	if (err) {
		printk("Failed to set advertising data (err %d)\n", err);
		return err;
	}

	return bt_le_ext_adv_start(adv_set, BT_LE_EXT_ADV_START_DEFAULT);
}

static int non_connectable_adv_create(void)
{
	int err;

	err = bt_set_name(NON_CONNECTABLE_DEVICE_NAME);
	if (err) {
		printk("Failed to set device name (err %d)\n", err);
		return err;
	}

	err = advertising_set_create(&ext_adv[NON_CONNECTABLE_ADV_IDX], non_connectable_adv_param,
				     non_connectable_data, ARRAY_SIZE(non_connectable_data));
	if (err) {
		printk("Failed to create a non-connectable advertising set (err %d)\n", err);
	}

	return err;
}

void main(void)
{
	int err;
	int blink_status = 0;
	char latit[MSG_SIZE];
	char longit[MSG_SIZE];

	/* configure interrupt and callback to receive data */
	uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	uart_irq_rx_enable(uart_dev);

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return;
	}

	print_uart("Please provide latitude\r\n");
	while (k_msgq_get(&uart_msgq, &latit, Z_FOREVER) != 0) {
		   k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
	strcat(latit,":");
	print_uart("Latitude provided: ");
	print_uart(latit);
	print_uart("\r\n");

	print_uart("Please provide longitude\r\n");
	while (k_msgq_get(&uart_msgq, &longit, Z_FOREVER) != 0) {
		   k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
	print_uart("Longitude provided: ");
	print_uart(longit);
	print_uart("\r\n");

	strcat(latit,longit);
	print_uart("Coordinates provided: ");
	print_uart(latit);
	print_uart("\r\n");

	non_connectable_data[0].data_len = 24;
	non_connectable_data[0].data = latit;
	
	err = dk_leds_init();
	if (err) {
		printk("LEDs init failed (err %d)\n", err);
		return;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	err = non_connectable_adv_create();
	if (err) {
		return;
	}

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}

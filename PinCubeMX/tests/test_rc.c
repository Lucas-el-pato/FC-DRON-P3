/**
 ******************************************************************************
 * @file    test_rc.c
 * @brief   Test del receptor RC SuperD ELRS por UART4 @ 420000 (CRSF).
 ******************************************************************************
 */

#include "console.h"
#include "driver_crsf.h"

void test_rc_run(void)
{
    console_banner("RC SuperD ELRS (CRSF 420000)");

    crsf_init();

    /* 1. Esperar el primer frame con CRC OK durante 3 s. */
    console_print("Esperando frame CRSF valido (timeout 3s)...\r\n");

    crsf_frame_t frame;
    crsf_status_t st = crsf_read_frame(&frame, 3000u);
    if (st != CRSF_OK) {
        console_printf("crsf_read_frame fallo (codigo=%d)\r\n", (int)st);
        console_result(false, "Sin frames CRSF validos");
        while (1) { HAL_Delay(500u); }
    }

    console_printf("Frame OK: addr=0x%02X type=0x%02X len=%u CRC=0x%02X\r\n",
                   (unsigned)frame.device_addr,
                   (unsigned)frame.type,
                   (unsigned)frame.len,
                   (unsigned)frame.crc_rx);
    console_result(true, "Receptor ELRS detectado");

    /* 2. Bucle de impresion de canales si llega frame tipo 0x16. */
    while (1) {
        st = crsf_read_frame(&frame, 1000u);
        if (st != CRSF_OK) {
            console_printf("(timeout o CRC malo, codigo=%d)\r\n", (int)st);
            console_led_fail();
            continue;
        }
        console_led_pass();

        if (frame.type == CRSF_TYPE_RC_CHANNELS) {
            crsf_channels_t ch;
            if (crsf_decode_channels(&frame, &ch) == CRSF_OK) {
                console_printf("CH1..8:  %4u %4u %4u %4u %4u %4u %4u %4u\r\n",
                               ch.ch[0], ch.ch[1], ch.ch[2], ch.ch[3],
                               ch.ch[4], ch.ch[5], ch.ch[6], ch.ch[7]);
            }
        } else {
            console_printf("Frame tipo 0x%02X (len=%u) ignorado\r\n",
                           (unsigned)frame.type, (unsigned)frame.len);
        }
    }
}

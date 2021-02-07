/*
 * Analog Device AD7150 CDC
 *
 * Copyright (C) 2020 Jonathan Cameron <jic23@kernel.org>
 * Based on tmp105
 * Copyright (C) 2008 Nokia Corporation
 * Written by Andrzej Zaborowski <andrew@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "ad7150.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/module.h"

/* read only registers */
#define AD7150_STATUS_REG            0x00
#define AD7150_CH1_DATA_HIGH_REG     0x01
#define AD7150_CH1_DATA_LOW_REG      0x02
#define AD7150_CH2_DATA_HIGH_REG     0x03
#define AD7150_CH2_DATA_LOW_REG      0x04
#define AD7150_CH1_AVG_HIGH_REG      0x05
#define AD7150_CH1_AVG_LOW_REG       0x06
#define AD7150_CH2_AVG_HIGH_REG      0x07
#define AD7150_CH2_AVG_LOW_REG       0x08
/* rw registers */
#define AD7150_CH1_SENS_TH_HIGH_REG  0x09
#define AD7150_CH1_TIMEOUT_REG       0x0A
#define AD7150_CH1_SETUP_REG         0x0B
#define AD7150_CH2_SENS_TH_HIGH_REG  0x0C
#define AD7150_CH2_TIMEOUT_REG       0x0D /* also threshold low reg */
#define AD7150_CH2_SETUP_REG         0x0E
#define AD7150_CONFIG_REG            0x0F
#define AD7150_PWR_DWN_TIMER_REG     0x10
#define AD7150_CH1_CAPDAC_REG        0x11
#define AD7150_CH2_CAPDAC_REG        0x12
/* read only registers */
#define AD7150_SERIAL_3_REG          0x13
#define AD7150_SERIAL_2_REG          0x14
#define AD7150_SERIAL_1_REG          0x15
#define AD7150_SERIAL_0_REG          0x16
#define AD7150_ID_REG                0x17

static void ad7150_get_capacitance(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    AD7150State *s = AD7150(obj);
    int index = (size_t)opaque;
    int64_t value = s->cap[index];

    visit_type_int(v, name, &value, errp);
}

/* Units are 0.001 centigrades relative to 0 C.  s->temperature is 8.8
 * fixed point, so units are 1/256 centigrades.  A simple ratio will do.
 */
static void ad7150_set_capacitance(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    AD7150State *s = AD7150(obj);
    int64_t capacitance;
    int index = (size_t)opaque;

    if (!visit_type_int(v, name, &capacitance, errp)) {
        return;
    }
    if (capacitance < 0 || capacitance >= (1 << 12)) {
        error_setg(errp, "value %" PRId64 " counts is out of range",
                   capacitance);
        return;
    }
    s->cap[index] = capacitance;

}

static void ad7150_get_capacitance_avg(Object *obj, Visitor *v,
                                       const char *name,
                                       void *opaque, Error **errp)
{
    AD7150State *s = AD7150(obj);
    int index = (size_t)opaque;
    int64_t value = s->cap_avg[index];

    visit_type_int(v, name, &value, errp);
}

static void ad7150_get_line(Object *obj, Visitor *v,
                                       const char *name,
                                       void *opaque, Error **errp)
{
//    AD7150State *s = AD7150(obj);
    int64_t state = 3;//qemu_get_irq(s->pin);
    visit_type_int(v, name, &state, errp);
}

static void ad7150_set_line(Object *obj, Visitor *v,
                            const char *name,
                            void *opaque, Error **errp)
{
    AD7150State *s = AD7150(obj);
    int64_t state;
    int index = (size_t)opaque;
    if (!visit_type_int(v, name, &state, errp)) {
        return;
    }

    s->out[index] = !!state;

    if (state != 0)
        qemu_set_irq(s->pin[index], 1);
    else
        qemu_set_irq(s->pin[index], 0);
}
static void ad7150_set_capacitance_avg(Object *obj, Visitor *v,
                                       const char *name,
                                       void *opaque, Error **errp)
{
    AD7150State *s = AD7150(obj);
    int64_t capacitance;
    int index = (size_t)opaque;

    if (!visit_type_int(v, name, &capacitance, errp)) {
        return;
    }
    if (capacitance < 0 || capacitance >= (1 << 12)) {
        error_setg(errp, "value %" PRId64 " counts is out of range",
                   capacitance);
        return;
    }
    s->cap_avg[index] = capacitance;

}

static void ad7150_read(AD7150State *s)
{
    s->len = 0;
    /*
     * Device does autoincrement - no easy way of knowing how much
     * we need so build the lot
     */
    switch (s->pointer) {
    case AD7150_STATUS_REG: {
        uint8_t val = 0x0;
        /* reading is always ready - or should we delay? */
        val |= (1 << 0) | (1 << 1);
        val |= s->out[0] ? (1 << 3) : 0;
        val |= s->out[1] ? (1 << 5) : 0;
        /* TODO implement dacstep */
        /* Always last on channel 1 */
        if (!s->pwrdwn)
            val |= (1 << 7);
        s->buf[s->len ++] = val;
        }
        /* Fallthrough */
    /* Note swapped wrt to LE expected for I2C */
    case AD7150_CH1_DATA_HIGH_REG:
        s->buf[s->len ++] = (s->cap[0] >> 4) & 0xFF;
        /* Fallthrough */
    case AD7150_CH1_DATA_LOW_REG:
        s->buf[s->len ++] = (s->cap[0] << 4) & 0xFF;;
        /* Fallthrough */
    /* Note swapped wrt to LE expected for I2C */
    case AD7150_CH2_DATA_HIGH_REG:
        s->buf[s->len ++] = (s->cap[1] >> 4) & 0xFF;
        /* Fallthrough */
    case AD7150_CH2_DATA_LOW_REG:
        s->buf[s->len ++] = (s->cap[1] << 4) & 0xFF;
        /* Fallthrough */
    case AD7150_CH1_AVG_HIGH_REG:
        s->buf[s->len ++] = (s->cap_avg[0] >> 4) & 0xFF;
        /* Fallthrough */
    case AD7150_CH1_AVG_LOW_REG:
        s->buf[s->len ++] = (s->cap_avg[0] << 4) & 0xFF;
        /* Fallthrough */
    case AD7150_CH2_AVG_HIGH_REG:
        s->buf[s->len ++] = (s->cap_avg[1] >> 4) & 0xFF;
        /* Fallthrough */
    case AD7150_CH2_AVG_LOW_REG:
        s->buf[s->len ++] = (s->cap_avg[1] << 4) & 0xFF;
        /* Fallthrough */
    case AD7150_CH1_SENS_TH_HIGH_REG:
        /* Dual purpose register, stored here simply as a register */
        s->buf[s->len ++] = s->sensitivity_or_thresh_high[0];
        /* Fallthrough */
    case AD7150_CH1_TIMEOUT_REG:
        s->buf[s->len ++] = s->timeout_or_thresh_low[0];
        /* Fallthrough */
    case AD7150_CH1_SETUP_REG:
        /*TODO */
        s->buf[s->len ++] = 0x00;//todo
        /* Fallthrough */
    case AD7150_CH2_SENS_TH_HIGH_REG:
        s->buf[s->len ++] = s->sensitivity_or_thresh_high[1];
        /* Fallthrough */
    case AD7150_CH2_TIMEOUT_REG:
        s->buf[s->len ++] = s->timeout_or_thresh_low[1];
        /* Fallthrough */
    case AD7150_CH2_SETUP_REG:
        s->buf[s->len ++] = 0x00;//todo
        /* Fallthrough */
    case AD7150_CONFIG_REG:
        s->buf[s->len ++] = 0x00;//todo
        /* Fallthrough */
    case AD7150_PWR_DWN_TIMER_REG:
        s->buf[s->len ++] = 0x00;//todo
        /* Fallthrough */
    case AD7150_CH1_CAPDAC_REG:
        s->buf[s->len ++] = 0x00;//todo
        /* Fallthrough */
    case AD7150_CH2_CAPDAC_REG:
        s->buf[s->len ++] = 0x00;//todo
        /* Fallthrough */;
    case AD7150_SERIAL_3_REG:
        s->buf[s->len ++] = 0x00;//todo
        /* Fallthrough */
    case AD7150_SERIAL_2_REG:
        s->buf[s->len ++] = 0x00;//todo
        /* Fallthrough */
    case AD7150_SERIAL_1_REG:
        s->buf[s->len ++] = 0x00;//todo
        /* Fallthrough */
    case AD7150_SERIAL_0_REG:
        s->buf[s->len ++] = 0x00;//todo
        /* Fallthrough */
    case AD7150_ID_REG:
        s->buf[s->len ++] = 0x13;
        break;
    }
}

static void ad7150_write(AD7150State *s)
{
    switch (s->pointer + s->len - 1) {
    case 0://AD7150_REG_TEMPERATURE:
        break;
    case AD7150_CH1_SENS_TH_HIGH_REG:
        s->sensitivity_or_thresh_high[0] = s->buf[s->len - 1];
        printf("wrote thresh high %x %d\n", s->buf[s->len - 1], s->len - 1);
        break;
    case AD7150_CH1_TIMEOUT_REG:
        s->timeout_or_thresh_low[0] = s->buf[s->len - 1];
        printf("wrote %x %d\n", s->buf[s->len - 1], s->len - 1);
        break;
    case AD7150_CH1_SETUP_REG:
    case AD7150_CH2_SENS_TH_HIGH_REG:
        s->sensitivity_or_thresh_high[1] = s->buf[s->len - 1];
        break;
    case AD7150_CH2_TIMEOUT_REG:
        s->timeout_or_thresh_low[1] = s->buf[s->len - 1];
        break;
    case AD7150_CH2_SETUP_REG:
    case AD7150_CONFIG_REG:
    case AD7150_PWR_DWN_TIMER_REG:
    case AD7150_CH1_CAPDAC_REG:
    case AD7150_CH2_CAPDAC_REG:
        break;
    }
}

static uint8_t ad7150_rx(I2CSlave *i2c)
{
    AD7150State *s = AD7150(i2c);

    //In theory can get a read of all registers?
    //Hmm. How to prepare for that?
    if (s->len < 4) {
        return s->buf[s->len ++];
    } else {
        return 0xff;
    }
}

static int ad7150_tx(I2CSlave *i2c, uint8_t data)
{
    AD7150State *s = AD7150(i2c);

    if (s->len == 0) {
        s->pointer = data;
        s->len++;
    } else {
        if (s->len <= 2) {
            s->buf[s->len - 1] = data;
        }
        ad7150_write(s);
        s->len++;        
    }

    return 0;
}

static int ad7150_event(I2CSlave *i2c, enum i2c_event event)
{
    AD7150State *s = AD7150(i2c);

    if (event == I2C_START_RECV) {
        ad7150_read(s);
    }

    s->len = 0;
    return 0;
}

static int ad7150_post_load(void *opaque, int version_id)
{
//    AD7150State *s = opaque;

    return 0;
}

static const VMStateDescription vmstate_ad7150 = {
    .name = "AD7150",
    .version_id = 0,
    .minimum_version_id = 0,
    .post_load = ad7150_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(len, AD7150State),
        VMSTATE_UINT8_ARRAY(buf, AD7150State, 0x18),
        VMSTATE_UINT8(pointer, AD7150State),
        VMSTATE_I2C_SLAVE(i2c, AD7150State),
        VMSTATE_END_OF_LIST()
    },
};

static void ad7150_reset(I2CSlave *i2c)
{
    AD7150State *s = AD7150(i2c);

    s->pwrdwn = false;
    s->timeout_or_thresh_low[0] = 0x86;
    s->timeout_or_thresh_low[1] = 0x86;
    s->cap[0] = 3200;
    s->cap[1] = 3301;
    s->cap_avg[0] = 3200;
    s->cap_avg[1] = 3302;
    s->pointer = 0;
}

static void ad7150_realize(DeviceState *dev, Error **errp)
{
    I2CSlave *i2c = I2C_SLAVE(dev);
    AD7150State *s = AD7150(i2c);

    qdev_init_gpio_out(&i2c->qdev, &s->pin[0], 1);
    qdev_init_gpio_out(&i2c->qdev, &s->pin[1], 1);

    ad7150_reset(&s->i2c);
}

static void ad7150_initfn(Object *obj)
{
    object_property_add(obj, "ch1_capacitance", "int",
                        ad7150_get_capacitance,
                        ad7150_set_capacitance, NULL, 0);
    object_property_add(obj, "ch1_capacitance_avg", "int",
                        ad7150_get_capacitance_avg,
                        ad7150_set_capacitance_avg, NULL, 0);
    object_property_add(obj, "ch2_capacitance", "int",
                        ad7150_get_capacitance,
                        ad7150_set_capacitance, NULL, (void *)1);
    object_property_add(obj, "ch2_capacitance_avg", "int",
                        ad7150_get_capacitance_avg,
                        ad7150_set_capacitance_avg, NULL, (void *)1);
    object_property_add(obj, "ch1_event_state", "int",
                        ad7150_get_line,
                        ad7150_set_line, NULL, NULL);
    object_property_add(obj, "ch2_event_state", "int",
                        ad7150_get_line,
                        ad7150_set_line, NULL, (void *)1);
}

static void ad7150_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = ad7150_realize;
    k->event = ad7150_event;
    k->recv = ad7150_rx;
    k->send = ad7150_tx;
    dc->vmsd = &vmstate_ad7150;
}

static const TypeInfo ad7150_info = {
    .name          = TYPE_AD7150,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(AD7150State),
    .instance_init = ad7150_initfn,
    .class_init    = ad7150_class_init,
};

static void ad7150_register_types(void)
{
    type_register_static(&ad7150_info);
}

type_init(ad7150_register_types)

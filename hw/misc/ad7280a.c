/*
 * Analog Device AD7280a
 *
 * Copyright (c) 2021 Jonathan Cameron <jic23@kernel.org>
 *
 * This code is licensed under the GNU GPL v2.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 *
 * Note this is a bunch of hacks to test the corners of the ad7280a driver
 * that cared about. It is neither complete nor necessarily correct.
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "ui/console.h"
#include "qom/object.h"
/* required table size for crc8 algorithm */
#define CRC8_TABLE_SIZE			256
#define NUM_DEVS 3

#define AD7280A_CELL_VOLT_1_REG		0x00
#define AD7280A_CELL_VOLT_2_REG		0x01
#define AD7280A_CELL_VOLT_3_REG		0x02
#define AD7280A_CELL_VOLT_4_REG		0x03
#define AD7280A_CELL_VOLT_5_REG		0x04
#define AD7280A_CELL_VOLT_6_REG		0x05
#define AD7280A_AUX_VOLT_1_REG		0x06
#define AD7280A_AUX_VOLT_2_REG		0x07
#define AD7280A_AUX_VOLT_3_REG		0x08
#define AD7280A_AUX_VOLT_4_REG		0x09
#define AD7280A_AUX_VOLT_5_REG		0x0a
#define AD7280A_AUX_VOLT_6_REG		0x0b
#define AD7280A_SELF_TEST_REG		0x0c
#define AD7280A_CONTROL_H_REG		0x0d
#define AD7280A_CONTROL_L_REG		0x0e
#define AD7280A_CELL_OVER_REG		0x0f
#define AD7280A_CELL_UNDER_REG		0x10
#define AD7280A_AUX_OVER_REG		0x11
#define AD7280A_AUX_UNDER_REG		0x12
#define AD7280A_ALERT_REG		0x13
#define AD7280A_CELL_BALANCE_REG	0x14
#define AD7280A_CB1_TIMER_REG		0x15
#define AD7280A_CB2_TIMER_REG		0x16
#define AD7280A_CB3_TIMER_REG		0x17
#define AD7280A_CB4_TIMER_REG		0x18
#define AD7280A_CB5_TIMER_REG		0x19
#define AD7280A_CB6_TIMER_REG		0x1a
#define AD7280A_PD_TIMER_REG		0x1b
#define AD7280A_READ_REG		0x1c
#define AD7280A_CNVST_CONT_REG		0x1d

struct AD7280AState {
    SSIPeripheral ssidev;

    int cycle;
    int output;
    uint32_t val;
    uint32_t read[NUM_DEVS];
    uint32_t reg_addr;/* used for chained reads */
    uint32_t rb_dev_addr;
    uint32_t rb_all_disabled;
    uint32_t chain;
    uint32_t chan;
    int index;
    /* 3 devices */
    uint16_t registers[NUM_DEVS][0x1e];
    unsigned char			crc_tab[CRC8_TABLE_SIZE];
};

#define TYPE_AD7280A "ad7280a"
OBJECT_DECLARE_SIMPLE_TYPE(AD7280AState, AD7280A)


static unsigned char ad7280_calc_crc8(unsigned char *crc_tab, unsigned int val)
{
	unsigned char crc;

	crc = crc_tab[val >> 16 & 0xFF];
	crc = crc_tab[crc ^ (val >> 8 & 0xFF)];

	return  crc ^ (val & 0xFF);
}

static uint32_t ad7280a_transfer(SSIPeripheral *dev, uint32_t value)
{
    AD7280AState *s = AD7280A(dev);
    uint32_t v;
    uint32_t out;
    int i;

    /* Runs every byte */
    /* Check here that the device is enabled - if not we may need to update the read out? */

    /* Handle the chaining off the end carefully */
    /* Verify chained element enabled, if not skip */
    for (i = s->chain; i < NUM_DEVS; i++) {
        if (((s->registers[i][AD7280A_CONTROL_H_REG] >> 4) & 0x3) != 0x3)
            break;
    }
    if (s->chain != i) {
        printf("HAD TO ADVANCE THE CHAIN - Why %x %x?\n", s->chain, i);
    }
    s->chain = i;
    if (s->chain < NUM_DEVS) {
        out = (s->read[s->chain]  >> (8 * s->index)) & 0xff;
    } else
        out = 0;
    
//    printf("index %#x, out %#x for chain %#x\n", s->index, out, s->chain);
    if (s->index == 0)
        s->val = 0;
    s->val |= (value << (8 * s->index));
    
    /* So.. seems we have an 8 bit write each time - so build it up */
    if (s->index == 3) {
        unsigned int revdevaddr_in;
        unsigned int devaddr;
        unsigned int addr;
        unsigned int val;
        unsigned int all;
        unsigned char crc;

//        printf("AD7280 transfer %#x\n", s->val);
        /* Big endian value */
        v = be32_to_cpu(s->val);
        revdevaddr_in = v >> 27;

        //check the crcs coming in...
        crc = ad7280_calc_crc8(s->crc_tab, v >> 11);
        if (((v >> 3) & 0xFF) != crc) {
            printf("crc fail on write ignore it?\n");
        } else {
            //printf("CRC good ;) %#x\n", crc);
        }
        devaddr = ((revdevaddr_in & 0x10) >> 4) |
            ((revdevaddr_in & 0x08) >> 2) |
            (revdevaddr_in & 0x4) |
            ((revdevaddr_in & 0x2) << 2) |
            ((revdevaddr_in & 0x1) << 4);
        addr = (v >> 21) & 0x7f;
        val = (v >> 13) & 0xFF;
        all = (v >> 12) & 0x1;
        if (all) {
            int i;
            for (i = 0; i < NUM_DEVS; i++)
                s->registers[i][addr] = val;
        } else {
            if (devaddr == 0x1f) {
//                printf("Skip as no write intended\n");
            } else if (devaddr >= NUM_DEVS) {
                
                printf("WTF - DEVADDR TOO HIGH %x\n", devaddr);
            } else {
                s->registers[devaddr][addr] = val;
            }
        }
        
        printf("devaddr = %#x, addr=%#x, val=%#x, all=%#x\n", devaddr, addr, val, all);

        if (devaddr != 0x1f) {
            /* last part if crc which I haven't done yet...  */
            switch (addr) {
            case AD7280A_CONTROL_H_REG:
                s->chain = 0; ///REALY NOT SURE
                s->chan = 0;
                printf("Control write H: Conv Inputs= ");
                switch ((val >> 6) & 0x3) {
                case 0:
                    printf("[6 Cells + 6 Aux ADCs]");
                    break;
                case 1:
                    printf("[6 Cells + 1, 3, 5 Aux ADCs]");
                    break;
                case 2:
                    printf("[6 Cells only]");
                    break;
                case 3:
                    printf("[ADC Self Test]");
                    break;
                }
                printf(" Read Conv Results=");
                switch ((val >> 4) & 0x3) {
                case 0:
                    printf("[6 Cells + 6 Aux ADCs]");
                    break;
                case 1:
                    printf("[6 Cells + 1, 3, 5 Aux ADCs]");
                    break;
                case 2:
                    printf("[6 Cells only]");
                    break;
                case 3:
                    printf("[NONE]");
                    break;
                }
                printf(" ConvStart=");
                switch ((val >> 3) & 0x1) {
                case 0:
                    printf("[falling CNVSTBAR]");
                    break;
                case 1:
                        printf("[rising CSBAR]");
                    break;;
                }
                printf(" Aver=");
                switch (val & 0x3) {
                case 0:
                    printf("[1]");
                    break;
                case 1:
                    printf("[2]");
                    break;
                case 2:
                    printf("[3]");
                    break;
                case 3:
                    printf("[4]");
                    break;
                }
                printf("\n");
                break;
            case AD7280A_CONTROL_L_REG:
                printf("Control Write L: Reset=[%d] AcqTime=", (val >> 7) & 0x1);
                switch ((val >> 5) & 0x3) {
                case 0:
                    printf("[400ns]");
                    break;
                case 1:
                    printf("[800ns]");
                    break;
                case 2:
                    printf("[1.2us]");
                    break;
                case 3:
                    printf("[1.6us]");
                    break;
                };
                printf(" ThremRes=[%d] LockDevAdr=[%d] IncDevAdr=[%d] DaisyRegRB=[%d]\n",
                       (val >> 3) & 0x1,
                       (val >> 2) & 0x1,
                       (val >> 1) & 0x1,
                       (val >> 0) & 0x1);
                break;
                
            case AD7280A_CB1_TIMER_REG:
                printf("val written is %d\n", val);
                break;
                       
            case AD7280A_READ_REG: { /* Read register */
                uint32_t rv;
                int i;

                s->chain = 0;
                s->chan = 0;
                printf("read register set - so on next cycle pass it back %#x\n", val >> 2);
                s->reg_addr = val >> 2;
                if (!all) {
                    int i = devaddr;
                    uint32_t revdevaddr = ((i & 0x10) >> 4) |
                        ((i & 0x08) >> 2) |
                        (i & 0x4) |
                        ((i & 0x2) << 2) |
                        ((i & 0x1) << 4);
                    //TODO handle multiple enabled mess
                    if (s->reg_addr > AD7280A_AUX_VOLT_6_REG)
                        rv = (revdevaddr << 27) | (s->reg_addr << 21) | (s->registers[i][s->reg_addr] << 13) | (1 << 10);
                    else
                        rv = (revdevaddr << 27) | (s->reg_addr << 23) | (s->registers[i][s->reg_addr] << 11) | (1 << 10);
                    
                    //Need to compute the CRC and apply to above.
                    rv |= ad7280_calc_crc8(s->crc_tab, rv >> 10) << 2;
                    s->read[devaddr] = cpu_to_be32(rv);
                } else {
                    for (i = 0; i < NUM_DEVS; i++) {
                        uint32_t revdevaddr = ((i & 0x10) >> 4) |
                            ((i & 0x08) >> 2) |
                            (i & 0x4) |
                            ((i & 0x2) << 2) |
                            ((i & 0x1) << 4);
                        
                        if (s->reg_addr > AD7280A_AUX_VOLT_6_REG)
                            rv = (revdevaddr << 27) | (s->reg_addr << 21) | (s->registers[i][s->reg_addr] << 13) | (1 << 10);
                        else
                            rv = (revdevaddr << 27) | (s->reg_addr << 23) | (s->registers[i][s->reg_addr] << 11) | (1 << 10);
                        
                        //Need to compute the CRC and apply to above.
                        rv |= ad7280_calc_crc8(s->crc_tab, rv >> 10) << 2;
                        s->read[i] = cpu_to_be32(rv);
                    }
                }
                break;
            }
                //catch special case of read chaining...
                
            default:
                printf("not implemented addr yet\n");
            }
        } else {
            /*
             * Some fun here iff we have address 0 as the read address as that chains through the local registers before jumping
             * to then next device. Which ones depends on what is configured. *sigh*  In that one case, we need to handle chain differently.
             */
            if (s->reg_addr == 0) {
                s->chan++;
                if (s->chan > AD7280A_AUX_VOLT_6_REG) {
                    s->chan = 0;
                    s->chain++;
                }
                /* Handle channel advance within device */
                {
                    uint32_t rv;
                    uint32_t revdevaddr = ((s->chain & 0x10) >> 4) |
                        ((s->chain & 0x08) >> 2) |
                        (s->chain & 0x4) |
                        ((s->chain & 0x2) << 2) |
                        ((s->chain & 0x1) << 4);
                    
                    rv = (revdevaddr << 27) | (s->chan << 23) | (s->registers[i][s->chan] << 11) | (1 << 10);
                    
                    //Need to compute the CRC and apply to above.
                    rv |= ad7280_calc_crc8(s->crc_tab, rv >> 10) << 2;
                    s->read[i] = cpu_to_be32(rv);
                }
            } else {
                s->chain++;
            }
        }
             
    }
    /* Based on read reg set last time \n" */;
    s->index++;
    s->index = s->index % 4;

    return out;
}

static const VMStateDescription vmstate_ad7280a = {
    .name = "ad7280a",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_SSI_PERIPHERAL(ssidev, AD7280AState),
        VMSTATE_INT32(cycle, AD7280AState),
        VMSTATE_END_OF_LIST()
    }
};




/*
 * AD7280 CRC
 *
 * P(x) = x^8 + x^5 + x^3 + x^2 + x^1 + x^0 = 0b100101111 => 0x2F
 */
#define POLYNOM		0x2F
/**
 * crc8_populate_msb - fill crc table for given polynomial in reverse bit order.
 *
 * @table:	table to be filled.
 * @polynomial:	polynomial for which table is to be filled.
 */
static void crc8_populate_msb(uint8_t table[CRC8_TABLE_SIZE], uint8_t polynomial)
{
	int i, j;
	const uint8_t msbit = 0x80;
	uint8_t t = msbit;

	table[0] = 0;

	for (i = 1; i < CRC8_TABLE_SIZE; i *= 2) {
		t = (t << 1) ^ (t & msbit ? polynomial : 0);
		for (j = 0; j < i; j++)
			table[i+j] = table[j] ^ t;
	}
}
static void ad7280a_realize(SSIPeripheral *d, Error **errp)
{
    AD7280AState *s = AD7280A(d);

    crc8_populate_msb(s->crc_tab, POLYNOM);
    s->registers[0][AD7280A_CELL_VOLT_1_REG] = 0x1;
    s->registers[0][AD7280A_CELL_VOLT_2_REG] = 0x2;
    s->registers[0][AD7280A_CELL_VOLT_3_REG] = 0x3;
    s->registers[0][AD7280A_CELL_VOLT_4_REG] = 0x4;
    s->registers[0][AD7280A_CELL_VOLT_5_REG] = 0x5;
    s->registers[0][AD7280A_CELL_VOLT_6_REG] = 0x6;
    s->registers[1][AD7280A_CELL_VOLT_1_REG] = 0x7;
    s->registers[1][AD7280A_CELL_VOLT_2_REG] = 0x8;
    s->registers[1][AD7280A_CELL_VOLT_3_REG] = 0x9;
    s->registers[1][AD7280A_CELL_VOLT_4_REG] = 0xa;
    s->registers[1][AD7280A_CELL_VOLT_5_REG] = 0xb;
    s->registers[1][AD7280A_CELL_VOLT_6_REG] = 0xc;
    s->registers[2][AD7280A_CELL_VOLT_1_REG] = 0xd;
    s->registers[2][AD7280A_CELL_VOLT_2_REG] = 0xe;
    s->registers[2][AD7280A_CELL_VOLT_3_REG] = 0xf;
    s->registers[2][AD7280A_CELL_VOLT_4_REG] = 0x10;
    s->registers[2][AD7280A_CELL_VOLT_5_REG] = 0x11;
    s->registers[2][AD7280A_CELL_VOLT_6_REG] = 0x12;    

    s->registers[0][AD7280A_CELL_UNDER_REG] = 0xDE;
    vmstate_register(NULL, VMSTATE_INSTANCE_ID_ANY, &vmstate_ad7280a, s);
}

static void ad7280a_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = ad7280a_realize;
    k->transfer = ad7280a_transfer;
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo ad7280a_info = {
    .name          = TYPE_AD7280A,
    .parent        = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(AD7280AState),
    .class_init    = ad7280a_class_init,
};

static void ad7280a_register_types(void)
{
    type_register_static(&ad7280a_info);
}

type_init(ad7280a_register_types)

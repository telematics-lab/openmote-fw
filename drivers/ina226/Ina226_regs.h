/**
 * @file       Ina226_regs.hpp
 * @author     Pere Tuset-Peiro     (peretuset@openmote.com)
 * @version    v0.1
 * @date       February, 2019
 * @brief
 *
 * @copyright  Copyright 2019, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef INA226_REGS_HPP_
#define INA226_REGS_HPP_

/* INA226 register definitions */
#define INA226_CONFIG                   ( 0x00 )
#define INA226_SHUNT_VOLTAGE            ( 0x01 )
#define INA226_BUS_VOLTAGE              ( 0x02 )
#define INA226_POWER                    ( 0x03 )
#define INA226_CURRENT                  ( 0x04 )
#define INA226_CALIBRATION              ( 0x05 )
#define INA226_MASK_ENABLE              ( 0x06 )
#define INA226_ALERT_LIMIT              ( 0x07 )
#define INA226_MANUFACTURER_ID          ( 0xFE )
#define INA226_DIE_ID                   ( 0xFF )

/* INA226 average number of bits */
#define INA226_CONFIG_AVG_BIT_1         ( 0 << 9 )
#define INA226_CONFIG_AVG_BIT_4         ( 1 << 9 )
#define INA226_CONFIG_AVG_BIT_16        ( 2 << 9 )
#define INA226_CONFIG_AVG_BIT_64        ( 3 << 9 )
#define INA226_CONFIG_AVG_BIT_128       ( 4 << 9 )
#define INA226_CONFIG_AVG_BIT_256       ( 5 << 9 )
#define INA226_CONFIG_AVG_BIT_512       ( 6 << 9 )
#define INA226_CONFIG_AVG_BIT_1024      ( 7 << 9 )

/* INA226 VSHUNT conversion time */
#define INA226_CONFIG_CT_VSH_140US      ( 0 << 3 )
#define INA226_CONFIG_CT_VSH_204US      ( 1 << 3 )
#define INA226_CONFIG_CT_VSH_332US      ( 2 << 3 )
#define INA226_CONFIG_CT_VSH_588US      ( 3 << 3 )
#define INA226_CONFIG_CT_VSH_1100US     ( 4 << 3 )
#define INA226_CONFIG_CT_VSH_2116US     ( 5 << 3 )
#define INA226_CONFIG_CT_VSH_4156US     ( 6 << 3 )
#define INA226_CONFIG_CT_VSH_8244US     ( 7 << 3 )

/* INA226 VBUS conversion time */
#define INA226_CONFIG_CT_VBUS_140US     ( 0 << 6 )
#define INA226_CONFIG_CT_VBUS_204US     ( 1 << 6 )
#define INA226_CONFIG_CT_VBUS_332US     ( 2 << 6 )
#define INA226_CONFIG_CT_VBUS_588US     ( 3 << 6 )
#define INA226_CONFIG_CT_VBUS_1100US    ( 4 << 6 )
#define INA226_CONFIG_CT_VBUS_2116US    ( 5 << 6 )
#define INA226_CONFIG_CT_VBUS_4156US    ( 6 << 6 )
#define INA226_CONFIG_CT_VBUS_8244US    ( 7 << 6 )

/* INA226 operation mode */
#define INA226_CONFIG_MODE_SHUTDOWN1    ( 0 << 0 )
#define INA226_CONFIG_MODE_SV_TRIG      ( 1 << 0 )
#define INA226_CONFIG_MODE_BV_TRIG      ( 2 << 0 )
#define INA226_CONFIG_MODE_SBV_TRIG     ( 3 << 0 )
#define INA226_CONFIG_MODE_SHUTDOWN2    ( 4 << 0 )
#define INA226_CONFIG_MODE_SV_CONT      ( 5 << 0 )
#define INA226_CONFIG_MODE_BV_CONT      ( 6 << 0 )
#define INA226_CONFIG_MODE_SBV_CONT     ( 7 << 0 )

#define INA226_CONFIG_DEFAULT           ( 0x4000 )

#define INA226_CONFIG_DEFAULT_MSB       ( 0x41 )
#define INA226_CONFIG_DEFAULT_LSB       ( 0x27 )

#define INA226_MANUFACTURER_ID_MSB      ( 0x54 )
#define INA226_MANUFACTURER_ID_LSB      ( 0x49 )

#define INA226_RSHUNT_DEFAULT           ( 10000 )

#endif /* AT86RF215_REGS_HPP_ */

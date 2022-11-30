/**
 * @file       Aes.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2016
 * @brief
 *
 * @copyright  Copyright 2016, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef AES_HPP_
#define AES_HPP_

#include <stdint.h>

class Aes {

friend class InterruptHandler;

public:
  Aes();
  void enable(void);
  void reset(void);
  bool sleep(void);
  bool wakeup(void);
  bool loadKey(uint8_t* key, uint16_t length);
  bool encrypt(uint8_t* input, uint8_t* output, uint16_t* length);
  bool decrypt(uint8_t* input, uint8_t* output, uint16_t* length);
protected:
  void interruptHandler(void);
private:
  bool processBuffer(uint8_t* input, uint8_t* output, uint16_t* length, bool encrypt);
  bool processBlock(uint8_t* input, uint8_t* output, uint8_t key, bool encrypt);
private:
  bool enabled_;
	uint8_t key_[16];
};

#endif /* AES_HPP_ */

/**
 * @file       Rendezvous.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef RENDEZVOUS_HPP_
#define RENDEZVOUS_HPP_

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "event_groups.h"

typedef uint32_t RendezvousId;

class Rendezvous
{
public:
  Rendezvous(void);
  ~Rendezvous(void);
  bool getId(RendezvousId* id);
  bool sync(RendezvousId id);
  bool sync(RendezvousId id, uint32_t milliseconds);
protected:
  EventGroupHandle_t handle_;

  uint32_t counter_;
  uint32_t lastId_;
  uint32_t maxId_;
};

#endif /* RENDEZVOUS_HPP_ */

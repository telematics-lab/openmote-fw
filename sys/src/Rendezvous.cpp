/**
 * @file       Rendezvous.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "CriticalSection.hpp"
#include "Rendezvous.hpp"

/*================================ define ===================================*/

#define MAX_RENDEZVOUS_TASKS        ( 32 )

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Rendezvous::Rendezvous(void):
	counter_(0), lastId_(0), maxId_(0)
{
  handle_ = xEventGroupCreate();
  if (handle_ == NULL)
  {
    while(true);
  }
}

Rendezvous::~Rendezvous(void)
{
  vEventGroupDelete(handle_);
}

bool Rendezvous::getId(RendezvousId* id)
{
  CriticalSection cs;

  if (counter_ < MAX_RENDEZVOUS_TASKS && id != NULL)
  {
    lastId_ = 1 << (counter_);
    *id = lastId_;
    counter_++;
    maxId_ |= lastId_;
    return true;
  }

  return false;
}

bool Rendezvous::sync(RendezvousId id)
{
  xEventGroupSync(handle_, id, maxId_, portMAX_DELAY);
  return true;
}

bool Rendezvous::sync(RendezvousId id, uint32_t milliseconds)
{
	EventBits_t uxReturn;

	TickType_t timeout = milliseconds / portTICK_RATE_MS;
	uxReturn = xEventGroupSync(handle_, id, maxId_, timeout);

  if ((uxReturn & maxId_) == maxId_)
  {
      return true;
  }

  return false;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

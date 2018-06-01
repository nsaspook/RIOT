
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "config.h"
#include "app.h"
#include "spi.h"
#include "link.h"

static struct LINK_DATA l_data;

bool Write_Link_Packet(const uint8_t *data, bool start)
{
    (void) data;
    (void) start;
    return false;
}

struct LINK_DATA *Read_Link_Packet(const uint8_t *data)
{
    return memcpy(&l_data, data, LINK_BYTES);
}

struct LINK_DATA *Get_Link_Packet(void)
{
    return &l_data;
}

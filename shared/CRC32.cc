#include "CRC32.hh"

namespace jaldimac {

static bool initialized = false;
static uint32_t crctable[256];

static void gentable();

// Initialize CRC32 table if needed
void initcrc32()
{
	if (! initialized)
		gentable();
}

// Calculate CRC32
uint32_t crc32(const uint8_t* block, unsigned length)
{
   register uint32_t crc;
   uint32_t i;

   crc = 0xFFFFFFFF;
   for (i = 0; i < length; i++)
   {
      crc = ((crc >> 8) & 0x00FFFFFF) ^ crctable[(crc ^ *block++) & 0xFF];
   }
   return (crc ^ 0xFFFFFFFF);
}

// Generate CRC32 table
static void gentable()
{
   uint32_t crc, poly;
   int i, j;

   poly = 0xEDB88320L;
   for (i = 0; i < 256; i++)
   {
      crc = i;

      for (j = 8; j > 0; j--)
      {
	 if (crc & 1)
	 {
	    crc = (crc >> 1) ^ poly;
	 }
	 else
	 {
	    crc >>= 1;
	 }
      }

      crctable[i] = crc;
   }
}

}

#include "stdio.h"

void * memset( void * p, int c, size_t n )
{
    uint8_t  * sp;
    uint64_t * lp;
    uint64_t   u64;
    uint8_t    u8;
    
    u8  = ( uint8_t )c;
    u64 = ( uint64_t )c;
    u64 = ( u64 << 32 ) | u64;
    sp  = ( uint8_t * )p;
    
    while( n-- && ( ( ( uint64_t )sp & ( uint64_t )-8 ) < ( uint64_t )sp ) )
    {
        *( sp++ ) = u8;
    }
    
    lp  = ( uint64_t * )( ( void * )sp );
    
    while( ( n / 8 ) > 0 )
    {
        *( lp++ ) = u64;
        n        -= 8;
    }
    
    sp = ( uint8_t * )( ( void * )lp );
    
    while( n-- )
    {
        *( sp++ ) = u8;
    }
    
    return p;
}
#include "error/error.h"
//----------------------
volatile error_t _error;
//---------------------------
uint16_t ERROR_checksum(void)
{
    return _error.checksum;
}
//--------------------------
uint16_t ERROR_command(void)
{
    return _error.command;
}
//-----------------------------
uint16_t ERROR_no_process(void)
{
    return _error.no_process;
}
//--------------------------
uint16_t ERROR_overrun(void)
{
    return _error.overrun;
}
//--------------------------
uint16_t ERROR_request(void)
{
    return _error.request;
}
//--------------------------
uint16_t ERROR_timeout(void)
{
    return _error.timeout;
}
//---------------------------
void ERROR_checksum_inc(void)
{
    _error.checksum++;
}
//--------------------------
void ERROR_command_inc(void)
{
    _error.command++;
}
//-----------------------------
void ERROR_no_process_inc(void)
{
    _error.no_process++;
}
//--------------------------
void ERROR_overrun_inc(void)
{
    _error.overrun++;
}
//--------------------------
void ERROR_request_inc(void)
{
    _error.request++;
}
//--------------------------
void ERROR_timeout_inc(void)
{
    _error.timeout++;
}

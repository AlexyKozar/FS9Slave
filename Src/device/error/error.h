#ifndef ERROR_H
    #define ERROR_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    //---------------------
    typedef struct _error_t
    {
        uint16_t request; // request counter
        uint16_t command; // commnad error
        uint16_t checksum; // checksum error
        uint16_t no_process; // no command processing error
        uint16_t overrun; // overrun error
        uint16_t timeout; // timeout error
    } error_t;
    /*!
     * /brief Error counter function
     * /return Error counter
     */
    uint16_t ERROR_checksum(void);
    uint16_t ERROR_command(void);
    uint16_t ERROR_no_process(void);
    uint16_t ERROR_overrun(void);
    uint16_t ERROR_request(void);
    uint16_t ERROR_timeout(void);
    /*!
     * /brief Error counter increment
     */
    void ERROR_checksum_inc(void);
    void ERROR_command_inc(void);
    void ERROR_no_process_inc(void);
    void ERROR_overrun_inc(void);
    void ERROR_request_inc(void);
    void ERROR_timeout_inc(void);
#endif

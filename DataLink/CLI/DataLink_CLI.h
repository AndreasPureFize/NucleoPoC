#ifndef CLI_DATALINK_CLI_H_
#define CLI_DATALINK_CLI_H_

#include <stdbool.h>
#include <stdint.h>

/* ================================
 * Command model (parser output)
 * ================================ */

typedef enum {
    CMD_NONE = 0,
    CMD_CONFIG,
    CMD_READ,
    CMD_RESET,
    CMD_SET,
    CMD_HELP,
    CMD_EXIT
} cli_primary_t;

typedef enum {
    SUB_NONE = 0,
    SUB_CHANNEL,
    SUB_POWER,
    SUB_CONFIG,
    SUB_DATA,
    SUB_ERRORS,
    SUB_OUTPUT,
    SUB_DEFAULT
} cli_secondary_t;

/* Parsed command with already-validated arguments */
typedef struct {
    cli_primary_t   primary;
    cli_secondary_t secondary;
    bool            has_int;
    int             ival;      /* e.g., CHANNEL (1..4), OUTPUT/DEFAULT (0|1) */
    bool            has_float;
    float           fval;      /* e.g., POWER (0.5..1.0) */
} cli_command_t;

/* ================================
 * Execute/Result model
 * ================================ */

typedef enum {
    CLI_RES_OK = 0,
    CLI_RES_INVALID_CMD,
    CLI_RES_BAD_ARGS,
    CLI_RES_RANGE_ERR,
    CLI_RES_TIMEOUT,
    CLI_RES_BUSY,
    CLI_RES_PROTOCOL_ERR,
    CLI_RES_LINK_ERR
} cli_result_code_t;

typedef struct {
    cli_result_code_t code;
    int               detail;     /* optional numeric detail (e.g., dl_status_t) */
    /* Optional payload for reads; expand later as needed */
    uint8_t  u8;
    float    f32;
    uint32_t u32;
} cli_result_t;

/* ================================
 * Public CLI API
 * ================================ */

/* Runs a blocking REPL on USART2 until user enters X/EXIT. */
bool ManualMode(void);

/* Utility: converts to uppercase in-place (ASCII). */
void ToUpperCase(char *str);

/* Parser: returns true on success and fills out; false if syntax/usage error. */
bool CLI_Parse(const char *line, cli_command_t *out);

/* Dispatcher: executes parsed command by calling mid-level functions. */
void CLI_Execute(const cli_command_t *cmd, cli_result_t *res);

/* Presenter: prints a human-readable result for a command execution. */
void CLI_PrintResult(const cli_command_t *cmd, const cli_result_t *res);

/* Prompt and line I/O helpers (USART2). */
void CLI_Prompt(void);
/* Returns true if a line was read; handles echo, CR/LF, backspace, truncation. */
bool CLI_ReadLine(char *buf, uint16_t cap);

/* Optional: print brief HELP text. */
void CLI_PrintHelp(void);

#endif /* CLI_DATALINK_CLI_H_ */
